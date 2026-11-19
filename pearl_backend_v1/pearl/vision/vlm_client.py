"""
VLM client — talks to llama.cpp serving Liquid AI LFM2.5-VL-450M over the
OpenAI-compatible /v1/chat/completions endpoint.

Contract: `verify(jpeg_bytes, target) -> VisionResult`.
Fail closed: ANY error (timeout, dead server, HTTP error, malformed JSON,
wrong types) returns match=False, ok=False, error=<string>. Never raises.

Expected model output (strict JSON, optionally inside a ```json fence):
    {"match": true, "confidence": 0.83, "bbox": [0.41, 0.22, 0.63, 0.88]}
bbox is [x_min, y_min, x_max, y_max] normalized 0-1, or null.
"""
from __future__ import annotations

import base64
import json
import re
import time
from typing import Any, Optional

import requests

from .. import config
from ..logging_setup import get_logger, log_event
from ..state import VisionResult

log = get_logger(__name__)

_FENCE = re.compile(r"```(?:json)?\s*(.*?)\s*```", re.S)
_BRACES = re.compile(r"\{.*\}", re.S)

SYSTEM_PROMPT = (
    "You are a visual verifier on a robot. Decide whether the described target is visible in the image. "
    "Reply with ONLY a JSON object: {\"match\": true|false, \"confidence\": 0.0-1.0, "
    "\"bbox\": [x_min, y_min, x_max, y_max] normalized 0-1 or null}. No prose."
)

#: JSON schema handed to llama.cpp (response_format) so decoding is constrained.
RESPONSE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "match": {"type": "boolean"},
        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "bbox": {
            "anyOf": [
                {"type": "null"},
                {"type": "array", "items": {"type": "number", "minimum": 0, "maximum": 1}, "minItems": 4, "maxItems": 4},
            ]
        },
    },
    "required": ["match", "confidence", "bbox"],
}


def parse_vlm_output(text: str) -> tuple[Optional[dict[str, Any]], Optional[str]]:
    """Tolerant parser. Returns (parsed, error). parsed has validated types."""
    if not isinstance(text, str) or not text.strip():
        return None, "empty model output"
    candidate = text.strip()
    m = _FENCE.search(candidate)
    if m:
        candidate = m.group(1)
    try:
        obj = json.loads(candidate)
    except json.JSONDecodeError:
        m2 = _BRACES.search(candidate)
        if not m2:
            return None, f"no JSON object in output: {text[:80]!r}"
        try:
            obj = json.loads(m2.group(0))
        except json.JSONDecodeError as e:
            return None, f"invalid JSON: {e}"
    if not isinstance(obj, dict):
        return None, "output is not a JSON object"

    match = obj.get("match")
    if isinstance(match, str) and match.strip().lower() in ("true", "false"):
        match = match.strip().lower() == "true"
    if not isinstance(match, bool):
        return None, f"'match' not boolean: {obj.get('match')!r}"

    conf = obj.get("confidence", 0.0)
    try:
        conf = float(conf)
    except (TypeError, ValueError):
        return None, f"'confidence' not numeric: {conf!r}"
    conf = max(0.0, min(1.0, conf))

    bbox = obj.get("bbox")
    if bbox is not None:
        if not (isinstance(bbox, list) and len(bbox) == 4):
            bbox = None
        else:
            try:
                bbox = [max(0.0, min(1.0, float(v))) for v in bbox]
            except (TypeError, ValueError):
                bbox = None
            if bbox and (bbox[2] <= bbox[0] or bbox[3] <= bbox[1]):
                bbox = None
    return {"match": match, "confidence": conf, "bbox": bbox}, None


def bearing_from_bbox(bbox: Optional[list[float]]) -> Optional[float]:
    """Horizontal bearing in degrees, +left / -right of the camera axis."""
    if not bbox:
        return None
    cx = (bbox[0] + bbox[2]) / 2.0
    return round((0.5 - cx) * config.CAMERA_HFOV_DEG, 1)


class VlmClient:
    def __init__(self, url: str = config.VLM_URL, timeout: float = config.VLM_TIMEOUT_S) -> None:
        self.url = url
        self.timeout = timeout
        self._session = requests.Session()

    def verify(self, jpeg: bytes, target: str, frame_ts: float) -> VisionResult:
        t0 = time.time()
        b64 = base64.b64encode(jpeg).decode("ascii")
        body = {
            "model": config.VLM_MODEL_NAME,
            "temperature": config.VLM_TEMPERATURE,
            "max_tokens": config.VLM_MAX_TOKENS,
            "response_format": {"type": "json_schema", "json_schema": {"name": "verify", "schema": RESPONSE_SCHEMA}},
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": [
                    {"type": "text", "text": f"Target: {target}. Is it visible?"},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                ]},
            ],
        }
        try:
            r = self._session.post(self.url, json=body, timeout=self.timeout)
            if r.status_code != 200:
                return self._fail(f"http {r.status_code}: {r.text[:120]}", frame_ts)
            text = r.json()["choices"][0]["message"]["content"]
        except requests.Timeout:
            return self._fail("timeout", frame_ts)
        except requests.ConnectionError:
            return self._fail("connection_error (model server down?)", frame_ts)
        except (KeyError, IndexError, TypeError, ValueError) as e:
            return self._fail(f"unexpected response shape: {e}", frame_ts)
        except Exception as e:                      # truly never crash the pipeline
            return self._fail(f"unexpected: {e!r}", frame_ts)

        parsed, err = parse_vlm_output(text)
        if err:
            return self._fail(err, frame_ts)
        match = parsed["match"] and parsed["confidence"] >= config.VISION_MATCH_CONFIDENCE_MIN
        res = VisionResult(match=match, confidence=parsed["confidence"], bbox=parsed["bbox"] if match else None,
                           ts=time.time(), frame_ts=frame_ts, ok=True, error=None,
                           bearing_deg=bearing_from_bbox(parsed["bbox"]) if match else None, source="vlm")
        log_event(log, "vision.result", match=res.match, confidence=round(res.confidence, 2), bbox=res.bbox,
                  latency_ms=int((time.time() - t0) * 1000), raw_match=parsed["match"])
        return res

    @staticmethod
    def _fail(error: str, frame_ts: float) -> VisionResult:
        log_event(log, "vision.error", error=error)
        return VisionResult(match=False, confidence=0.0, bbox=None, ts=time.time(), frame_ts=frame_ts,
                            ok=False, error=error, source="vlm")

    def healthy(self) -> bool:
        base = self.url.split("/v1/")[0]
        try:
            return self._session.get(base + config.MODEL_HEALTH_PATH, timeout=1.5).status_code == 200
        except requests.RequestException:
            return False
