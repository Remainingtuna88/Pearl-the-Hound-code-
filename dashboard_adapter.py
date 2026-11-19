"""
Structured (JSON-lines) logging.

Every planner decision, goal transition and safety intervention goes through
`log_event(logger, event, **fields)` so the traces are machine-readable for
evaluation later. Human-readable text also goes to stderr.

    from pearl.logging_setup import get_logger, log_event
    log = get_logger(__name__)
    log_event(log, "goal.transition", goal_id=g.id, frm="EXECUTING", to="ABORTED", reason="ttl_expired")

Output:
  stderr           -> plain text (level, logger, message)
  logs/pearl.jsonl -> one JSON object per line, rotating
"""
from __future__ import annotations

import json
import logging
import logging.handlers
import os
import time
from typing import Any

from . import config

_CONFIGURED = False


class JsonLineFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "ts": round(record.created, 3),
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        event = getattr(record, "event", None)
        if event:
            payload["event"] = event
        fields = getattr(record, "fields", None)
        if fields:
            payload.update(fields)
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, default=str, separators=(",", ":"))


def setup_logging() -> None:
    global _CONFIGURED
    if _CONFIGURED:
        return
    _CONFIGURED = True
    root = logging.getLogger("pearl")
    root.setLevel(getattr(logging, config.LOG_LEVEL.upper(), logging.INFO))
    root.propagate = False

    text = logging.StreamHandler()
    text.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s", "%H:%M:%S"))
    root.addHandler(text)

    try:
        os.makedirs(config.LOG_DIR, exist_ok=True)
        jl = logging.handlers.RotatingFileHandler(
            config.LOG_JSONL_FILE, maxBytes=config.LOG_JSONL_MAX_BYTES, backupCount=config.LOG_JSONL_BACKUPS
        )
        jl.setFormatter(JsonLineFormatter())
        root.addHandler(jl)
    except OSError as e:  # read-only FS etc. — never fatal
        root.warning("could not open jsonl log file: %s", e)

    # Quiet the werkzeug request log; it is noisy and not useful for evaluation.
    logging.getLogger("werkzeug").setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    setup_logging()
    if not name.startswith("pearl"):
        name = "pearl." + name
    return logging.getLogger(name)


def log_event(logger: logging.Logger, event: str, level: int = logging.INFO, **fields: Any) -> None:
    """Emit one structured event. `fields` land as top-level JSON keys."""
    summary = " ".join(f"{k}={v}" for k, v in fields.items() if not isinstance(v, (dict, list)))
    logger.log(level, f"{event} {summary}", extra={"event": event, "fields": fields})


def now() -> float:
    return time.time()
