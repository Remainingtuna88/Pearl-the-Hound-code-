#!/usr/bin/env bash
# Starts the two llama.cpp servers the backend talks to. Edit the GGUF paths.
# Ports must match config.VLM_URL / config.PLANNER_URL (8080 / 8081).
#
# Build llama.cpp on the Pi 5 (ARM64, CPU only):
#   git clone https://github.com/ggerganov/llama.cpp && cd llama.cpp
#   cmake -B build -DGGML_NATIVE=ON && cmake --build build --config Release -j4 --target llama-server
#
# TODO: set these paths
LLAMA_SERVER=${LLAMA_SERVER:-$HOME/llama.cpp/build/bin/llama-server}
VLM_GGUF=${VLM_GGUF:-$HOME/models/LFM2.5-VL-450M-Q8_0.gguf}
VLM_MMPROJ=${VLM_MMPROJ:-$HOME/models/mmproj-LFM2.5-VL-450M.gguf}     # vision projector file that ships with the VL GGUF
PLANNER_GGUF=${PLANNER_GGUF:-$HOME/models/LFM2.5-230M-Q8_0.gguf}

# Thread budget: the Pi has 4 cores shared with SLAM/Nav2/camera. 2 + 1 is a
# starting point; measure with `top` and adjust. TODO: tune.
"$LLAMA_SERVER" -m "$VLM_GGUF" --mmproj "$VLM_MMPROJ" \
    --host 127.0.0.1 --port 8080 -t 2 -c 2048 --parallel 1 \
    > logs/llama_vlm.log 2>&1 &
echo "VLM server pid $!  (port 8080)"

"$LLAMA_SERVER" -m "$PLANNER_GGUF" \
    --host 127.0.0.1 --port 8081 -t 1 -c 2048 --parallel 1 --jinja \
    > logs/llama_planner.log 2>&1 &
echo "planner server pid $!  (port 8081)"

# --jinja enables the model's chat template incl. tool-call formatting, only
# needed for PLANNER_CONSTRAIN_MODE="native". Harmless otherwise.
# Check: curl -s localhost:8080/health ; curl -s localhost:8081/health
wait
