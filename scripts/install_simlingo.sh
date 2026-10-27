#!/usr/bin/env bash
# Set up SimLingo (CVPR 2025, https://github.com/RenzKa/simlingo) and its copy of Bench2Drive, to
# run the published checkpoint closed loop on the Bench2Drive routes (scripts/run_bench2drive.sh).
#
#   external/simlingo/           the SimLingo repository at SIMLINGO_COMMIT (agent, Bench2Drive leaderboard and scenario
#                                runner), with scripts/simlingo.patch: environment switches for run_bench2drive.sh
#   external/simlingo_venv/      Python 3.8 with the inference subset of SimLingo's environment.yaml (no conda, no flash-attn)
#   external/simlingo_ckpt/      the checkpoint from huggingface.co/RenzKa/simlingo (2.6 GB, weights and .hydra config only)
#   external/carla/              CARLA's AdditionalMaps (7.4 GB): Bench2Drive drives Town06, 07, 11, 12, 13 and 15 too
#
# InternVL2-1B, the model SimLingo is built on, is fetched from Hugging Face on the first run.
# Run scripts/install_carla.sh first. Each step is skipped when already done.
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh

SIMLINGO_COMMIT=743b243afd6cf5ff51b9fa1f8cac86f22d569684  # main as of 2025-08-25
MAPS_URL=${MAPS_URL:-https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/AdditionalMaps_0.9.15.tar.gz}
CKPT_URL=https://huggingface.co/RenzKa/simlingo/resolve/main/simlingo
SIMLINGO=$EXTERNAL/simlingo
VENV=$EXTERNAL/simlingo_venv
CKPT=$EXTERNAL/simlingo_ckpt/simlingo

[[ -f $CARLA_ROOT/.carla-0.9.15 ]] || { echo "CARLA is not installed: run scripts/install_carla.sh" >&2; exit 1; }
command -v uv >/dev/null || { echo "uv is needed (curl -LsSf https://astral.sh/uv/install.sh | sh)" >&2; exit 1; }
mkdir -p "$EXTERNAL/downloads"

if [[ ! -d $SIMLINGO/.git ]]; then
    echo "   SimLingo repository"
    git clone https://github.com/RenzKa/simlingo.git "$SIMLINGO"
fi
git -C "$SIMLINGO" fetch -q --depth 1 origin "$SIMLINGO_COMMIT" 2>/dev/null || true
git -C "$SIMLINGO" checkout -q "$SIMLINGO_COMMIT"
# switches for run_bench2drive.sh (--debug, --no-cot, --fast), all off by default so the agent runs as upstream's
if ! git -C "$SIMLINGO" apply --reverse --check "$REPO/scripts/simlingo.patch" 2>/dev/null; then
    git -C "$SIMLINGO" apply "$REPO/scripts/simlingo.patch"
fi

if [[ ! -f $CARLA_ROOT/.carla-additional-maps ]]; then
    archive=$EXTERNAL/downloads/AdditionalMaps_0.9.15.tar.gz
    echo "   CARLA AdditionalMaps (7.4 GB)"
    curl -fL --retry 5 --retry-delay 10 -C - -o "$archive" "$MAPS_URL"
    tar -xzf "$archive" -C "$CARLA_ROOT"
    touch "$CARLA_ROOT/.carla-additional-maps"
    rm -f "$archive"
fi

ckpt_file="$CKPT/checkpoints/epoch=013.ckpt/pytorch_model.pt"
if [[ ! -f $ckpt_file ]]; then
    echo "   SimLingo checkpoint (2.6 GB)"
    mkdir -p "$CKPT/.hydra" "$(dirname "$ckpt_file")"
    curl -fsSL -o "$CKPT/.hydra/config.yaml" "$CKPT_URL/.hydra/config.yaml"
    curl -fL --retry 5 -C - -o "$ckpt_file.part" "$CKPT_URL/checkpoints/epoch=013.ckpt/pytorch_model.pt"
    mv "$ckpt_file.part" "$ckpt_file"
fi

if [[ ! -x $VENV/bin/python ]]; then
    echo "   Python 3.8 environment"
    uv venv -q -p 3.8 "$VENV"
fi
# versions as in SimLingo's environment.yaml; OpenCV headless and newer, as 4.2 has no wheel for this numpy
VIRTUAL_ENV=$VENV uv pip install -q --index-strategy unsafe-best-match \
    --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch==2.2.0 torchvision==0.17.0 transformers==4.46.3 tokenizers==0.20.3 peft==0.13.2 accelerate==1.0.1 \
    safetensors==0.5.2 huggingface-hub==0.27.0 hydra-core==1.3.2 omegaconf==2.3.0 lightning==2.3.3 \
    pytorch-lightning==2.4.0 torchmetrics==1.3.1 timm==0.9.16 einops==0.7.0 sentencepiece \
    numpy==1.23.0 scipy==1.10.1 opencv-python-headless==4.8.1.78 pillow==10.2.0 matplotlib==3.5.3 \
    filterpy==1.4.5 shapely==1.7.1 networkx==3.1 transforms3d==0.4.1 rdp==0.8 ujson==5.9.0 simplejson \
    dictor==0.1.12 ephem==4.1.5 py-trees==0.8.3 six tabulate==0.9.0 xmlschema==1.0.18 \
    simple-watchdog-timer==0.1.1 retry==0.9.2 requests pexpect pygame==2.5.2 gitpython==3.1.42 \
    openai==1.76.0 psutil carla==0.9.15 \
    casadi==3.7.2 pybind11==3.0.4 imageio-ffmpeg pytest  # this repository: the NMPC core and its build, videos, tests
# FlashAttention 2, as SimLingo pins it: without it InternVL2 falls back to eager attention, about 1.5x slower
FA_WHEEL=flash_attn-2.7.0.post2+cu12torch2.2cxx11abiFALSE-cp38-cp38-linux_x86_64.whl
if ! "$VENV/bin/python" -c "import flash_attn" 2>/dev/null; then
    echo "   FlashAttention 2.7.0.post2"
    rm -rf "$EXTERNAL/downloads/flash_attn" && mkdir -p "$EXTERNAL/downloads/flash_attn"
    curl -fsSL -o "$EXTERNAL/downloads/flash_attn/$FA_WHEEL" \
        "https://github.com/Dao-AILab/flash-attention/releases/download/v2.7.0.post2/$FA_WHEEL"
    VIRTUAL_ENV=$VENV uv pip install -q --no-deps "$EXTERNAL/downloads/flash_attn/$FA_WHEEL"
    rm -rf "$EXTERNAL/downloads/flash_attn"
fi
"$VENV/bin/python" -c "import torch; assert torch.cuda.is_available(), 'PyTorch sees no GPU'"
echo "   SimLingo ready: build the NMPC core (cmake -S . -B build && cmake --build build -j), then scripts/run_bench2drive.sh"
