#!/usr/bin/env bash
set -e

cd "$HOME/TFM/agarre_inteligente"

if [ -f ".venv/bin/activate" ]; then
  source .venv/bin/activate
fi

export PYTHONPATH="$PWD/src:${PYTHONPATH}"

CONFIG="configs/exp_resnet18_rgbd.yaml"

echo "[ExpRGBD] Lanzando experimento RGB-D con ${CONFIG}..."
python src/graspnet/train/train_cornell.py --config "$CONFIG"
