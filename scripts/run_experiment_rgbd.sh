#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/run_experiment_rgbd.sh
# Summary: Launches RGB-D training experiment in agarre_inteligente.
set -e

cd "$HOME/TFM/agarre_inteligente"

if [ -f ".venv/bin/activate" ]; then
  source .venv/bin/activate
fi

export PYTHONPATH="$PWD/src:${PYTHONPATH}"

CONFIG="configs/exp_resnet18_rgbd.yaml"

echo "[ExpRGBD] Lanzando experimento RGB-D con ${CONFIG}..."
python src/graspnet/train/train_cornell.py --config "$CONFIG"
