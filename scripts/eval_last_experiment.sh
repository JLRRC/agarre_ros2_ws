#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/eval_last_experiment.sh
# Summary: Runs evaluation of best epochs in agarre_inteligente.
set -e

cd "$HOME/TFM/agarre_inteligente"

if [ -f ".venv/bin/activate" ]; then
  source .venv/bin/activate
fi

export PYTHONPATH="$PWD/src:${PYTHONPATH}"

echo "[ExpEval] Evaluando mejores épocas..."
python src/graspnet/metrics/select_best_epoch.py
