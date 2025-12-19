#!/usr/bin/env bash
set -e

cd "$HOME/TFM/agarre_inteligente"

if [ -f ".venv/bin/activate" ]; then
  source .venv/bin/activate
fi

export PYTHONPATH="$PWD/src:${PYTHONPATH}"

echo "[ExpEval] Evaluando mejores épocas..."
python src/graspnet/metrics/select_best_epoch.py
