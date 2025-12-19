#!/usr/bin/env bash
set -e

# Ir al repo del TFM
cd "$HOME/TFM/agarre_inteligente"

# Activar venv si existe
if [ -f ".venv/bin/activate" ]; then
  source .venv/bin/activate
fi

# Añadir src/ al PYTHONPATH para que Python vea el paquete graspnet
export PYTHONPATH="$PWD/src:${PYTHONPATH}"

CONFIG="configs/exp_resnet18_rgb.yaml"

echo "[ExpRGB] Lanzando experimento RGB con ${CONFIG}..."
python src/graspnet/train/train_cornell.py --config "$CONFIG"
