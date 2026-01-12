#!/bin/bash
set -e

echo "🔒 Forzando rama correcta: env/pc_nvidia"

# Asegurar rama correcta
git checkout env/pc_nvidia

# Limpiar restos raros
git rebase --abort 2>/dev/null || true
git merge --abort 2>/dev/null || true

# Eliminar conflictos y basura
git reset --hard
git clean -fd

echo "📦 Copiando estado ACTUAL del working tree como verdad absoluta"

# Añadir todo lo que HAY en la carpeta
git add -A

# Commit forzado
git commit -m "PC_NVIDIA: estado funcional estable (TF OK, panel, pose_info debug)" || true

echo "🚀 Subiendo a GitHub (FORCE, rama env/pc_nvidia)"

git push origin env/pc_nvidia --force

echo "✅ LISTO: GitHub = carpeta local (env/pc_nvidia)"
