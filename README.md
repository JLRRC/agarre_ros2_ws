<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/README.md -->
<!-- Summary: ROS 2 Jazzy workspace overview and build/run notes. -->
# agarre_ros2_ws

Workspace ROS 2 (Jazzy) para el trabajo de agarre inteligente.

## Build
```bash
cd ~/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Panel (modo PRO)
```bash
./scripts/run_panel_superpro.sh
```

## Gazebo (headless)
```bash
./scripts/run_ur5_world.sh
```

## Bridge ROS <-> Gazebo (YAML)
```bash
./scripts/run_gz_ros_bridge.sh
```

## Narrativa TFM — Evidencias

- Desde el panel principal (`src/ur5_qt_panel/.../main_panel.py`), la nueva pestaña **TFM • Evidencias** aparece junto al robot; permite listar y regenerar artefactos (PNG/CSV/Markdown) y copiarlos al portapapeles.
- Para regenerar todo de golpe, ejecuta el exportador CLI:
  ```bash
  cd ~/TFM/agarre_ros2_ws
  python3 tools/tfm_export_all.py --all
  python3 tools/tfm_export_all.py --only tables
  python3 tools/tfm_export_all.py --only figures
  ```
  Los resultados se colocan en `reports/tfm_artifacts/{figures|tables}` y se produce `reports/tfm_artifacts/INDEX.md`.

### Pestaña “TFM • Evidencias”

- La nueva pestaña vive en la segunda posición del split izquierdo y solo muestra ilustraciones/tablas, dejando la primera pestaña exclusiva para el panel ROS.
- Usa **Actualizar / Re-escanear** para volver a leer los PNG/MD/CSV de `reports/`, `experiments/figures_memoria`, y otras carpetas que contengan “ilustracion”. Puedes registrar figuras fuera creando `reports/tfm_artifacts/manual_figures.json` con entradas `{ "name", "path", "description" }`.
- Las tablas disponibles se listan bajo “Tablas (datos reales)” y si falta alguna (p. ej. latencia) aparece en “Pendientes” hasta que exista el CSV esperado en `reports/bench/latency_results.csv`.
- El botón **Ejecutar snapshot** lanza `python3 scripts/review_snapshot.py`, que regenera los artefactos internos, actualiza el índice y escribe sus logs en la caja inferior.
- Cada artefacto ofrece controles de copia de Markdown, guardado de la imagen y acceso al directorio de salida.
- El panel ROS ahora intenta arrancarse solo: si no detecta `ros2_control` ni Gazebo después de unos segundos arranca automáticamente `START ALL`, y el bloque Robot/DEMO solo cambia de estado cuando el sistema se considera estable (evita cambios continuos al comprobar controladores).
