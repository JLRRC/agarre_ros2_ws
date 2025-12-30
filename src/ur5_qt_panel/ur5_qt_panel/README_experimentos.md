# Ruta/URL: file:///home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/README_experimentos.md
# Nombre: README_experimentos.md
# Qué hace: Guia rapida del Asistente de Experimentos, estados, logs y rutas de export TFM.

# Panel Experimentos (PRO)

## Como usar el asistente
1) Abre la pestaña "Experimentos".
2) En el bloque "Asistente Experimentos", pulsa "Recomprobar todo".
3) Revisa los pasos A–G y corrige los que estén en 🟠/🔴.
4) Selecciona un experimento en el combo superior y revisa la descripcion.
5) Ejecuta:
   - "Smoke test (1 epoch)" para comprobar la cadena completa.
   - "Ejecutar (seed unica)" para un entrenamiento real.
   - "Ejecutar (multi-seed: 0,1,2)" para comparativas.
6) Usa "Generar resumen A/B" y "Exportar TFM" al finalizar.

## Que significan los estados
- 🟢 OK: paso listo.
- 🟠 Advertencia: algo falta pero se puede corregir.
- 🔴 Bloqueado: no se permite entrenar (Cornell PRO incompleto).

## Logs y artefactos
- Logs del panel:
  - `agarre_ros2_ws/log/experiments/experiments_debug.log`
  - `agarre_ros2_ws/log/experiments/experiments_debug_latest.log`
- Logs por ejecucion:
  - `agarre_ros2_ws/log/experiments/experiments_debug_<tag>_<timestamp>.log`
- Limpieza segura:
  - `agarre_ros2_ws/log/panel_experimentos_cleanup.log`

## Salidas de export TFM
- Tablas:
  - `agarre_inteligente/reports/tfm_tablas/tabla_ab_resumen.csv`
- Figuras:
  - `agarre_inteligente/reports/tfm_figuras/comparativa_val_success.png`
  - `agarre_inteligente/reports/tfm_figuras/comparativa_loss.png`
- Memoria:
  - `agarre_inteligente/experiments/figures_memoria/memoria_resumen.md`

