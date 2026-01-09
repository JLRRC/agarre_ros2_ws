"""Servicio para manejar el estado de calibración de Panel V2."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import numpy as np


class CalibrationMode(Enum):
    """Modo soportado de calibración de la mesa."""

    LINEAR_2PT = "linear_2pt"
    HOMOGRAPHY = "homography"


@dataclass
class CalibrationData:
    """Estado actual de calibración."""

    mode: CalibrationMode
    matrix: Optional[np.ndarray] = None
    camera_topic: Optional[str] = None


class CalibrationService:
    """Servicio ligero que guarda matrices y estado de calibración."""

    def __init__(self, log_fn: Optional[Callable[[str], None]] = None):
        self._log = log_fn or (lambda _: None)
        self._state: Optional[CalibrationData] = None

    def start_calibration(self, camera_topic: str, mode: CalibrationMode) -> None:
        self._log(f"[CALIB] start {mode.name} ({camera_topic})")
        self._state = CalibrationData(mode=mode, matrix=None, camera_topic=camera_topic)

    def get_calibration(self) -> Optional[CalibrationData]:
        return self._state

    def set_homography(self, homography) -> None:
        arr = np.array(homography, dtype=float)
        if arr.shape != (3, 3):
            raise ValueError("Homografía inválida: debe ser 3x3.")
        camera_topic = self._state.camera_topic if self._state else None
        self._state = CalibrationData(mode=CalibrationMode.HOMOGRAPHY, matrix=arr, camera_topic=camera_topic)
        self._log("[CALIB] homografía almacenada")
