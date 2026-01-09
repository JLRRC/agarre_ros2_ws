"""Servicio ligero para mover el UR5 a HOME desde Panel V2."""

from __future__ import annotations

import os
import shlex
from typing import Callable, Optional

from .panel_utils import CmdRunner  # evitamos dependencias circulares


class RobotService:
    """Encapsula comandos de movimiento del robot usados por el panel."""

    def __init__(self, log_fn: Optional[Callable[[str], None]] = None, ws_dir: str = ""):
        self._log = log_fn or (lambda _: None)
        self._ws_dir = os.path.abspath(ws_dir or os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))

    def move_to_home(self, runner: CmdRunner, timeout: float = 30.0) -> bool:
        """Lanza el script *ur5_go_home.sh* y retorna True si terminó sin errores."""

        script = os.path.join(self._ws_dir, "scripts", "ur5_go_home.sh")
        if not os.path.isfile(script):
            self._log(f"[ROBOT] Error: script no encontrado ({script})")
            return False

        cmd = f"bash {shlex.quote(script)}"
        env = os.environ.copy()
        env["WS_DIR"] = self._ws_dir
        self._log(f"[ROBOT] Ejecutando HOME: {cmd}")
        rc, stdout, stderr = runner.run_cmd(cmd, timeout=timeout, env=env)
        if stdout:
            self._log(f"[ROBOT] STDOUT: {stdout.strip()}")
        if stderr:
            self._log(f"[ROBOT] STDERR: {stderr.strip()}")
        if rc != 0:
            self._log(f"[ROBOT] Falló HOME (rc={rc})")
            return False
        return True
