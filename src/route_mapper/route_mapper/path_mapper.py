#!/usr/bin/env python3
"""
path_mapper.py
---------------
Módulo independiente para crear una memoria simple del recorrido:
- Guarda el camino del robot usando /odom.
- Convierte puntos LiDAR del marco del robot al marco odom.
- Separa visualmente trayectoria, paredes y puntos/obstáculos detectados.
- Calcula una corrección suave para evitar volver sobre sus pasos en la primera vuelta.

Convención de los puntos que llegan desde radar_utils:
- x_local positivo = derecha del robot
- y_local positivo = frente del robot
"""
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np


Point = Tuple[float, float]


@dataclass
class RouteMemoryMapper:
    # Muestreo del recorrido
    path_min_step: float = 0.035
    max_path_points: int = 2500

    # Puntos LiDAR transformados a odom
    max_wall_points: int = 5000
    max_obstacle_points: int = 2500
    max_map_range: float = 2.60
    lidar_stride: int = 4

    # Anti-retorno
    min_route_before_avoid_m: float = 1.20
    revisit_radius: float = 0.18
    recent_points_to_ignore: int = 45
    lap_radius: float = 0.28
    min_lap_distance_m: float = 2.20
    min_lap_time_s: float = 18.0
    anti_return_bias: float = 0.18

    path: Deque[Point] = field(default_factory=lambda: deque(maxlen=2500))
    wall_points: Deque[Point] = field(default_factory=lambda: deque(maxlen=5000))
    obstacle_points: Deque[Point] = field(default_factory=lambda: deque(maxlen=2500))

    start_pose: Optional[Tuple[float, float, float]] = None
    last_pose: Optional[Tuple[float, float, float]] = None
    last_path_point: Optional[Point] = None
    start_time: float = field(default_factory=time.time)
    total_distance: float = 0.0
    lap_count: int = 0
    lap_locked: bool = False
    last_revisit_distance: float = float("inf")

    def __post_init__(self):
        # Ajustar maxlen según los valores configurados.
        self.path = deque(maxlen=self.max_path_points)
        self.wall_points = deque(maxlen=self.max_wall_points)
        self.obstacle_points = deque(maxlen=self.max_obstacle_points)

    # ------------------------------------------------------
    # ODOMETRÍA Y RUTA
    # ------------------------------------------------------
    def actualizar_pose(self, x: float, y: float, yaw: float):
        if self.start_pose is None:
            self.start_pose = (float(x), float(y), float(yaw))
            self.start_time = time.time()

        if self.last_pose is not None:
            dx = float(x) - self.last_pose[0]
            dy = float(y) - self.last_pose[1]
            d = math.hypot(dx, dy)
            if d < 0.50:  # evita saltos raros de odometría
                self.total_distance += d

        self.last_pose = (float(x), float(y), float(yaw))

        if self.last_path_point is None:
            self.path.append((float(x), float(y)))
            self.last_path_point = (float(x), float(y))
        else:
            dx = float(x) - self.last_path_point[0]
            dy = float(y) - self.last_path_point[1]
            if math.hypot(dx, dy) >= self.path_min_step:
                self.path.append((float(x), float(y)))
                self.last_path_point = (float(x), float(y))

        self._actualizar_vueltas(float(x), float(y))

    def _actualizar_vueltas(self, x: float, y: float):
        if self.start_pose is None:
            return

        sx, sy, _ = self.start_pose
        dist_inicio = math.hypot(x - sx, y - sy)
        tiempo = time.time() - self.start_time

        if (self.total_distance > self.min_lap_distance_m and
                tiempo > self.min_lap_time_s and
                dist_inicio < self.lap_radius and
                not self.lap_locked):
            self.lap_count += 1
            self.lap_locked = True

        # desbloquea cuando se aleja del inicio, para poder contar otra vuelta.
        if dist_inicio > self.lap_radius * 1.8:
            self.lap_locked = False

    # ------------------------------------------------------
    # LIDAR A MAPA GLOBAL
    # ------------------------------------------------------
    def actualizar_lidar(self, datos_lidar: Dict, odom_x: float, odom_y: float, yaw: float,
                         pared_der_valida: bool = False, pared_izq_valida: bool = False):
        if datos_lidar is None:
            return

        x_local = datos_lidar.get('x')
        y_local = datos_lidar.get('y')
        if x_local is None or y_local is None or len(x_local) == 0:
            return

        x_local = np.asarray(x_local, dtype=float)
        y_local = np.asarray(y_local, dtype=float)
        ranges = np.hypot(x_local, y_local)
        valid = np.isfinite(x_local) & np.isfinite(y_local) & (ranges > 0.06) & (ranges < self.max_map_range)

        mask_der = np.asarray(datos_lidar.get('mask_der', np.zeros_like(valid)), dtype=bool)
        mask_izq = np.asarray(datos_lidar.get('mask_izq', np.zeros_like(valid)), dtype=bool)
        mask_frente = np.asarray(datos_lidar.get('mask_frente', np.zeros_like(valid)), dtype=bool)

        # Paredes: puntos laterales. Solo se guardan si al menos una pared lateral está validada.
        wall_mask = valid & (mask_der | mask_izq) & (pared_der_valida | pared_izq_valida)
        obstacle_mask = valid & mask_frente

        idx_wall = np.where(wall_mask)[0][::max(1, self.lidar_stride)]
        idx_obs = np.where(obstacle_mask)[0][::max(1, self.lidar_stride)]

        for idx in idx_wall:
            self.wall_points.append(self._local_a_odom(x_local[idx], y_local[idx], odom_x, odom_y, yaw))

        for idx in idx_obs:
            self.obstacle_points.append(self._local_a_odom(x_local[idx], y_local[idx], odom_x, odom_y, yaw))

    def _local_a_odom(self, x_der: float, y_frente: float, ox: float, oy: float, yaw: float) -> Point:
        # forward = (cos yaw, sin yaw), right = (sin yaw, -cos yaw)
        wx = ox + y_frente * math.cos(yaw) + x_der * math.sin(yaw)
        wy = oy + y_frente * math.sin(yaw) - x_der * math.cos(yaw)
        return (float(wx), float(wy))

    # ------------------------------------------------------
    # ANTI-RETORNO SUAVE
    # ------------------------------------------------------
    def obtener_correccion_antiretorno(self, x: float, y: float, yaw: float) -> Dict:
        """Devuelve una corrección angular pequeña si el robot pisa ruta antigua.

        La lógica se desactiva cuando lap_count >= 1 porque eso significa que ya está
        pasando por segunda o tercera vuelta y sí puede repetir el recorrido.
        """
        if self.lap_count >= 1:
            self.last_revisit_distance = float("inf")
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}

        if self.total_distance < self.min_route_before_avoid_m:
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}

        if len(self.path) <= self.recent_points_to_ignore + 5:
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}

        antiguos = list(self.path)[:-self.recent_points_to_ignore]
        if not antiguos:
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}

        arr = np.asarray(antiguos, dtype=float)
        dx = arr[:, 0] - x
        dy = arr[:, 1] - y
        d2 = dx * dx + dy * dy
        i = int(np.argmin(d2))
        d = math.sqrt(float(d2[i]))
        self.last_revisit_distance = d

        if d > self.revisit_radius:
            return {"activo": False, "bias_angular": 0.0, "distancia": d}

        # Ubicación del camino viejo respecto al robot: si está a la izquierda,
        # gira suave a la derecha; si está a la derecha, gira suave a la izquierda.
        vx = float(dx[i])
        vy = float(dy[i])
        lateral_izq = -math.sin(yaw) * vx + math.cos(yaw) * vy

        if abs(lateral_izq) < 0.04:
            bias = -self.anti_return_bias  # caso ambiguo: preferir derecha
        else:
            bias = -self.anti_return_bias if lateral_izq > 0 else self.anti_return_bias

        return {"activo": True, "bias_angular": bias, "distancia": d}

    # ------------------------------------------------------
    # DATOS PARA GUI
    # ------------------------------------------------------
    def obtener_rectangulo_estimado(self) -> List[Point]:
        if len(self.path) < 12:
            return []
        arr = np.asarray(self.path, dtype=float)
        min_x, min_y = np.min(arr[:, 0]), np.min(arr[:, 1])
        max_x, max_y = np.max(arr[:, 0]), np.max(arr[:, 1])
        if (max_x - min_x) < 0.15 or (max_y - min_y) < 0.15:
            return []
        return [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y), (min_x, min_y)]

    def obtener_datos_mapa(self) -> Dict:
        return {
            "path": list(self.path),
            "walls": list(self.wall_points),
            "obstacles": list(self.obstacle_points),
            "rectangle": self.obtener_rectangulo_estimado(),
            "laps": self.lap_count,
            "total_distance": self.total_distance,
            "last_revisit_distance": self.last_revisit_distance,
        }

    def obtener_estado_resumen(self) -> Dict:
        return {
            "laps": self.lap_count,
            "distance": self.total_distance,
            "path_points": len(self.path),
            "wall_points": len(self.wall_points),
            "obstacle_points": len(self.obstacle_points),
            "revisit_distance": self.last_revisit_distance,
        }
