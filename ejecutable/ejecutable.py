#!/usr/bin/env python3
import sys
import os
import time
import math
import threading
from datetime import datetime
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Inyectar dinámicamente la ruta src de Reto_03
ruta_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../src'))
if ruta_src not in sys.path:
    sys.path.append(ruta_src)

# Ruta extra para el paquete route_mapper agregado en src/route_mapper/route_mapper
ruta_route_mapper = os.path.abspath(os.path.join(ruta_src, 'route_mapper'))
if ruta_route_mapper not in sys.path:
    sys.path.append(ruta_route_mapper)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from utils_scripts.control_config import ControlConfig
from utils_scripts.radar_utils import procesar_escaneo_lidar
from utils_scripts.radar_interface import RadarInterface
from utils_scripts.box_avoidance import BoxAvoidanceFSM
from route_mapper.path_mapper import RouteMemoryMapper


class SistemaControlBorde(Node):
    """
    Nodo integrado y modular:
    1) Al ejecutar, abre la interfaz y deja motores en 0.
    2) Al presionar INICIAR, conduce por el centro entre pared derecha e izquierda.
    3) La pared derecha/izquierda se valida por clusters conectados al lateral ±90°.
       Así la pared del frente NO se toma como derecha.
    4) Si solo ve una pared lateral, usa seguimiento lateral suave como respaldo.
    5) Integra memoria de recorrido: ruta, paredes/puntos detectados y control anti-retorno.
    6) La velocidad lineal y angular se cambian desde la GUI; la batería se lee correctamente.
    """

    def __init__(self):
        super().__init__('sistema_control_centrado_modular')

        self.cfg = ControlConfig()

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_lidar)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_sub = self.create_subscription(BatteryState, '/battery', self.battery_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.ui = RadarInterface(
            callback_iniciar=self.handler_iniciar,
            callback_detener=self.handler_detener,
            callback_salir=self.handler_salir,
            callback_vel_lenta=self.handler_vel_lenta,
            callback_vel_media=self.handler_vel_media,
            callback_vel_rapida=self.handler_vel_rapida,
            callback_ang_menos=self.handler_ang_menos,
            callback_ang_mas=self.handler_ang_mas,
        )
        self.fig = self.ui.fig

        # Telemetría LiDAR
        self.datos_filtrados = None
        self.dist_frente = float('inf')
        self.dist_izq = float('inf')
        self.dist_der = float('inf')
        self.dist_diag_der = float('inf')
        self.dist_diag_izq = float('inf')

        # Distancias laterales filtradas por clusters conectados.
        self.dist_der_pared = float('inf')
        self.dist_izq_pared = float('inf')
        self.pendiente_pared_der = 0.0
        self.pendiente_pared_izq = 0.0
        self.pared_der_valida = False
        self.pared_izq_valida = False
        self.puntos_pared_der = 0
        self.puntos_pared_izq = 0
        self.ancho_pasillo = float('inf')
        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0
        self.total_puntos = 0

        # Odometría
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.tengo_odom = False

        # Control y estados
        self.estado_actual = "ESPERANDO INICIO"
        self.robot_habilitado = False
        self.robot_pausado = False
        self.solicitud_salir = False
        self.ciclos_estable = 0
        self.error_anterior = 0.0
        self.t_anterior = time.time()
        self.t_estado = time.time()
        self.estado_post_pausa = None
        self.sentido_giro_frente = self.cfg.sentido_giro_esquina
        self.t_inicio_giro_frente = time.time()
        self.t_inicio_alinear_post_giro = time.time()
        self.t_ultima_esquina = 0.0
        self.yaw_inicio_giro = None
        self.ultimo_lado_pared = "izquierda"  # antihorario: se prioriza pared izquierda / isla central

        # Bloqueo temporal de referencia lateral.
        # Evita que, antes/durante una esquina, el robot pierda la pared derecha
        # y tome la pared izquierda como nueva referencia, provocando una vuelta.
        self.referencia_estable = self.cfg.referencia_preferida_esquina
        self.referencia_candidata = None
        self.frames_referencia_candidata = 0
        self.referencia_bloqueada = None
        self.t_bloqueo_referencia = 0.0
        self.t_ultima_pared_der = 0.0
        self.t_ultima_pared_izq = 0.0

        # Control por esquinas: evita que una esquina corta dispare otro giro completo.
        self.contador_esquinas = 0
        self.indice_esquina_actual = 0
        self.esquina_en_proceso = False
        self.t_inicio_avance_post_esquina = time.time()
        self.x_inicio_avance_post_esquina = 0.0
        self.y_inicio_avance_post_esquina = 0.0
        self.x_fin_ultima_esquina = 0.0
        self.y_fin_ultima_esquina = 0.0
        self.lateral_estable_post_giro = 0

        # Salida protegida de esquina:
        # después de girar, el robot avanza recto unos cm sin volver a
        # analizar otra esquina. Esto evita que la 4ta esquina acumule giros
        # y termine regresando por donde vino.
        self.yaw_salida_post_esquina = None
        self.sentido_salida_post_esquina = 1.0
        self.indice_salida_post_esquina = 0

        # Análisis frontal: pasadizo vs esquina.
        self.tipo_frente = "SIN_DATOS"
        self.frente_es_pasillo = False
        self.frente_es_esquina = False
        self.frente_conecta_der = False
        self.frente_conecta_izq = False
        self.frente_conecta_ambos = False
        self.frente_confianza = 0.0
        self.front_class_sm = "NONE"
        self.front_ang_width = 0.0
        self.front_dist_sm = float('inf')
        self.alpha_pared_der = 0.0
        self.alpha_pared_izq = 0.0
        self.len_pared_der = 0.0
        self.len_pared_izq = 0.0

        # Persistencia tipo CapyGuardian: una lectura mala no cambia de estado.
        self._persist = {}

        # Conteo simple de cajas detectadas por arco frontal corto.
        self.contador_cajas = 0
        self.t_ultima_caja = 0.0

        # FSM modular de rodeo de caja: giro IZQ -> DER -> DER -> IZQ.
        self.rodeo_caja = BoxAvoidanceFSM(self.cfg, logger=self.get_logger())

        # Selector de 3 velocidades lineales desde la GUI.
        self.modo_velocidad = self.cfg.modo_velocidad_inicial
        self.factor_velocidad = self.obtener_factor_velocidad(self.modo_velocidad)
        self.nombre_modo_velocidad = self.obtener_nombre_velocidad(self.modo_velocidad)

        # Control angular independiente desde GUI.
        self.factor_angular = self.cfg.factor_angular_inicial

        # Velocidades publicadas.
        self.vel_lineal = 0.0
        self.vel_angular = 0.0

        # Batería real: se lee desde /battery. Si no llega, se muestra SIN DATOS.
        self.voltaje_bateria = float('nan')
        self.porcentaje_bateria = None
        self.bateria_fuente = "SIN DATOS"
        self.bateria_real_recibida = False

        # Memoria/mapa del recorrido y puntos detectados por LiDAR.
        self.mapa_ruta = RouteMemoryMapper()
        self.evitar_retorno_activo = False

        # Historial para reporte al pausar.
        # Se usa para plotear recorrido + factores relevantes sin frenar el control.
        self.historial_control = deque(maxlen=3500)
        self.ultimo_reporte_pausa = None

        # Trayectoria independiente SOLO PARA EL PLOT.
        # No modifica la lógica de navegación ni los comandos del robot.
        # Si /odom no se mueve o no llega, se estima la ruta integrando cmd_vel
        # para que al pausar aparezca el recorrido como en el dashboard de referencia.
        self.plot_x = 0.0
        self.plot_y = 0.0
        self.plot_yaw = 0.0
        self.plot_total_distance = 0.0
        self.plot_last_t = None
        self.plot_path = deque(maxlen=5000)
        self.plot_path.append((0.0, 0.0))
        self.plot_source = "cmd_vel estimado"

        # Estadísticas de procesamiento.
        self.tiempos_proc = deque(maxlen=80)
        self.tiempos_loop = deque(maxlen=80)
        self.ultimo_scan_time = None
        self.t_proc_actual = 0.0

        # Timer de seguridad: aunque no llegue scan, publica 0 al inicio/pausa.
        self.timer_seguridad = self.create_timer(0.20, self.timer_seguridad_callback)

        self.get_logger().info("Interfaz lista. El robot NO se moverá hasta presionar INICIAR.")

    # ==========================================================
    # BOTONES DE INTERFAZ
    # ==========================================================
    def handler_iniciar(self, event):
        self.robot_habilitado = True
        self.robot_pausado = False
        self.plot_last_t = time.time()
        self.cambiar_estado("CENTRAR_PASILLO")
        self.get_logger().info("INICIAR presionado: conduciendo por el centro del pasillo.")

    def handler_detener(self, event):
        if not self.robot_habilitado:
            self.detener_robot()
            self.estado_actual = "ESPERANDO INICIO"
            return

        self.robot_pausado = not self.robot_pausado
        if self.robot_pausado:
            self.estado_post_pausa = self.estado_actual
            self.detener_robot()
            self.estado_actual = "PAUSA MANUAL"
            self.get_logger().warn("PAUSA: motores detenidos. Generando reporte del recorrido...")
            self.generar_reporte_pausa()
        else:
            self.plot_last_t = time.time()
            self.cambiar_estado(self.estado_post_pausa or "CENTRAR_PASILLO")
            self.get_logger().info("REANUDAR: control activo.")

    def handler_salir(self, event):
        self.get_logger().info("Cerrando aplicación por interfaz gráfica...")
        self.solicitud_salir = True
        self.detener_robot()
        plt.close(self.fig)

    def handler_vel_lenta(self, event):
        self.set_velocidad(1)

    def handler_vel_media(self, event):
        self.set_velocidad(2)

    def handler_vel_rapida(self, event):
        self.set_velocidad(3)

    def handler_ang_menos(self, event):
        self.set_factor_angular(self.factor_angular - self.cfg.factor_angular_paso)

    def handler_ang_mas(self, event):
        self.set_factor_angular(self.factor_angular + self.cfg.factor_angular_paso)

    def set_velocidad(self, modo):
        self.modo_velocidad = modo
        self.factor_velocidad = self.obtener_factor_velocidad(modo)
        self.nombre_modo_velocidad = self.obtener_nombre_velocidad(modo)
        self.get_logger().info(
            f"Velocidad lineal seleccionada: {self.nombre_modo_velocidad} "
            f"(factor {self.factor_velocidad:.2f})"
        )

    def set_factor_angular(self, factor):
        self.factor_angular = max(self.cfg.factor_angular_min, min(self.cfg.factor_angular_max, float(factor)))
        self.get_logger().info(f"Factor de velocidad angular: {self.factor_angular:.2f}x")

    def porcentaje_a_voltaje(self, porcentaje):
        porcentaje = max(0, min(100, porcentaje))
        return 10.5 + (porcentaje / 100.0) * 2.1

    def obtener_factor_velocidad(self, modo):
        if modo == 1:
            return self.cfg.factor_lento
        if modo == 3:
            return self.cfg.factor_rapido
        return self.cfg.factor_medio

    def obtener_nombre_velocidad(self, modo):
        if modo == 1:
            return "LENTO"
        if modo == 3:
            return "RÁPIDO"
        return "MEDIO"

    # ==========================================================
    # SENSORES
    # ==========================================================
    def battery_callback(self, msg):
        """Actualiza indicador de batería desde sensor_msgs/BatteryState.

        Corrección importante:
        - percentage en ROS suele venir entre 0.0 y 1.0.
        - si no hay percentage válido, se estima por voltaje.
        - ya no hay botones ni simulación manual de batería.
        """
        self.bateria_real_recibida = True

        volt = float(msg.voltage) if math.isfinite(float(msg.voltage)) and msg.voltage > 0.0 else float('nan')
        self.voltaje_bateria = volt

        pct = None
        try:
            raw_pct = float(msg.percentage)
            if math.isfinite(raw_pct) and raw_pct >= 0.0:
                pct = raw_pct * 100.0 if raw_pct <= 1.0 else raw_pct
        except Exception:
            pct = None

        if pct is None and math.isfinite(volt):
            pct = ((volt - self.cfg.bateria_voltaje_min) /
                   (self.cfg.bateria_voltaje_max - self.cfg.bateria_voltaje_min)) * 100.0
            self.bateria_fuente = "EST. VOLTAJE"
        else:
            self.bateria_fuente = "REAL"

        if pct is not None:
            self.porcentaje_bateria = int(max(0, min(100, round(pct))))

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_x = p.x
        self.odom_y = p.y
        self.odom_yaw = self.quaternion_a_yaw(q.x, q.y, q.z, q.w)
        self.tengo_odom = True
        self.mapa_ruta.actualizar_pose(self.odom_x, self.odom_y, self.odom_yaw)

    def quaternion_a_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ==========================================================
    # CALLBACK PRINCIPAL DEL LiDAR
    # ==========================================================
    def lidar_callback(self, msg):
        t0 = time.perf_counter()
        res = procesar_escaneo_lidar(msg)
        self.datos_filtrados = res

        self.dist_frente = res['dist_frente']
        self.dist_izq = res['dist_izq']
        self.dist_der = res['dist_der']
        self.dist_diag_der = res['dist_diag_der']
        self.dist_diag_izq = res['dist_diag_izq']
        self.dist_der_pared = res['dist_der_pared']
        self.dist_izq_pared = res['dist_izq_pared']
        self.pendiente_pared_der = res['pared_der_pendiente']
        self.pendiente_pared_izq = res['pared_izq_pendiente']
        self.pared_der_valida = res['pared_der_valida']
        self.pared_izq_valida = res['pared_izq_valida']
        self.puntos_pared_der = res['puntos_pared_der']
        self.puntos_pared_izq = res['puntos_pared_izq']
        self.ancho_pasillo = res['ancho_pasillo']
        self.error_centro_actual = res['error_centro']
        self.tipo_frente = res.get('tipo_frente', 'INDEFINIDO')
        self.frente_es_pasillo = bool(res.get('frente_es_pasillo', False))
        self.frente_es_esquina = bool(res.get('frente_es_esquina', False))
        self.frente_conecta_der = bool(res.get('frente_conecta_der', False))
        self.frente_conecta_izq = bool(res.get('frente_conecta_izq', False))
        self.frente_conecta_ambos = bool(res.get('frente_conecta_ambos', False))
        self.frente_confianza = float(res.get('frente_confianza', 0.0))
        self.front_class_sm = res.get('front_class_sm', 'NONE')
        self.front_ang_width = float(res.get('front_ang_width', 0.0))
        self.front_dist_sm = float(res.get('front_dist_sm', self.dist_frente))
        self.alpha_pared_der = float(res.get('pared_der_alpha', self.pendiente_pared_der))
        self.alpha_pared_izq = float(res.get('pared_izq_alpha', self.pendiente_pared_izq))
        self.len_pared_der = float(res.get('pared_der_len', 0.0))
        self.len_pared_izq = float(res.get('pared_izq_len', 0.0))
        self.total_puntos = res['num_puntos']
        self.t_proc_actual = res['tiempo_proc_ms']
        self.tiempos_proc.append(self.t_proc_actual)

        ahora_ref = time.time()
        if self.pared_der_valida:
            self.t_ultima_pared_der = ahora_ref
        if self.pared_izq_valida:
            self.t_ultima_pared_izq = ahora_ref
        self.actualizar_referencia_estable()

        if self.tengo_odom:
            self.mapa_ruta.actualizar_lidar(
                res,
                self.odom_x,
                self.odom_y,
                self.odom_yaw,
                pared_der_valida=self.pared_der_valida,
                pared_izq_valida=self.pared_izq_valida,
            )

        ahora = time.time()
        if self.ultimo_scan_time is not None:
            dt_scan = ahora - self.ultimo_scan_time
            if dt_scan > 0:
                self.tiempos_loop.append(dt_scan)
        self.ultimo_scan_time = ahora

        # Estado inicial o pausa: interfaz activa, motores quietos.
        if not self.robot_habilitado:
            self.estado_actual = "ESPERANDO INICIO"
            self.detener_robot()
            return

        if self.robot_pausado:
            self.detener_robot()
            return

        cmd = self.calcular_comando()
        self.vel_lineal = cmd.linear.x
        self.vel_angular = cmd.angular.z
        self.publisher.publish(cmd)
        self.registrar_historial_control()

        t_loop_ms = (time.perf_counter() - t0) * 1000.0
        self.tiempos_proc.append(max(self.t_proc_actual, t_loop_ms))

    # ==========================================================
    # FSM MODULAR DE MOVIMIENTO
    # ==========================================================
    def calcular_comando(self):
        if self.rodeo_caja.es_estado_rodeo(self.estado_actual):
            return self.rodear_obstaculo()

        if self.estado_actual == "ACERCAR_ESQUINA":
            return self.acercar_esquina()

        if self.estado_actual == "GIRO_EVITAR_FRENTE":
            return self.girar_por_frente()

        if self.estado_actual == "ALINEAR_POST_GIRO":
            return self.alinear_post_giro()

        if self.estado_actual == "AVANZAR_POST_ESQUINA":
            return self.avanzar_post_esquina()

        if self.estado_actual in {"CENTRAR_PASILLO", "ACERCARSE_DERECHA", "ALINEAR_PARED", "SEGUIR_PARED", "RECUPERAR_PARED", "SEGUIR_PARED_DERECHA_SUAVE", "SEGUIR_PARED_IZQUIERDA_SUAVE"}:
            return self.navegar_por_centro()

        if self.estado_actual == "BUSCAR_DERECHA":
            return self.buscar_referencia_lateral()

        if self.estado_actual == "REFERENCIA_BLOQUEADA":
            return self.control_con_referencia_bloqueada()

        return self.cmd_vel(0.0, 0.0)

    def persist_check(self, clave, condicion, frames=None):
        """Exige varios scans consecutivos antes de disparar una maniobra."""
        n = int(frames if frames is not None else self.cfg.persist_frames)
        c = self._persist.get(clave, 0)
        c = c + 1 if condicion else 0
        self._persist[clave] = c
        return c >= max(1, n)

    def reset_persistencia(self):
        self._persist.clear()

    # ----------------------------------------------------------
    # Bloqueo temporal de referencia lateral en esquinas
    # ----------------------------------------------------------
    def estados_maniobra_esquina(self):
        return {"ACERCAR_ESQUINA", "GIRO_EVITAR_FRENTE", "ALINEAR_POST_GIRO", "AVANZAR_POST_ESQUINA"}

    def zona_pre_esquina_activa(self):
        """Zona donde NO conviene cambiar de referencia lateral.

        Si el frente empieza a cerrarse o el clasificador ve esquina, mantener la
        referencia anterior evita que la pared izquierda sea tomada como guía falsa.
        """
        return (
            self.estado_actual in self.estados_maniobra_esquina()
            or self.dist_frente < self.cfg.frente_zona_bloqueo_referencia
            or self.frente_es_esquina
            or self.front_class_sm == "CORNER"
        )

    def actualizar_referencia_estable(self):
        """Actualiza la referencia estable con histéresis.

        La referencia NO cambia dentro de zona de esquina. Fuera de esa zona,
        permite cambiar solo si el nuevo lado aparece varios frames seguidos.
        """
        if not self.cfg.bloqueo_referencia_esquinas:
            return
        if self.referencia_bloqueada is not None:
            return

        lado = None
        if self.pared_der_valida and not self.pared_izq_valida:
            lado = "derecha"
        elif self.pared_izq_valida and not self.pared_der_valida:
            lado = "izquierda"
        elif self.pared_der_valida and self.pared_izq_valida:
            # En pasillo con ambas paredes se conserva la referencia anterior.
            if self.referencia_estable not in {"derecha", "izquierda"}:
                self.referencia_estable = self.cfg.referencia_preferida_esquina
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            return

        if lado is None:
            return

        # Cerca de esquina no se cambia a otro lado; solo se mantiene memoria.
        if self.zona_pre_esquina_activa() and lado != self.referencia_estable:
            return

        if lado == self.referencia_estable:
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            return

        if self.referencia_candidata == lado:
            self.frames_referencia_candidata += 1
        else:
            self.referencia_candidata = lado
            self.frames_referencia_candidata = 1

        if self.frames_referencia_candidata >= self.cfg.frames_cambio_referencia:
            self.referencia_estable = lado
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            self.get_logger().info(f"Referencia estable -> {self.referencia_estable}")

    def lado_con_memoria_reciente(self, lado):
        ahora = time.time()
        if lado == "derecha":
            return (ahora - self.t_ultima_pared_der) <= self.cfg.ref_memoria_lateral_seg
        if lado == "izquierda":
            return (ahora - self.t_ultima_pared_izq) <= self.cfg.ref_memoria_lateral_seg
        return False

    def bloquear_referencia_para_esquina(self):
        """Congela una referencia para toda la maniobra de esquina."""
        if not self.cfg.bloqueo_referencia_esquinas:
            return

        preferida = self.cfg.referencia_preferida_esquina

        # Para este circuito se prefiere derecha para evitar que la izquierda
        # capture el control antes de la 4ta esquina. Si alguna vez se desea el
        # otro sentido, cambia referencia_preferida_esquina en control_config.py.
        if preferida in {"derecha", "izquierda"}:
            ref = preferida
        elif self.referencia_estable in {"derecha", "izquierda"}:
            ref = self.referencia_estable
        elif self.pared_der_valida or self.lado_con_memoria_reciente("derecha"):
            ref = "derecha"
        elif self.pared_izq_valida or self.lado_con_memoria_reciente("izquierda"):
            ref = "izquierda"
        else:
            ref = "derecha"

        self.referencia_bloqueada = ref
        self.t_bloqueo_referencia = time.time()
        self.referencia_estable = ref
        self.get_logger().info(f"Referencia bloqueada en esquina -> {ref}")

    def liberar_bloqueo_referencia(self, motivo=""):
        if self.referencia_bloqueada is not None:
            ref = self.referencia_bloqueada
            self.referencia_bloqueada = None
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            txt = f"Referencia liberada ({ref})"
            if motivo:
                txt += f": {motivo}"
            self.get_logger().info(txt)

    def bloqueo_referencia_activo(self):
        if not self.cfg.bloqueo_referencia_esquinas:
            return False
        if self.referencia_bloqueada is None:
            return False
        if (time.time() - self.t_bloqueo_referencia) > self.cfg.max_tiempo_bloqueo_referencia:
            self.liberar_bloqueo_referencia("tiempo máximo")
            return False
        return True

    def debe_proteger_cambio_referencia_pre_esquina(self):
        """Evita saltar de derecha a izquierda justo antes de una esquina."""
        if not self.cfg.bloqueo_referencia_pre_esquina:
            return False
        if not self.zona_pre_esquina_activa():
            return False

        # Caso que estaba fallando: derecha perdida, izquierda visible, frente cerca.
        if self.referencia_estable == "derecha" and not self.pared_der_valida and self.pared_izq_valida:
            return True
        if self.referencia_estable == "izquierda" and not self.pared_izq_valida and self.pared_der_valida:
            return True
        return False

    def control_con_referencia_bloqueada(self):
        """Control seguro cuando no se permite cambiar de referencia.

        Si la referencia bloqueada está visible, la sigue. Si no está visible,
        avanza recto lento e ignora el lado contrario para no darse vuelta.
        """
        ref = self.referencia_bloqueada or self.referencia_estable or self.cfg.referencia_preferida_esquina

        # Si ya está muy cerca del frente, no seguir avanzando: preparar esquina.
        if self.dist_frente <= self.cfg.distancia_detencion_esquina or self.dist_frente <= self.cfg.frente_critico:
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        if ref == "derecha" and self.pared_der_valida:
            self.cambiar_estado("SEGUIR_PARED_DERECHA_SUAVE")
            return self.seguir_derecha_suave()

        if ref == "izquierda" and self.pared_izq_valida:
            self.cambiar_estado("SEGUIR_PARED_IZQUIERDA_SUAVE")
            return self.seguir_izquierda_suave()

        # Referencia perdida: no tomar el otro lado como guía.
        self.cambiar_estado("REFERENCIA_BLOQUEADA")
        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0
        v = min(self.cfg.avance_recto_ref_bloqueada, self.cfg.vel_lenta)
        return self.cmd_vel(v, 0.0)

    def navegar_por_centro(self):
        """
        Control principal: calcula el centro entre derecha e izquierda.
        Si ambas paredes están validadas por clusters conectados, usa centro.
        Si solo una pared es confiable, usa seguimiento lateral suave.
        """
        # Primero diferencia PASADIZO vs ESQUINA.
        # Pasadizo: paredes a ambos lados y frente libre -> pasar por el medio.
        # Esquina: frontal conectado a lateral -> acercarse, detenerse y girar controlado.
        en_cooldown = self.deteccion_esquina_bloqueada()

        # Clasificador fusionado: arco frontal corto = caja; arco largo/conectado = esquina.
        # Además usa persistencia para que un scan aislado no active maniobras.
        ahora = time.time()
        caja_candidata = (
            self.front_class_sm == "BOX"
            and self.dist_frente < self.cfg.frente_caja_detect
            and not en_cooldown
            and (ahora - self.t_ultima_caja) > self.cfg.cooldown_caja
        )
        esquina_candidata = (
            (self.frente_es_esquina or self.front_class_sm == "CORNER")
            and self.dist_frente < self.cfg.frente_esquina_detect
            and not en_cooldown
        )

        if self.persist_check('to_box', caja_candidata):
            self.contador_cajas += 1
            self.t_ultima_caja = ahora
            self.reset_persistencia()
            self.get_logger().warn(
                f"Caja detectada por arco corto #{self.contador_cajas} | "
                f"ancho={math.degrees(self.front_ang_width):.1f}°"
            )
            self.iniciar_rodeo()
            return self.rodear_obstaculo()

        if self.persist_check('to_corner', esquina_candidata):
            self.reset_persistencia()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        # Protección general: si se aproxima una esquina y pierde la referencia
        # derecha, NO cambia a seguir la izquierda; avanza recto lento hasta
        # confirmar esquina o recuperar la referencia.
        if self.bloqueo_referencia_activo() or self.debe_proteger_cambio_referencia_pre_esquina():
            if self.referencia_bloqueada is None:
                self.bloquear_referencia_para_esquina()
            return self.control_con_referencia_bloqueada()

        # Si está demasiado cerca y no es pasadizo, decide por el clasificador.
        # BOX -> rodeo; CORNER/conectado -> esquina controlada.
        if self.dist_frente < self.cfg.frente_critico:
            if en_cooldown:
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            if self.front_class_sm == "BOX" and (ahora - self.t_ultima_caja) > self.cfg.cooldown_caja:
                self.contador_cajas += 1
                self.t_ultima_caja = ahora
                self.reset_persistencia()
                self.get_logger().warn(f"Caja crítica detectada #{self.contador_cajas}: iniciando rodeo.")
                self.iniciar_rodeo()
                return self.rodear_obstaculo()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        # Caso ideal: ambas paredes laterales conectadas existen.
        if self.pared_der_valida and self.pared_izq_valida:
            self.ultimo_lado_pared = "ambas"
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.conducir_centrado()

        # Respaldo: solo pared derecha conectada.
        if self.pared_der_valida:
            self.ultimo_lado_pared = "derecha"
            self.cambiar_estado("SEGUIR_PARED_DERECHA_SUAVE")
            return self.seguir_derecha_suave()

        # Respaldo: solo pared izquierda conectada.
        # Cerca de una esquina, si la referencia estable es derecha, se ignora
        # la izquierda para evitar que el robot se dé la vuelta.
        if self.pared_izq_valida:
            if self.debe_proteger_cambio_referencia_pre_esquina():
                if self.referencia_bloqueada is None:
                    self.bloquear_referencia_para_esquina()
                return self.control_con_referencia_bloqueada()
            self.ultimo_lado_pared = "izquierda"
            self.cambiar_estado("SEGUIR_PARED_IZQUIERDA_SUAVE")
            return self.seguir_izquierda_suave()

        # Sin paredes laterales conectadas: no tomes la pared frontal como derecha.
        self.cambiar_estado("BUSCAR_DERECHA")
        return self.buscar_referencia_lateral()

    def conducir_centrado(self):
        ahora = time.time()
        dt = max(0.01, ahora - self.t_anterior)

        # Positivo = está más cerca de derecha, corregir a izquierda.
        error_centro = self.dist_izq_pared - self.dist_der_pared
        derivada = (error_centro - self.error_anterior) / dt

        # Orientación promedio de las líneas laterales. Si las paredes están inclinadas,
        # corrige suave sin pegarse a ninguna.
        # Heading real del segmento Split&Merge: mata zigzag y ayuda a quedar paralelo.
        error_ang = 0.5 * (self.alpha_pared_der + self.alpha_pared_izq)

        w = (
            self.cfg.kp_centrado * error_centro
            + self.cfg.kd_centrado * derivada
            - self.cfg.kp_angulo * error_ang
        )
        w = self.saturar(w, self.cfg.max_angular)

        # Si se acerca al frente, bajar velocidad pero no activar rodeo agresivo.
        factor_giro = 1.0 - min(abs(w) / self.cfg.max_angular, 1.0) * 0.45
        v = max(0.035, self.cfg.vel_crucero * factor_giro)
        if self.dist_frente < self.cfg.frente_alerta:
            v = min(v, self.cfg.vel_lenta)

        self.error_anterior = error_centro
        self.t_anterior = ahora
        self.error_centro_actual = error_centro
        self.error_ang_actual = error_ang
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def seguir_derecha_suave(self):
        """Fallback: sigue derecha usando SOLO la pared derecha conectada/anclada."""
        target = self.cfg.distancia_derecha_objetivo
        ahora = time.time()
        dt = max(0.01, ahora - self.t_anterior)

        if self.dist_frente < self.cfg.frente_critico:
            if self.deteccion_esquina_bloqueada():
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        error = self.dist_der_pared - target
        derivada = (error - self.error_anterior) / dt
        error_ang = self.alpha_pared_der

        # Control tipo CapyGuardian: distancia + heading(alpha) + derivativo.
        # Lejos de derecha -> gira derecha; cerca de derecha -> gira izquierda.
        w = (
            -self.cfg.kp_distancia * error
            -self.cfg.kd_distancia * derivada
            -self.cfg.k_alpha_guardian * error_ang
        )
        w = self.saturar(w, self.cfg.max_angular * 0.80)

        if self.dist_der_pared < self.cfg.derecha_muy_cerca:
            w = abs(self.cfg.max_angular * 0.65)  # abrir a izquierda

        v = self.cfg.vel_lenta if self.dist_frente < self.cfg.frente_alerta else self.cfg.vel_crucero * 0.85
        self.error_anterior = error
        self.t_anterior = ahora
        self.error_centro_actual = 0.0
        self.error_ang_actual = error_ang
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def seguir_izquierda_suave(self):
        """Fallback: sigue izquierda si no hay derecha conectada."""
        target = self.cfg.distancia_izquierda_objetivo
        ahora = time.time()
        dt = max(0.01, ahora - self.t_anterior)

        if self.dist_frente < self.cfg.frente_critico:
            if self.deteccion_esquina_bloqueada():
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        error = self.dist_izq_pared - target
        derivada = (error - self.error_anterior) / dt
        error_ang = self.alpha_pared_izq

        # Control tipo CapyGuardian: distancia + heading(alpha) + derivativo.
        # Lejos de izquierda -> gira izquierda; cerca de izquierda -> gira derecha.
        w = (
            self.cfg.kp_distancia * error
            + self.cfg.kd_distancia * derivada
            - self.cfg.k_alpha_guardian * error_ang
        )
        w = self.saturar(w, self.cfg.max_angular * 0.80)

        if self.dist_izq_pared < self.cfg.izquierda_muy_cerca:
            w = -abs(self.cfg.max_angular * 0.65)  # abrir a derecha

        v = self.cfg.vel_lenta if self.dist_frente < self.cfg.frente_alerta else self.cfg.vel_crucero * 0.85
        self.error_anterior = error
        self.t_anterior = ahora
        self.error_centro_actual = 0.0
        self.error_ang_actual = error_ang
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def normalizar_angulo(self, angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))

    def yaw_girado_desde_inicio(self):
        if self.yaw_inicio_giro is None or not self.tengo_odom:
            return None
        return abs(self.normalizar_angulo(self.odom_yaw - self.yaw_inicio_giro))

    def pared_post_giro_paralela(self):
        """Verifica si ya hay una pared lateral útil y casi paralela.

        Si la referencia está bloqueada, no se acepta el lado contrario como
        condición de salida. Esto evita cortar el giro porque apareció la pared
        izquierda y luego usarla como guía falsa.
        """
        der_ok = self.pared_der_valida and abs(self.pendiente_pared_der) < self.cfg.tolerancia_paralelo_post_giro
        izq_ok = self.pared_izq_valida and abs(self.pendiente_pared_izq) < self.cfg.tolerancia_paralelo_post_giro

        if self.referencia_bloqueada == "derecha":
            return der_ok or (self.pared_der_valida and self.pared_izq_valida)
        if self.referencia_bloqueada == "izquierda":
            return izq_ok or (self.pared_der_valida and self.pared_izq_valida)

        return der_ok or izq_ok or (self.pared_der_valida and self.pared_izq_valida)

    def distancia_odometrica_desde(self, x0, y0):
        if not self.tengo_odom:
            return None
        return math.hypot(self.odom_x - x0, self.odom_y - y0)

    def valor_por_esquina(self, valores, defecto):
        """Devuelve un ajuste según la esquina actual, usando contador circular."""
        try:
            if not valores:
                return defecto
            idx = int(self.indice_esquina_actual) % len(valores)
            return float(valores[idx])
        except Exception:
            return float(defecto)

    def deteccion_esquina_bloqueada(self):
        """Bloquea reentrada a esquina justo después de girar.

        Esto es lo que evita que en el tramo corto de la segunda esquina el robot
        vuelva a interpretar el frente como otra esquina y acumule giro hasta dar
        una vuelta completa.
        """
        dt = time.time() - self.t_ultima_esquina

        # IMPORTANTE: antes se desbloqueaba si el frente seguía crítico después
        # de 0.35 s. En la 4ta esquina eso provocaba reentradas a GIRO_EVITAR_FRENTE
        # y acumulaba giro hasta casi media vuelta. Ahora se bloquea por tiempo
        # y por distancia recorrida sin excepciones por frente crítico.
        if dt < self.cfg.cooldown_esquina:
            return True

        if dt < self.cfg.bloqueo_reentrada_esquina_seg:
            return True

        if self.tengo_odom and self.contador_esquinas > 0:
            dist = self.distancia_odometrica_desde(self.x_fin_ultima_esquina, self.y_fin_ultima_esquina)
            if dist is not None and dist < self.cfg.bloqueo_reentrada_esquina_m:
                return True

        return False

    def registrar_esquina_completada(self):
        if not self.esquina_en_proceso:
            return
        self.esquina_en_proceso = False
        self.contador_esquinas += 1
        self.t_ultima_esquina = time.time()
        self.x_fin_ultima_esquina = self.odom_x
        self.y_fin_ultima_esquina = self.odom_y
        self.get_logger().info(
            f"Esquina completada #{self.contador_esquinas} | próxima índice {(self.contador_esquinas % 4) + 1}"
        )

    def iniciar_avance_post_esquina(self):
        self.t_inicio_avance_post_esquina = time.time()
        self.x_inicio_avance_post_esquina = self.odom_x
        self.y_inicio_avance_post_esquina = self.odom_y
        self.yaw_salida_post_esquina = self.odom_yaw if self.tengo_odom else None
        self.sentido_salida_post_esquina = self.sentido_giro_frente
        self.indice_salida_post_esquina = self.indice_esquina_actual
        self.cambiar_estado("AVANZAR_POST_ESQUINA")

    def preparar_maniobra_esquina(self):
        """Prepara la esquina sin girar de golpe.

        El robot primero se acerca hasta una distancia fija y luego gira lentamente.
        En sentido antihorario siempre se toma la esquina hacia la izquierda.
        """
        if self.estado_actual in {"ACERCAR_ESQUINA", "GIRO_EVITAR_FRENTE", "ALINEAR_POST_GIRO", "AVANZAR_POST_ESQUINA"}:
            return

        self.indice_esquina_actual = self.contador_esquinas % 4
        self.esquina_en_proceso = True
        self.bloquear_referencia_para_esquina()
        self.get_logger().info(f"Preparando esquina #{self.indice_esquina_actual + 1}")

        if self.cfg.sentido_circuito_antihorario:
            self.sentido_giro_frente = self.cfg.sentido_giro_esquina  # +1 izquierda
        elif self.frente_conecta_der and not self.frente_conecta_izq:
            self.sentido_giro_frente = 1.0   # frontal conecta con derecha -> girar izquierda
        elif self.frente_conecta_izq and not self.frente_conecta_der:
            self.sentido_giro_frente = -1.0  # frontal conecta con izquierda -> girar derecha
        elif self.ultimo_lado_pared == "izquierda":
            self.sentido_giro_frente = 1.0
        elif self.ultimo_lado_pared == "derecha":
            self.sentido_giro_frente = -1.0
        else:
            self.sentido_giro_frente = self.cfg.sentido_giro_esquina

        self.yaw_inicio_giro = None
        self.error_anterior = 0.0
        self.t_anterior = time.time()
        self.cambiar_estado("ACERCAR_ESQUINA")

    # Compatibilidad con versiones anteriores del código.
    def preparar_giro_por_frente(self):
        self.preparar_maniobra_esquina()

    def acercar_esquina(self):
        """Se acerca lento a la esquina antes de girar.

        Si en realidad era pasadizo, vuelve al centrado. Si sí es esquina,
        se detiene a una distancia segura y recién ahí gira.
        """
        if self.frente_es_pasillo and self.dist_frente > self.cfg.frente_pasillo_libre:
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.navegar_por_centro()

        if self.dist_frente <= self.cfg.distancia_detencion_esquina or self.dist_frente <= self.cfg.frente_critico:
            self.t_inicio_giro_frente = time.time()
            self.yaw_inicio_giro = self.odom_yaw if self.tengo_odom else None
            self.cambiar_estado("GIRO_EVITAR_FRENTE")
            return self.girar_por_frente()

        # Acercamiento lento respetando el bloqueo de referencia.
        # Si el bloqueo es derecha, no usamos izquierda sola como guía aunque aparezca.
        if self.bloqueo_referencia_activo():
            ref = self.referencia_bloqueada
            if self.pared_der_valida and self.pared_izq_valida:
                cmd = self.conducir_centrado()
                cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
                cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.35)
                return cmd
            if ref == "derecha" and self.pared_der_valida:
                cmd = self.seguir_derecha_suave()
                cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
                cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.35)
                return cmd
            if ref == "izquierda" and self.pared_izq_valida:
                cmd = self.seguir_izquierda_suave()
                cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
                cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.35)
                return cmd
            # Referencia perdida: avanza recto lento, sin girar hacia la pared contraria.
            return self.cmd_vel(self.cfg.vel_acercar_esquina, 0.0)

        # Acercamiento lento manteniendo el centro o la pared visible.
        if self.pared_der_valida and self.pared_izq_valida:
            cmd = self.conducir_centrado()
            cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
            cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.45)
            return cmd

        if self.pared_der_valida:
            cmd = self.seguir_derecha_suave()
            cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
            cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.45)
            return cmd

        if self.pared_izq_valida:
            cmd = self.seguir_izquierda_suave()
            cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
            cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.45)
            return cmd

        return self.cmd_vel(self.cfg.vel_acercar_esquina, 0.0)

    def girar_por_frente(self):
        """Giro controlado de esquina.

        Ya no depende solo del tiempo ni sigue girando indefinidamente. Sale por:
        1) odometría: llegó al giro objetivo o al máximo permitido,
        2) LiDAR: volvió a ver pasadizo/pared lateral paralela,
        3) tiempo máximo de respaldo.
        """
        tiempo_girando = time.time() - self.t_inicio_giro_frente
        yaw_girado = self.yaw_girado_desde_inicio()
        yaw_obj = math.radians(self.valor_por_esquina(
            self.cfg.yaw_objetivo_esquinas_deg,
            self.cfg.yaw_objetivo_giro_esquina_deg
        ))
        yaw_max = math.radians(self.valor_por_esquina(
            self.cfg.yaw_max_esquinas_deg,
            self.cfg.yaw_max_giro_esquina_deg
        ))
        max_tiempo_giro = self.valor_por_esquina(
            self.cfg.max_tiempo_giro_esquinas,
            self.cfg.max_tiempo_giro_frente
        )
        factor_giro_actual = self.valor_por_esquina(self.cfg.factor_giro_esquinas, 1.0)

        hay_lateral_util = (
            self.pared_der_valida or self.pared_izq_valida or
            self.dist_der < self.cfg.lateral_reenganche_max or
            self.dist_izq < self.cfg.lateral_reenganche_max
        )

        pasadizo_recuperado = (
            self.frente_es_pasillo and
            self.dist_frente > self.cfg.frente_salida_libre and
            (self.pared_der_valida or self.pared_izq_valida)
        )

        paralelo_recuperado = (
            hay_lateral_util and
            self.dist_frente > self.cfg.frente_salida_con_lateral and
            self.pared_post_giro_paralela()
        )

        # Salida temprana: no espera a que el frente quede muy libre.
        # Si ya giró el objetivo y no está en distancia crítica, sale a alinear.
        frente_no_critico = self.dist_frente > self.cfg.frente_critico
        llego_objetivo = yaw_girado is not None and yaw_girado >= yaw_obj and frente_no_critico
        llego_maximo = yaw_girado is not None and yaw_girado >= yaw_max
        excedio_tiempo = yaw_girado is None and tiempo_girando > max_tiempo_giro

        # Si llegó al máximo o al tiempo máximo, NO se permite seguir corrigiendo
        # con giro grande. Sale directo a avance post-esquina para evitar media vuelta.
        if llego_maximo or excedio_tiempo:
            self.registrar_esquina_completada()
            self.iniciar_avance_post_esquina()
            return self.avanzar_post_esquina()

        if pasadizo_recuperado or paralelo_recuperado or llego_objetivo:
            self.registrar_esquina_completada()
            self.t_inicio_alinear_post_giro = time.time()
            self.lateral_estable_post_giro = 0
            self.cambiar_estado("ALINEAR_POST_GIRO")
            return self.alinear_post_giro()

        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0

        # Gira lento. Avanza muy poco solo si no está demasiado pegado al frente.
        v = self.cfg.vel_avance_esquina if self.dist_frente > self.cfg.frente_min_avance_giro else 0.0
        w = self.cfg.signo_giro * self.sentido_giro_frente * self.cfg.vel_giro_esquina * factor_giro_actual
        return self.cmd_vel(v, w)

    def alinear_post_giro(self):
        """Corrige la posición después del giro sin permitir otra vuelta completa."""
        tiempo_alineando = time.time() - self.t_inicio_alinear_post_giro

        estable = False
        if self.pared_der_valida and self.pared_izq_valida and self.dist_frente > self.cfg.frente_critico:
            estable = True
        elif self.referencia_bloqueada == "derecha":
            estable = (
                self.pared_der_valida
                and self.dist_frente > self.cfg.frente_critico
                and abs(self.pendiente_pared_der) < self.cfg.tolerancia_paralelo_post_giro
            )
        elif self.referencia_bloqueada == "izquierda":
            estable = (
                self.pared_izq_valida
                and self.dist_frente > self.cfg.frente_critico
                and abs(self.pendiente_pared_izq) < self.cfg.tolerancia_paralelo_post_giro
            )
        elif self.pared_izq_valida and self.dist_frente > self.cfg.frente_critico and abs(self.pendiente_pared_izq) < self.cfg.tolerancia_paralelo_post_giro:
            estable = True
        elif self.pared_der_valida and self.dist_frente > self.cfg.frente_critico and abs(self.pendiente_pared_der) < self.cfg.tolerancia_paralelo_post_giro:
            estable = True

        if estable:
            self.lateral_estable_post_giro += 1
        else:
            self.lateral_estable_post_giro = 0

        # Debe ver lateral estable algunos ciclos antes de soltar el giro.
        if self.lateral_estable_post_giro >= 3:
            self.iniciar_avance_post_esquina()
            return self.avanzar_post_esquina()

        # Corrección corta: no reinicia el giro grande.
        # En la 4ta esquina NO seguimos girando en el mismo sentido; salimos
        # recto y dejamos que el avance protegido estabilice el LiDAR.
        if self.indice_esquina_actual == 3:
            self.iniciar_avance_post_esquina()
            return self.avanzar_post_esquina()

        factor_alinear = min(1.0, self.valor_por_esquina(self.cfg.factor_giro_esquinas, 1.0))
        if tiempo_alineando < self.cfg.max_tiempo_alinear_post_giro:
            if self.dist_frente < self.cfg.frente_critico:
                # Si aún ve pared al frente, no aumentes el giro: contravolante muy suave.
                return self.cmd_vel(0.0, -self.cfg.signo_giro * self.sentido_giro_frente * self.cfg.vel_giro_esquina * 0.10 * factor_alinear)
            return self.cmd_vel(self.cfg.vel_lenta, self.cfg.signo_giro * self.sentido_giro_frente * self.cfg.vel_giro_esquina * 0.08 * factor_alinear)

        # Si no logró alinear en poco tiempo, igual sale a avance post-esquina.
        # Así no se queda girando hasta completar una vuelta.
        self.iniciar_avance_post_esquina()
        return self.avanzar_post_esquina()

    def avanzar_post_esquina(self):
        """Avanza protegido después de cada esquina antes de detectar otra.

        Arreglo para la 4ta esquina:
        - no vuelve a lanzar GIRO_EVITAR_FRENTE mientras está saliendo,
        - no usa todavía seguir pared/centrar porque eso puede interpretar mal
          la pared frontal como referencia lateral,
        - mantiene el rumbo con odometría si existe,
        - si todavía ve frente crítico, hace contravolante suave en vez de seguir
          girando hacia la esquina.
        """
        dt = time.time() - self.t_inicio_avance_post_esquina
        dist = self.distancia_odometrica_desde(
            self.x_inicio_avance_post_esquina,
            self.y_inicio_avance_post_esquina
        )

        # La 4ta esquina necesita una salida protegida un poco más larga porque
        # la pared de la caja queda muy cerca y el LiDAR la vuelve a leer como frente.
        min_m = self.cfg.avance_post_esquina_min_m
        min_seg = self.cfg.avance_post_esquina_min_seg
        max_seg = self.cfg.avance_post_esquina_max_seg
        if self.indice_salida_post_esquina == 3:
            min_m = max(min_m, self.cfg.avance_post_esquina4_min_m)
            min_seg = max(min_seg, self.cfg.avance_post_esquina4_min_seg)
            max_seg = max(max_seg, self.cfg.avance_post_esquina4_max_seg)

        avance_ok = False
        if dist is not None and dist >= min_m:
            avance_ok = True
        if dt >= min_seg and dist is None:
            avance_ok = True
        if dt >= max_seg:
            avance_ok = True

        if avance_ok:
            self.liberar_bloqueo_referencia("avance post-esquina completado")
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.navegar_por_centro()

        # Durante la salida obligatoria se ignora la detección de esquina.
        # Se avanza casi recto y solo se corrige rumbo.
        w_hold = 0.0
        if self.yaw_salida_post_esquina is not None and self.tengo_odom:
            yaw_err = self.normalizar_angulo(self.yaw_salida_post_esquina - self.odom_yaw)
            w_hold = self.saturar(self.cfg.kp_yaw_salida_post_esquina * yaw_err,
                                  self.cfg.max_w_salida_post_esquina)

        v = self.cfg.vel_avance_post_esquina

        # Si queda muy cerca del frente justo al salir, no volver a girar hacia
        # la izquierda. Hacer contravolante pequeño para no encarar la pared.
        if self.dist_frente < self.cfg.frente_critico:
            v = min(v, self.cfg.vel_lenta * 0.55)
            w_hold += -self.cfg.signo_giro * self.sentido_salida_post_esquina * self.cfg.contravolante_salida_esquina
        elif self.dist_frente < self.cfg.frente_alerta:
            v = min(v, self.cfg.vel_lenta * 0.80)

        w_hold = self.saturar(w_hold, self.cfg.max_w_salida_post_esquina)
        return self.cmd_vel(v, w_hold)

    def buscar_referencia_lateral(self):
        """
        Si no hay paredes laterales conectadas, gira suave a la derecha.
        Importante: no usa la pared frontal como derecha porque radar_utils exige
        cluster conectado al sector lateral ±90°.
        """
        if self.pared_der_valida or self.pared_izq_valida:
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.navegar_por_centro()

        if self.dist_frente < self.cfg.frente_critico:
            if self.deteccion_esquina_bloqueada():
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0
        # En antihorario conviene buscar la referencia por la izquierda
        # (isla central). En horario se conserva la búsqueda a la derecha.
        sentido_busqueda = 1.0 if self.cfg.sentido_circuito_antihorario else -1.0
        return self.cmd_vel(
            self.cfg.vel_busqueda_derecha,
            self.cfg.signo_giro * sentido_busqueda * self.cfg.giro_busqueda_derecha
        )

    # ----------------------------------------------------------
    # Rodeo de caja: se conserva, pero ya no se dispara por cualquier pared frontal.
    # ----------------------------------------------------------
    def iniciar_rodeo(self):
        self.rodeo_caja.iniciar(self.cambiar_estado)

    def rodear_obstaculo(self):
        v, w = self.rodeo_caja.actualizar(
            self.estado_actual,
            self.dist_frente,
            self.cambiar_estado
        )
        return self.cmd_vel(v, w)

    # ==========================================================
    # UTILIDADES DE CONTROL
    # ==========================================================
    def cambiar_estado(self, nuevo_estado):
        if self.estado_actual != nuevo_estado:
            self.estado_actual = nuevo_estado
            self.t_estado = time.time()
            self.ciclos_estable = 0
            if nuevo_estado not in {"CENTRAR_PASILLO", "SEGUIR_PARED_DERECHA_SUAVE", "SEGUIR_PARED_IZQUIERDA_SUAVE", "REFERENCIA_BLOQUEADA"}:
                self.reset_persistencia()
            self.get_logger().info(f"Estado -> {self.estado_actual}")

    def cmd_vel(self, v, w):
        """
        Crea Twist aplicando:
        - factor_velocidad solo a lineal
        - factor_angular solo a angular
        """
        cmd = Twist()

        v_base = float(v)
        w_base = float(w)

        # Memoria de recorrido: si todavía está en la primera vuelta y detecta que
        # está volviendo sobre sus propios pasos, aplica una corrección SUAVE.
        # Cuando el mapper detecta una vuelta completa, permite pasar otra vez.
        correccion = self.mapa_ruta.obtener_correccion_antiretorno(self.odom_x, self.odom_y, self.odom_yaw)
        self.evitar_retorno_activo = bool(correccion.get('activo', False))
        if self.evitar_retorno_activo and self.estado_actual not in {"ACERCAR_ESQUINA", "GIRO_EVITAR_FRENTE", "ALINEAR_POST_GIRO", "AVANZAR_POST_ESQUINA"}:
            w_base += self.cfg.signo_giro * correccion.get('bias_angular', 0.0)
            v_base *= self.cfg.factor_velocidad_antiretorno

        v_final = v_base * self.factor_velocidad
        w_final = w_base * self.factor_angular

        v_final = self.saturar(v_final, self.cfg.max_lineal_segura)
        w_final = self.saturar(w_final, self.cfg.max_angular_segura)

        cmd.linear.x = v_final
        cmd.angular.z = w_final
        self.vel_lineal = cmd.linear.x
        self.vel_angular = cmd.angular.z
        return cmd

    def saturar(self, valor, limite):
        return max(-limite, min(limite, valor))

    def detener_robot(self):
        cmd = Twist()
        self.vel_lineal = 0.0
        self.vel_angular = 0.0
        self.publisher.publish(cmd)

    def timer_seguridad_callback(self):
        if self.solicitud_salir or not self.robot_habilitado or self.robot_pausado:
            self.detener_robot()

    # ==========================================================
    # REPORTE / PLOT AL PAUSAR
    # ==========================================================
    def actualizar_trayectoria_plot(self):
        """Integra cmd_vel únicamente para dibujar la ruta al pausar.

        Esta función NO cambia estados, NO cambia velocidades y NO interviene
        en la lógica del robot. Solo guarda puntos para el reporte gráfico.
        """
        ahora = time.time()
        if self.plot_last_t is None:
            self.plot_last_t = ahora
            return

        dt = ahora - self.plot_last_t
        self.plot_last_t = ahora

        # Evita saltos si la Raspberry se congela o la ventana queda bloqueada.
        if dt <= 0.0 or dt > 0.60:
            return

        v = float(self.vel_lineal)
        w = float(self.vel_angular)

        if not math.isfinite(v) or not math.isfinite(w):
            return

        # Modelo diferencial simple: suficiente para graficar la forma del recorrido.
        yaw_mid = self.plot_yaw + 0.5 * w * dt
        dx = v * math.cos(yaw_mid) * dt
        dy = v * math.sin(yaw_mid) * dt
        self.plot_x += dx
        self.plot_y += dy
        self.plot_yaw = math.atan2(math.sin(self.plot_yaw + w * dt), math.cos(self.plot_yaw + w * dt))
        self.plot_total_distance += abs(v) * dt

        if not self.plot_path:
            self.plot_path.append((self.plot_x, self.plot_y))
            return

        ux, uy = self.plot_path[-1]
        if math.hypot(self.plot_x - ux, self.plot_y - uy) >= 0.015:
            self.plot_path.append((self.plot_x, self.plot_y))

    def distancia_de_path(self, path):
        total = 0.0
        if len(path) < 2:
            return 0.0
        for a, b in zip(path[:-1], path[1:]):
            total += math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        return total

    def registrar_historial_control(self):
        """Guarda muestras ligeras para graficar al presionar PAUSAR."""
        try:
            self.actualizar_trayectoria_plot()
            self.historial_control.append({
                "t": time.time(),
                "x": float(self.odom_x),
                "y": float(self.odom_y),
                "yaw": float(self.odom_yaw),
                "x_plot": float(self.plot_x),
                "y_plot": float(self.plot_y),
                "yaw_plot": float(self.plot_yaw),
                "v": float(self.vel_lineal),
                "w": float(self.vel_angular),
                "frente": float(self.dist_frente),
                "error_centro": float(self.error_centro_actual),
                "error_ang": float(self.error_ang_actual),
                "estado": str(self.estado_actual),
                "cajas": int(self.contador_cajas),
                "esquinas": int(self.contador_esquinas),
            })
        except Exception:
            # El historial nunca debe interrumpir el control del robot.
            pass

    def _promedio_historial(self, clave, absoluto=False):
        vals = []
        for item in self.historial_control:
            try:
                val = float(item.get(clave, float("nan")))
            except Exception:
                continue
            if math.isfinite(val):
                vals.append(abs(val) if absoluto else val)
        if not vals:
            return 0.0
        return sum(vals) / len(vals)

    def _max_historial(self, clave, absoluto=False):
        vals = []
        for item in self.historial_control:
            try:
                val = float(item.get(clave, float("nan")))
            except Exception:
                continue
            if math.isfinite(val):
                vals.append(abs(val) if absoluto else val)
        if not vals:
            return 0.0
        return max(vals)

    def generar_reporte_pausa(self):
        """Abre y guarda un plot de diagnóstico cuando se presiona PAUSAR.

        Incluye:
        - recorrido del robot por odometría,
        - pose actual con flecha de orientación,
        - puntos de pared/obstáculo guardados por LiDAR,
        - métricas útiles para revisar pérdidas de orientación.
        """
        try:
            mapa = self.mapa_ruta.obtener_datos_mapa()
            resumen = self.mapa_ruta.obtener_estado_resumen()
            odom_path = list(mapa.get("path", []))
            walls = list(mapa.get("walls", []))[-1200:]
            obstacles = list(mapa.get("obstacles", []))[-800:]
            rectangle = list(mapa.get("rectangle", []))

            # Ruta principal del plot:
            # 1) usar /odom si realmente tiene movimiento;
            # 2) si /odom está en cero, usar la trayectoria estimada con cmd_vel.
            path_cmd = list(self.plot_path)
            odom_dist = float(resumen.get('distance', 0.0))
            cmd_dist = float(self.plot_total_distance)
            odom_span = self.distancia_de_path(odom_path)
            usar_cmd = (len(odom_path) < 2 or odom_dist < 0.05 or odom_span < 0.05) and len(path_cmd) >= 2 and cmd_dist > 0.03

            if usar_cmd:
                path = path_cmd
                pose_x = float(self.plot_x)
                pose_y = float(self.plot_y)
                pose_yaw = float(self.plot_yaw)
                distancia_reporte = cmd_dist
                fuente_recorrido = "cmd_vel estimado"
                # Sin odometría real no se dibujan puntos LiDAR globales para evitar mapa falso.
                walls = []
                obstacles = []
                rectangle = []
            else:
                path = odom_path
                pose_x = float(self.odom_x)
                pose_y = float(self.odom_y)
                pose_yaw = float(self.odom_yaw)
                distancia_reporte = odom_dist
                fuente_recorrido = "/odom" if self.tengo_odom else "sin datos"

            # Si todavía no hay suficientes puntos, al menos dibuja la pose actual.
            if not path:
                path = [(pose_x, pose_y)]

            fig = plt.figure(figsize=(12.8, 7.2), facecolor="#0b0b0e")
            fig.canvas.manager.set_window_title("Reporte de pausa — recorrido del robot")
            gs = fig.add_gridspec(2, 2, width_ratios=[1.45, 1.0], height_ratios=[1.0, 0.72])

            ax_map = fig.add_subplot(gs[:, 0], facecolor="#111116")
            ax_info = fig.add_subplot(gs[0, 1], facecolor="#0b0b0e")
            ax_hist = fig.add_subplot(gs[1, 1], facecolor="#111116")

            fig.suptitle("Reporte al pausar: recorrido + diagnóstico", color="white", fontsize=14, weight="bold")

            # -------------------------
            # Mapa / recorrido
            # -------------------------
            if walls:
                wx, wy = zip(*walls)
                ax_map.scatter(wx, wy, s=4, alpha=0.45, label="paredes LiDAR")
            if obstacles:
                ox, oy = zip(*obstacles)
                ax_map.scatter(ox, oy, s=9, alpha=0.70, label="frente/obstáculos")
            if rectangle:
                rx, ry = zip(*rectangle)
                ax_map.plot(rx, ry, linestyle="--", linewidth=1.2, label="rectángulo estimado")
            if path:
                px, py = zip(*path)
                ax_map.plot(px, py, linewidth=2.2, label="recorrido robot")
                ax_map.scatter([px[0]], [py[0]], s=65, marker="o", label="inicio")
                ax_map.scatter([px[-1]], [py[-1]], s=75, marker="s", label="pausa")

            arrow_len = 0.24
            dx = arrow_len * math.cos(pose_yaw)
            dy = arrow_len * math.sin(pose_yaw)
            ax_map.arrow(
                pose_x, pose_y, dx, dy,
                width=0.012, head_width=0.075, head_length=0.09,
                length_includes_head=True, label="orientación"
            )
            ax_map.text(pose_x, pose_y, "  robot", color="white", fontsize=9, weight="bold")

            ax_map.set_title(f"Recorrido del robot ({fuente_recorrido})", color="#e0e0e0", fontsize=11)
            ax_map.set_xlabel("x [m]", color="#cccccc")
            ax_map.set_ylabel("y [m]", color="#cccccc")
            ax_map.grid(True, linestyle="--", alpha=0.25)
            ax_map.tick_params(colors="#cccccc")
            ax_map.set_aspect("equal", adjustable="box")
            ax_map.legend(loc="best", fontsize=8, facecolor="#111116", edgecolor="#777777")

            all_points = []
            all_points.extend(path)
            all_points.extend(walls)
            all_points.extend(obstacles)
            if all_points:
                xs = [p[0] for p in all_points]
                ys = [p[1] for p in all_points]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                span = max(max_x - min_x, max_y - min_y, 0.8)
                pad = max(0.35, span * 0.12)
                ax_map.set_xlim(min_x - pad, max_x + pad)
                ax_map.set_ylim(min_y - pad, max_y + pad)

            # -------------------------
            # Panel de datos
            # -------------------------
            ax_info.axis("off")
            if len(self.tiempos_loop) > 0:
                dt_prom = sum(self.tiempos_loop) / len(self.tiempos_loop)
                hz = 1.0 / dt_prom if dt_prom > 0 else 0.0
            else:
                hz = 0.0
            t_prom = (sum(self.tiempos_proc) / len(self.tiempos_proc)) if self.tiempos_proc else 0.0
            t_max = max(self.tiempos_proc) if self.tiempos_proc else 0.0
            promedio_v = self._promedio_historial("v", absoluto=True)
            promedio_w = self._promedio_historial("w", absoluto=True)
            max_w = self._max_historial("w", absoluto=True)
            estado_previo = self.estado_post_pausa or self.estado_actual
            bateria_txt = "sin datos"
            if self.porcentaje_bateria is not None:
                bateria_txt = f"{self.porcentaje_bateria}%"
                if math.isfinite(self.voltaje_bateria):
                    bateria_txt += f" / {self.voltaje_bateria:.1f} V"

            info = [
                "DATOS CLAVE",
                f"Estado antes de pausa : {estado_previo}",
                f"Fuente recorrido      : {fuente_recorrido}",
                f"Recorrido total        : {distancia_reporte:.2f} m",
                f"Vueltas detectadas     : {resumen.get('laps', 0)}",
                f"Esquinas completadas   : {self.contador_esquinas}",
                f"Cajas detectadas       : {self.contador_cajas}",
                f"Pose x/y               : {pose_x:+.2f}, {pose_y:+.2f} m",
                f"Yaw actual             : {math.degrees(pose_yaw):+.1f}°",
                f"Frente actual          : {self.dist_frente:.2f} m",
                f"Ancho pasillo          : {self.ancho_pasillo:.2f} m",
                f"Clase frontal          : {self.front_class_sm} / {math.degrees(self.front_ang_width):.0f}°",
                f"Referencia bloqueada   : {self.referencia_bloqueada or 'NO'}",
                f"Referencia estable     : {self.referencia_estable}",
                f"Anti-retorno activo    : {self.evitar_retorno_activo}",
                f"Dist. ruta antigua     : {resumen.get('revisit_distance', float('inf')):.2f} m",
                f"Velocidad prom.        : {promedio_v:.3f} m/s",
                f"Giro prom. |w|         : {promedio_w:.3f} rad/s",
                f"Giro máximo |w|        : {max_w:.3f} rad/s",
                f"Loop aprox.            : {hz:.1f} Hz",
                f"Proc. LiDAR prom/max   : {t_prom:.1f}/{t_max:.1f} ms",
                f"Batería                : {bateria_txt}",
            ]
            ax_info.text(
                0.02, 0.98, "\n".join(info),
                va="top", ha="left", color="#eeeeee", fontsize=9.2,
                family="monospace",
                bbox=dict(facecolor="#111116", edgecolor="#555555", boxstyle="round,pad=0.55")
            )

            # -------------------------
            # Historial corto
            # -------------------------
            hist = list(self.historial_control)[-350:]
            if hist:
                t0 = hist[0]["t"]
                ts = [h["t"] - t0 for h in hist]
                frente = [h["frente"] if math.isfinite(h["frente"]) else float("nan") for h in hist]
                error_centro = [h["error_centro"] for h in hist]
                giro = [h["w"] for h in hist]
                ax_hist.plot(ts, frente, linewidth=1.6, label="frente [m]")
                ax_hist.plot(ts, error_centro, linewidth=1.2, label="error centro [m]")
                ax_hist.plot(ts, giro, linewidth=1.2, label="angular [rad/s]")
                ax_hist.set_xlabel("últimos segundos", color="#cccccc")
                ax_hist.set_title("Historial antes de pausar", color="#e0e0e0", fontsize=10)
                ax_hist.grid(True, linestyle="--", alpha=0.25)
                ax_hist.legend(loc="best", fontsize=7, facecolor="#111116", edgecolor="#777777")
            else:
                ax_hist.text(0.5, 0.5, "Sin historial todavía", ha="center", va="center", color="#eeeeee")
            ax_hist.tick_params(colors="#cccccc")

            fig.tight_layout(rect=[0, 0, 1, 0.95])

            carpeta = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "reportes_pausa"))
            os.makedirs(carpeta, exist_ok=True)
            nombre = f"reporte_pausa_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            ruta = os.path.join(carpeta, nombre)
            fig.savefig(ruta, dpi=150, facecolor=fig.get_facecolor(), bbox_inches="tight")
            self.ultimo_reporte_pausa = ruta
            self.get_logger().warn(f"Reporte de pausa guardado: {ruta}")

            # Mostrar ventana sin bloquear la interfaz principal.
            fig.canvas.draw_idle()
            try:
                fig.show()
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"No se pudo generar el reporte de pausa: {e}")

    # ==========================================================
    # INTERFAZ
    # ==========================================================
    def update_interface(self, frame):
        if self.solicitud_salir:
            return self.ui.scatter_frente, self.ui.scatter_izq, self.ui.scatter_der

        self.ui.actualizar_graficos(
            self.datos_filtrados,
            distancia_objetivo=0.0
        )
        self.ui.actualizar_mapa_recorrido(self.mapa_ruta.obtener_datos_mapa())

        if len(self.tiempos_proc) > 0:
            t_prom = sum(self.tiempos_proc) / len(self.tiempos_proc)
            t_max = max(self.tiempos_proc)
        else:
            t_prom = 0.0
            t_max = 0.0

        if len(self.tiempos_loop) > 0:
            dt_prom = sum(self.tiempos_loop) / len(self.tiempos_loop)
            hz = 1.0 / dt_prom if dt_prom > 0 else 0.0
        else:
            hz = 0.0

        dist_der_mostrar = self.dist_der_pared if math.isfinite(self.dist_der_pared) else self.dist_der
        dist_izq_mostrar = self.dist_izq_pared if math.isfinite(self.dist_izq_pared) else self.dist_izq

        self.ui.renderizar_telemetria(
            self.estado_actual,
            [self.dist_frente, dist_izq_mostrar, dist_der_mostrar, self.dist_diag_der],
            [self.vel_lineal, self.vel_angular],
            [self.voltaje_bateria, self.porcentaje_bateria, self.bateria_fuente],
            [self.odom_x, self.odom_y, self.odom_yaw],
            [
                self.cfg.kp_centrado,
                self.cfg.kp_angulo,
                0.0,
                self.error_centro_actual,
                self.error_ang_actual,
                self.ancho_pasillo,
            ],
            [self.t_proc_actual, t_prom, t_max, hz],
            self.total_puntos,
            self.nombre_modo_velocidad,
            self.factor_angular,
            self.bateria_fuente,
            [self.pared_der_valida, self.pared_izq_valida, self.puntos_pared_der, self.puntos_pared_izq],
            self.mapa_ruta.obtener_estado_resumen(),
            [self.tipo_frente, self.front_class_sm, self.front_ang_width, self.contador_cajas],
        )
        return self.ui.scatter_frente, self.ui.scatter_izq, self.ui.scatter_der


def main(args=None):
    rclpy.init(args=args)
    sistema = SistemaControlBorde()

    ani = FuncAnimation(sistema.fig, sistema.update_interface, blit=False, interval=80)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(sistema), daemon=True)
    ros_thread.start()

    plt.show()

    print("Deteniendo de manera segura los actuadores del robot...")
    sistema.detener_robot()
    time.sleep(0.2)

    sistema.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
