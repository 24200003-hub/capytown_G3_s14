#!/usr/bin/env python3
"""
control_config.py
-----------------
Parámetros concentrados para tunear el robot sin tocar la lógica principal.
El robot ahora prioriza conducir por el centro del pasillo calculando la distancia
entre pared derecha e izquierda. La pared derecha/izquierda se valida por líneas
conectadas para no confundir la pared frontal con una pared lateral. Además se
integra mapeo simple del recorrido y anti-retorno suave.
"""
from dataclasses import dataclass


@dataclass
class ControlConfig:
    # Conducción por centro del pasillo
    distancia_derecha_objetivo: float = 0.25   # m, fallback cuando solo se ve derecha
    distancia_izquierda_objetivo: float = 0.25 # m, fallback cuando solo se ve izquierda
    tolerancia_distancia: float = 0.040        # m
    tolerancia_angulo: float = 0.11            # pendiente aprox. de pared para considerar paralelo
    ciclos_estable_necesarios: int = 6

    # Ganancias del controlador
    kp_centrado: float = 1.05                  # corrige hacia el medio entre izquierda y derecha
    kd_centrado: float = 0.18                  # reduce zigzag al centrar
    kp_distancia: float = 1.25                 # fallback lateral
    kd_distancia: float = 0.20                 # fallback lateral
    kp_angulo: float = 0.82                    # corrección suave por orientación de paredes
    k_alpha_guardian: float = 1.05              # término heading tipo CapyGuardian (paralelismo)
    persist_frames: int = 4                     # frames seguidos antes de cambiar a esquina/caja

    # Velocidades base. La interfaz aplica factor lento/medio/rápido.
    vel_crucero: float = 0.105                 # m/s
    vel_acercamiento: float = 0.060            # m/s
    vel_lenta: float = 0.040                   # m/s
    vel_rodeo: float = 0.072                   # m/s
    vel_giro: float = 0.36                     # rad/s, giro general más suave

    # Esquinas del circuito en sentido ANTIHORARIO:
    # ROS usa angular.z positivo para girar a la izquierda.
    sentido_circuito_antihorario: bool = True
    sentido_giro_esquina: float = 1.0          # +1 izquierda, -1 derecha
    vel_giro_esquina: float = 0.20             # rad/s, giro lento/controlado para no pasarse
    vel_avance_esquina: float = 0.000          # m/s, avance mínimo mientras gira y sigue sensando
    frente_min_avance_giro: float = 0.34       # m, si hay menos se gira sin avanzar
    frente_salida_con_lateral: float = 0.30    # m, sale si reaparece pared lateral
    frente_salida_libre: float = 0.38          # m, frente suficiente para volver a avanzar
    max_tiempo_giro_frente: float = 4.20       # s, respaldo si no hay odometría
    lateral_reenganche_max: float = 1.15       # m, pared lateral útil para reincorporarse

    # Análisis frontal: diferencia entre PASADIZO y ESQUINA.
    frente_pasillo_libre: float = 0.46         # m, si está libre y hay paredes a ambos lados: pasadizo
    frente_esquina_detect: float = 0.68        # m, empieza a analizar pared frontal conectada
    frente_caja_detect: float = 0.43            # m, arco frontal corto = caja/obstáculo
    cooldown_caja: float = 2.00                 # s, evita contar/iniciar la misma caja varias veces
    distancia_detencion_esquina: float = 0.36  # m, se acerca hasta aquí antes de girar
    vel_acercar_esquina: float = 0.028         # m/s, acercamiento lento antes del giro
    yaw_objetivo_giro_esquina_deg: float = 64.0 # grados; sale temprano para alinear, no para completar 90° de golpe
    yaw_max_giro_esquina_deg: float = 78.0     # grados; nunca permite que una esquina se vuelva vuelta completa
    # Ajustes por esquina del circuito. La segunda suele estar más cerca, por eso
    # se gira menos y con más cuidado para no perder orientación.
    yaw_objetivo_esquinas_deg: tuple = (64.0, 56.0, 62.0, 38.0)
    yaw_max_esquinas_deg: tuple = (78.0, 70.0, 76.0, 48.0)
    max_tiempo_giro_esquinas: tuple = (3.60, 2.70, 3.30, 1.45)
    factor_giro_esquinas: tuple = (1.00, 0.82, 0.92, 0.42)
    tolerancia_paralelo_post_giro: float = 0.18 # pendiente aprox. para considerar paralelo
    max_tiempo_alinear_post_giro: float = 0.65 # s, corrección corta después del giro; bajo para no sobregirar en esquina 4
    cooldown_esquina: float = 1.10             # s, evita reentrar a giro y acumular vueltas
    avance_post_esquina_min_m: float = 0.16    # m, avance mínimo antes de volver a detectar esquina
    avance_post_esquina_min_seg: float = 0.85  # s, bloqueo corto post-giro para la segunda esquina
    avance_post_esquina_max_seg: float = 2.00  # s, no se queda avanzando si el tramo es corto
    vel_avance_post_esquina: float = 0.055     # m/s, avance lento de estabilización

    # Salida protegida especial para la 4ta esquina: reduce reentrada y contravolantea.
    avance_post_esquina4_min_m: float = 0.28   # m, mínimo de salida en esquina 4
    avance_post_esquina4_min_seg: float = 1.35 # s, mínimo de salida sin reanalizar esquina
    avance_post_esquina4_max_seg: float = 2.60 # s, respaldo si odometría no mide bien
    kp_yaw_salida_post_esquina: float = 1.15   # mantiene rumbo al salir de la esquina
    max_w_salida_post_esquina: float = 0.13    # rad/s, corrección muy suave
    contravolante_salida_esquina: float = 0.055 # rad/s, evita seguir girando contra la pared

    bloqueo_reentrada_esquina_m: float = 0.25  # m, ignora esquina hasta separarse un poco
    bloqueo_reentrada_esquina_seg: float = 1.65 # s, respaldo por tiempo si no hay odometría

    # Bloqueo de referencia lateral en todas las esquinas.
    # Evita que el robot pierda derecha y use izquierda como guía falsa antes
    # de una esquina, que era lo que causaba la vuelta en la 4ta esquina.
    bloqueo_referencia_esquinas: bool = True
    bloqueo_referencia_pre_esquina: bool = True
    referencia_preferida_esquina: str = "derecha"
    frente_zona_bloqueo_referencia: float = 0.72
    ref_memoria_lateral_seg: float = 1.40
    frames_cambio_referencia: int = 5
    avance_recto_ref_bloqueada: float = 0.032
    max_tiempo_bloqueo_referencia: float = 4.00

    max_angular: float = 0.50                  # rad/s antes del factor angular
    max_lineal_segura: float = 0.22            # límite final de seguridad
    max_angular_segura: float = 1.10           # límite final de seguridad

    # Selector de 3 velocidades desde GUI
    modo_velocidad_inicial: int = 2            # 1=lento, 2=medio, 3=rápido
    factor_lento: float = 0.65
    factor_medio: float = 1.00
    factor_rapido: float = 1.30

    # Control independiente de velocidad angular desde GUI
    factor_angular_inicial: float = 0.80
    factor_angular_min: float = 0.35
    factor_angular_max: float = 1.60
    factor_angular_paso: float = 0.10

    # Batería: indicador leído desde /battery. Sin botones manuales.
    bateria_voltaje_min: float = 10.5
    bateria_voltaje_max: float = 12.6

    # Umbrales de seguridad
    frente_alerta: float = 0.46                # m, reduce velocidad si algo aparece al frente
    frente_critico: float = 0.30               # m, gira para no chocar
    derecha_muy_cerca: float = 0.13            # m, evita rozar pared derecha
    izquierda_muy_cerca: float = 0.13          # m, evita rozar pared izquierda
    lateral_perdida: float = 1.50              # m, pared lateral demasiado lejos/no confiable
    vel_busqueda_derecha: float = 0.00
    giro_busqueda_derecha: float = 0.24        # giro suave hacia derecha si no hay paredes

    # Tiempos de rodeo para secuencia solicitada: IZQ -> DER -> DER -> IZQ
    # Bajados para que no gire demasiado.
    rodeo_giro_izq_seg: float = 0.52
    rodeo_avance_1_seg: float = 0.45
    rodeo_giro_der_1_seg: float = 0.44
    rodeo_avance_2_seg: float = 0.82
    rodeo_giro_der_2_seg: float = 0.44
    rodeo_avance_3_seg: float = 0.48
    rodeo_giro_izq_final_seg: float = 0.44

    # Memoria de recorrido / anti-retorno
    factor_velocidad_antiretorno: float = 0.72  # reduce avance si detecta que vuelve sobre sus pasos

    # Si el robot gira al revés, cambia a -1.0
    signo_giro: float = 1.0
