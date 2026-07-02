Reto03 – Sistema de Navegación Autónoma con LiDAR para ROS 2

================================================== 
                    DESCRIPCIÓN
==================================================

Reto03 es un sistema de navegación autónoma desarrollado sobre ROS 2
para un robot móvil diferencial equipado con un sensor LiDAR. El
proyecto integra procesamiento de datos del sensor, una máquina de
estados (FSM), algoritmos de seguimiento de pasillos, detección de
esquinas, evasión de obstáculos, mapeo del recorrido e interfaz gráfica
para supervisión en tiempo real.

La arquitectura modular facilita el mantenimiento del software, la
reutilización de componentes y la incorporación de nuevas
funcionalidades.

================================================== 
            CARACTERÍSTICAS PRINCIPALES 
==================================================

-   Navegación autónoma basada en LiDAR.
-   Seguimiento centrado entre paredes.
-   Detección de pasillos, esquinas y obstáculos.
-   Máquina de estados para el control del robot.
-   Mapeo del recorrido mediante odometría.
-   Interfaz gráfica de monitoreo.
-   Reporte gráfico automático al pausar.
-   Configuración centralizada mediante parámetros.
-   Arquitectura modular basada en ROS 2.

================================================== 
                  ARQUITECTURA
==================================================

LiDAR (/scan) | v Procesamiento LiDAR | v Box Detector | v Behavior FSM
| v /cmd_vel | v Robot | | v v /odom GUI | v Route Mapper

================================================== 
            ESTRUCTURA DEL PROYECTO
==================================================

ejecutable/ ejecutable.py

src/ behavior_fsm/ box_detector/ route_mapper/ utils_scripts/

README.md

================================================== 
                    MÓDULOS
==================================================

behavior_fsm Implementa la lógica principal de navegación mediante una
máquina de estados encargada del seguimiento de pasillos, aproximación a
esquinas, giros controlados, alineación y recuperación.

box_detector Procesa la información del LiDAR para detectar paredes,
obstáculos y objetos mediante filtrado y agrupamiento de segmentos.

route_mapper Construye el mapa del recorrido utilizando la información
de odometría y las observaciones del LiDAR.

utils_scripts Contiene utilidades compartidas, configuración del
controlador, funciones geométricas, visualización e integración con la
interfaz.

ejecutable Punto de entrada del sistema e inicialización de la interfaz
gráfica.

================================================== 
              FLUJO DE FUNCIONAMIENTO
==================================================

1.  El LiDAR publica datos en /scan.
2.  El detector procesa las mediciones.
3.  La FSM clasifica el entorno.
4.  Se generan velocidades lineales y angulares.
5.  Se publica /cmd_vel.
6.  La odometría actualiza el mapa.
7.  La interfaz gráfica muestra el estado del sistema.

================================================== 
        FUNCIONALIDADES IMPLEMENTADAS 
==================================================

-   Seguimiento centrado entre paredes.
-   Clasificación de pasillos y esquinas.
-   Giros controlados por odometría y tiempo.
-   Protección contra sobregiros.
-   Bloqueo de referencias laterales durante maniobras.
-   Configuración independiente para cada esquina.
-   Soporte para recorridos antihorarios.
-   Integración del controlador CapyGuardian.
-   Persistencia temporal para reducir falsos positivos.
-   Memoria anti-retorno.
-   Mapeo del recorrido.
-   Visualización de obstáculos.
-   Reportes automáticos al pausar.
-   Lectura del estado de batería.

================================================== 
                  REQUISITOS
==================================================

-   ROS 2
-   Python 3
-   Sensor LiDAR compatible con /scan
-   Odometría disponible en /odom

================================================== 
                  EJECUCIÓN
==================================================

python3 ejecutable/ejecutable.py

================================================== 
                  RESULTADOS
==================================================

El sistema permite recorrer un circuito de forma autónoma manteniéndose
centrado entre paredes, detectando esquinas y obstáculos, generando un
mapa del recorrido y proporcionando información en tiempo real mediante
una interfaz gráfica.

================================================== 
                MEJORAS FUTURAS
==================================================

-   Integración con SLAM.
-   Planificación global de trayectorias.
-   Visión artificial.
-   Optimización automática de parámetros.
-   Exportación de mapas.

================================================== 
                    AUTOR
==================================================

Proyecto desarrollado para el Reto 03 de Robótica Móvil.

================================================== 
                  LICENCIA
==================================================

Uso académico.

