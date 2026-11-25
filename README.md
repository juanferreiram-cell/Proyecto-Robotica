# RAUL - Navegación Autónoma con ROS 2 y Gazebo

## Descripción

Este repositorio contiene los archivos utilizados para el desarrollo del sistema de navegación autónoma del proyecto **RAUL (Robot Autónomo Uruguayo Ligero)**, realizado para la asignatura Tecnologías de Control y Robótica de UTEC.

El objetivo es simular el desplazamiento de un buggy dentro de un laboratorio, utilizando ROS 2, Gazebo y un sensor LiDAR para obtener información del entorno.

## Herramientas utilizadas

- ROS 2 Lyrical
- Gazebo Sim
- Python 3
- Fusion 360
- Onshape

## Archivos principales

| Archivo | Descripción |
|---|---|
| `recorrido_completo_buggy.py` | Ejecución del recorrido autónomo. |
| `resolver_laberinto_bfs.py` | Planificación de rutas mediante BFS. |
| `seguir_bfs.py` | Seguimiento del primer tramo de la ruta BFS. |
| `control_buggy.py` | Control manual del buggy. |
| `extraer_curva.py` | Extracción de trayectorias grabadas. |
| `ruta_curva_aprendida.csv` | Trayectoria de la primera curva. |
| `ruta_salida_aprendida.csv` | Trayectoria de la segunda curva. |
| `ruta_bfs.csv` | Ruta calculada mediante BFS. |
| `lidar_world.sdf` | Mundo de simulación en Gazebo. |
| `buggy_con_odom.urdf` | Modelo del buggy con odometría. |

## Integrantes

- Lucas Elizalde
- Juan Manuel Ferreira
- Ignacio Martínez
- Felipe Morrudo

**UTEC - Ingeniería en Mecatrónica**

Tecnologías de Control y Robótica - 2026