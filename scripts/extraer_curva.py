#!/usr/bin/env python3

import csv
import math
import re
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message



# CONFIGURACION


BAG = Path.home() / "ros2_ws/curva_manual_20261115_122839"

TOPICO_ODOM = "/model/buggy/odometry"
TOPICO_LIDAR = "/scan"

# Posicion inicial del buggy en Gazebo
X_INICIO = 4.10
Y_INICIO = 7.00
YAW_INICIO = -math.pi / 2


def normalizar(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def calcular_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


# IDENTIFICAR FORMATO DEL ROSBAG


if not BAG.is_dir():
    sys.exit(f"ERROR: No existe la carpeta {BAG}")

metadata = BAG / "metadata.yaml"

if not metadata.exists():
    sys.exit("ERROR: No se encontro metadata.yaml")

contenido = metadata.read_text()

coincidencia = re.search(
    r"^\s*storage_identifier:\s*([A-Za-z0-9_]+)",
    contenido,
    re.MULTILINE
)

if coincidencia:
    formato = coincidencia.group(1)
elif list(BAG.glob("*.mcap")):
    formato = "mcap"
elif list(BAG.glob("*.db3")):
    formato = "sqlite3"
else:
    sys.exit("ERROR: No se pudo identificar el formato del rosbag")



# LEER MENSAJES GRABADOS


print("\nLeyendo la grabacion...")
print("Carpeta:", BAG)
print("Formato:", formato)

reader = rosbag2_py.SequentialReader()

reader.open(
    rosbag2_py.StorageOptions(
        uri=str(BAG),
        storage_id=formato
    ),
    rosbag2_py.ConverterOptions("", "")
)

tipos = {
    t.name: t.type
    for t in reader.get_all_topics_and_types()
}

if TOPICO_ODOM not in tipos:
    sys.exit("ERROR: No hay mensajes de odometria en el rosbag")

ClaseOdom = get_message(tipos[TOPICO_ODOM])

ClaseLidar = (
    get_message(tipos[TOPICO_LIDAR])
    if TOPICO_LIDAR in tipos
    else None
)

odometria = []
lidar = []

tiempo_inicial = None

while reader.has_next():

    topico, datos, tiempo_ns = reader.read_next()

    if tiempo_inicial is None:
        tiempo_inicial = tiempo_ns

    tiempo = (tiempo_ns - tiempo_inicial) / 1e9

    if topico == TOPICO_ODOM:

        msg = deserialize_message(datos, ClaseOdom)

        pos = msg.pose.pose.position
        orientacion = msg.pose.pose.orientation

        yaw = calcular_yaw(orientacion)

        odometria.append({
            "tiempo": tiempo,
            "x": pos.x,
            "y": pos.y,
            "yaw": yaw,
            "velocidad": msg.twist.twist.linear.x
        })

    elif topico == TOPICO_LIDAR and ClaseLidar is not None:

        msg = deserialize_message(datos, ClaseLidar)

        validas = [
            distancia
            for distancia in msg.ranges
            if math.isfinite(distancia)
            and msg.range_min <= distancia <= msg.range_max
        ]

        lidar.append({
            "tiempo": tiempo,
            "distancia_min": min(validas) if validas else float("nan"),
            "distancia_media": (
                sum(validas) / len(validas)
                if validas else float("nan")
            ),
            "puntos_validos": len(validas)
        })

del reader

if not odometria:
    sys.exit("ERROR: La grabacion no contiene muestras de odometria")



# CONVERTIR A COORDENADAS DEL LABERINTO


x0 = odometria[0]["x"]
y0 = odometria[0]["y"]
yaw0 = odometria[0]["yaw"]

# Diferencia entre la orientacion de Gazebo y la odometria
offset_yaw = normalizar(YAW_INICIO - yaw0)

coseno = math.cos(offset_yaw)
seno = math.sin(offset_yaw)

distancia_total = 0.0
ultimo_x = None
ultimo_y = None

giro_acumulado = 0.0
yaw_anterior = yaw0

for fila in odometria:

    dx = fila["x"] - x0
    dy = fila["y"] - y0

    # Transformacion desde odometria hacia el mapa
    x_mapa = X_INICIO + coseno * dx - seno * dy
    y_mapa = Y_INICIO + seno * dx + coseno * dy

    yaw_mapa = normalizar(fila["yaw"] + offset_yaw)

    # El frente del buggy apunta hacia el eje -X del modelo
    rumbo_frontal = normalizar(yaw_mapa + math.pi)

    giro_acumulado += normalizar(fila["yaw"] - yaw_anterior)
    yaw_anterior = fila["yaw"]

    if ultimo_x is not None:
        distancia_total += math.hypot(
            x_mapa - ultimo_x,
            y_mapa - ultimo_y
        )

    ultimo_x = x_mapa
    ultimo_y = y_mapa

    fila["x_mapa"] = x_mapa
    fila["y_mapa"] = y_mapa
    fila["yaw_mapa"] = yaw_mapa
    fila["rumbo_frontal"] = rumbo_frontal
    fila["giro_acumulado"] = math.degrees(giro_acumulado)
    fila["distancia_total"] = distancia_total



# EXPORTAR CSV DE ODOMETRIA


archivo_odom = BAG / "trayectoria_curva.csv"

with archivo_odom.open("w", newline="") as f:

    escritor = csv.writer(f)

    escritor.writerow([
        "tiempo_s",
        "x_mapa_m",
        "y_mapa_m",
        "yaw_mapa_grados",
        "rumbo_frontal_grados",
        "giro_acumulado_grados",
        "distancia_recorrida_m",
        "velocidad_lineal_odom_m_s",
        "x_odom_m",
        "y_odom_m"
    ])

    for fila in odometria:

        escritor.writerow([
            round(fila["tiempo"], 4),
            round(fila["x_mapa"], 5),
            round(fila["y_mapa"], 5),
            round(math.degrees(fila["yaw_mapa"]), 3),
            round(math.degrees(fila["rumbo_frontal"]), 3),
            round(fila["giro_acumulado"], 3),
            round(fila["distancia_total"], 5),
            round(fila["velocidad"], 5),
            round(fila["x"], 5),
            round(fila["y"], 5)
        ])



# EXPORTAR CSV DEL RPLIDAR


archivo_lidar = BAG / "medidas_lidar.csv"

with archivo_lidar.open("w", newline="") as f:

    escritor = csv.writer(f)

    escritor.writerow([
        "tiempo_s",
        "distancia_minima_m",
        "distancia_media_m",
        "puntos_validos"
    ])

    for fila in lidar:

        escritor.writerow([
            round(fila["tiempo"], 4),
            round(fila["distancia_min"], 4),
            round(fila["distancia_media"], 4),
            fila["puntos_validos"]
        ])



# MOSTRAR RESULTADOS


primera = odometria[0]
ultima = odometria[-1]

print("\n======================================")
print("       MEDIDAS EXTRAIDAS")
print("======================================")

print(f"Muestras de odometria: {len(odometria)}")
print(f"Barridos del RPLIDAR: {len(lidar)}")

print(f"\nTiempo registrado: {ultima['tiempo'] - primera['tiempo']:.2f} s")

print("\nPOSICION INICIAL")
print(f"X = {primera['x_mapa']:.3f} m")
print(f"Y = {primera['y_mapa']:.3f} m")

print("\nPOSICION FINAL")
print(f"X = {ultima['x_mapa']:.3f} m")
print(f"Y = {ultima['y_mapa']:.3f} m")

print(f"\nDistancia recorrida: {distancia_total:.3f} m")

print(
    "Cambio neto de orientacion:",
    f"{ultima['giro_acumulado']:.2f} grados"
)

print("\nCSV de trayectoria:", archivo_odom)
print("CSV del RPLIDAR:", archivo_lidar)



# GRAFICAS


try:
    import matplotlib
    matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    xs = [fila["x_mapa"] for fila in odometria]
    ys = [fila["y_mapa"] for fila in odometria]

    # Grafica de trayectoria XY
    plt.figure(figsize=(8, 9))

    plt.plot(xs, ys, linewidth=2, label="Trayectoria medida")

    plt.scatter(xs[0], ys[0], s=90, label="Inicio")
    plt.scatter(xs[-1], ys[-1], s=90, label="Final")

    plt.xlabel("X del mapa (m)")
    plt.ylabel("Y del mapa (m)")
    plt.title("Curva realizada manualmente por el buggy")

    plt.grid(True)
    plt.axis("equal")
    plt.legend()

    grafica = BAG / "trayectoria_curva.png"

    plt.savefig(grafica, dpi=180, bbox_inches="tight")
    plt.close()

    # Grafica del giro durante la maniobra
    plt.figure(figsize=(10, 5))

    plt.plot(
        [fila["tiempo"] for fila in odometria],
        [fila["giro_acumulado"] for fila in odometria]
    )

    plt.xlabel("Tiempo (s)")
    plt.ylabel("Giro acumulado (grados)")
    plt.title("Orientacion durante la maniobra")
    plt.grid(True)

    grafica_giro = BAG / "giro_curva.png"

    plt.savefig(grafica_giro, dpi=180, bbox_inches="tight")
    plt.close()

    print("\nGraficas generadas:")
    print(grafica)
    print(grafica_giro)

except ImportError:
    print("\nNo se encontro matplotlib.")
    print("Los CSV se guardaron correctamente.")
    print("Para generar graficas: sudo apt install python3-matplotlib")


print("\nEXTRACCION FINALIZADA")
