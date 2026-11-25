import argparse
import csv
import math
import re
import subprocess
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# CONFIGURACION
VELOCIDAD = 1.0
ANGULO_MAXIMO = 0.20
KP = 1.5
DISTANCIA_SEGURIDAD = 0.25
DESVIACION_MAXIMA = 0.30

LB = "/model/buggy/joint/LB/cmd_vel"
RB = "/model/buggy/joint/RB/cmd_vel"
DIRECCION = "/model/buggy/joint/tubochasis/0/cmd_pos"

RUTA = Path.home() / "ros2_ws/ruta_bfs.csv"


def enviar(topico, valor):
    subprocess.run(
        [
            "gz", "topic",
            "-t", topico,
            "-m", "gz.msgs.Double",
            "-p", f"data: {valor}"
        ],
        check=True,
        timeout=5
    )


def mover(velocidad):
    enviar(LB, -velocidad)
    enviar(RB, velocidad)


def detener():
    for topico in (LB, RB):
        try:
            enviar(topico, 0.0)
        except Exception as error:
            print("Error al detener:", error)

    try:
        enviar(DIRECCION, 0.0)
    except Exception:
        pass


def limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


def normalizar(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def obtener_pose_gazebo():
    resultado = subprocess.run(
        ["gz", "model", "-m", "buggy", "--pose"],
        capture_output=True,
        text=True,
        check=True,
        timeout=10
    )

    patron = r"\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]"
    valores = re.findall(patron, resultado.stdout)

    if len(valores) < 2:
        raise RuntimeError("No pude leer la posicion del buggy en Gazebo.")

    x, y, z = map(float, valores[0])
    roll, pitch, yaw = map(float, valores[1])

    return x, y, z, yaw


def leer_primer_tramo():
    with open(RUTA, newline="") as archivo:
        puntos = [
            (float(fila["x"]), float(fila["y"]))
            for fila in csv.DictReader(archivo)
        ]

    if len(puntos) < 2:
        raise RuntimeError("La ruta BFS no tiene suficientes puntos.")

    dx = puntos[1][0] - puntos[0][0]
    dy = puntos[1][1] - puntos[0][1]

    longitud = math.hypot(dx, dy)

    if longitud == 0:
        raise RuntimeError("Los primeros puntos BFS son iguales.")

    ux = dx / longitud
    uy = dy / longitud

    # Buscar el primer cambio de direccion
    esquina = puntos[-1]

    for i in range(2, len(puntos)):
        dx2 = puntos[i][0] - puntos[i - 1][0]
        dy2 = puntos[i][1] - puntos[i - 1][1]

        if abs(dx2 * uy - dy2 * ux) > 1e-5:
            esquina = puntos[i - 1]
            break

    return ux, uy, esquina


class Sensores(Node):

    def __init__(self):
        super().__init__("seguir_bfs")

        self.x = None
        self.y = None
        self.yaw = None

        self.ultima_odom = None
        self.ultima_scan = None
        self.distancia_obstaculo = None

        self.create_subscription(
            Odometry,
            "/model/buggy/odometry",
            self.odom_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y

        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        self.ultima_odom = time.monotonic()

    def scan_callback(self, msg):
        distancias = [
            d for d in msg.ranges
            if math.isfinite(d)
            and msg.range_min <= d <= msg.range_max
        ]

        self.distancia_obstaculo = (
            min(distancias) if distancias else None
        )

        self.ultima_scan = time.monotonic()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--metros",
        type=float,
        default=1.0,
        help="Distancia del primer tramo a recorrer"
    )

    args = parser.parse_args()

    if args.metros <= 0:
        raise SystemExit("La distancia debe ser positiva.")

    ux, uy, esquina = leer_primer_tramo()

    print("\n========== SEGUIDOR BFS ==========")
    print(f"Direccion BFS: ({ux:.2f}, {uy:.2f})")
    print(f"Primera esquina: {esquina}")
    print(f"Distancia solicitada: {args.metros:.2f} m")

    rclpy.init()
    nodo = Sensores()

    # Procesar odometria y LiDAR en un hilo independiente
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)

    hilo = threading.Thread(
        target=executor.spin,
        daemon=True
    )
    hilo.start()

    try:
        print("\nEsperando odometria y RPLIDAR...")

        limite = time.monotonic() + 12

        while nodo.x is None or nodo.ultima_scan is None:
            time.sleep(0.1)

            if time.monotonic() > limite:
                raise RuntimeError(
                    "No llegan datos de odometria o del RPLIDAR."
                )

        # Posicion real de Gazebo
        x0, y0, z0, yaw_gz = obtener_pose_gazebo()

        # Referencia inicial de odometria
        ox0 = nodo.x
        oy0 = nodo.y
        oyaw0 = nodo.yaw

        # Transformacion del marco odom al marco del mapa
        offset_yaw = normalizar(yaw_gz - oyaw0)

        rumbo_bfs = math.atan2(uy, ux)
        rumbo_frente = normalizar(yaw_gz + math.pi)

        error_inicial = normalizar(
            rumbo_bfs - rumbo_frente
        )

        print(f"\nPosicion Gazebo: X={x0:.3f}, Y={y0:.3f}")
        print(f"Altura: Z={z0:.3f}")
        print(f"Orientacion del frente: {math.degrees(rumbo_frente):.1f}°")
        print(f"Orientacion BFS: {math.degrees(rumbo_bfs):.1f}°")

        if abs(error_inicial) > math.radians(20):
            raise RuntimeError(
                "El buggy no esta orientado hacia el primer pasillo."
            )

        # No avanzar hasta la esquina durante esta prueba
        distancia_esquina = (
            (esquina[0] - x0) * ux
            + (esquina[1] - y0) * uy
        )

        distancia_objetivo = min(
            args.metros,
            distancia_esquina - 1.5
        )

        if distancia_objetivo <= 0:
            raise RuntimeError(
                "El buggy esta demasiado cerca de la primera esquina."
            )

        print(f"Avance autorizado: {distancia_objetivo:.2f} m")
        print("\nIniciando seguimiento...\n")

        enviar(DIRECCION, 0.0)
        mover(VELOCIDAD)

        inicio = time.monotonic()
        ultima_orden = 0
        ultima_impresion = 0

        while True:
            time.sleep(0.1)

            ahora = time.monotonic()

            if ahora - nodo.ultima_odom > 1.5:
                print("PARADA: odometria interrumpida.")
                break

            if ahora - nodo.ultima_scan > 2.0:
                print("PARADA: RPLIDAR interrumpido.")
                break

            # Seguridad conservadora en todas las direcciones
            if (
                nodo.distancia_obstaculo is not None
                and nodo.distancia_obstaculo < DISTANCIA_SEGURIDAD
            ):
                print("PARADA: obstaculo demasiado cercano.")
                break

            # Convertir odometria a coordenadas del mapa
            dx = nodo.x - ox0
            dy = nodo.y - oy0

            c = math.cos(offset_yaw)
            s = math.sin(offset_yaw)

            x = x0 + c*dx - s*dy
            y = y0 + s*dx + c*dy

            yaw_mapa = normalizar(nodo.yaw + offset_yaw)
            rumbo_actual = normalizar(yaw_mapa + math.pi)

            # Avance sobre la direccion calculada por BFS
            avance = (x-x0)*ux + (y-y0)*uy

            # Desviacion lateral respecto al centro inicial
            lateral = -(x-x0)*uy + (y-y0)*ux

            if avance >= distancia_objetivo:
                print("\nTRAMO COMPLETADO")
                break

            if abs(lateral) > DESVIACION_MAXIMA:
                print("\nPARADA: desviacion lateral excesiva.")
                break

            # Corregir suavemente la direccion
            rumbo_deseado = rumbo_bfs - math.atan(1.5*lateral)

            error = normalizar(rumbo_deseado - rumbo_actual)

            if abs(error) > math.radians(35):
                print("\nPARADA: error de orientacion excesivo.")
                break

            angulo = limitar(
                KP * error,
                -ANGULO_MAXIMO,
                ANGULO_MAXIMO
            )

            if ahora - ultima_orden >= 0.35:
                enviar(DIRECCION, angulo)
                ultima_orden = ahora

            if ahora - ultima_impresion >= 1.0:
                print(
                    f"X={x:.2f} | Y={y:.2f} | "
                    f"Avance={avance:.2f} m | "
                    f"Desviacion={lateral:.2f} m | "
                    f"Manubrio={math.degrees(angulo):.1f}°"
                )
                ultima_impresion = ahora

            if ahora - inicio > 180:
                print("\nPARADA: tiempo maximo.")
                break

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")

    except Exception as error:
        print("\nERROR:", error)

    finally:
        detener()
        executor.shutdown()
        hilo.join(timeout=2.0)
        nodo.destroy_node()
        rclpy.shutdown()
        print("Buggy detenido.")


if __name__ == "__main__":
    main()
