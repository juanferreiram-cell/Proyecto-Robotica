#!/usr/bin/env python3
"""Recorrido integrado del buggy en Gazebo: pasillo inicial + S1 + S2.

Utiliza los DOS CSV de las maniobras manuales que YA funcionaron en la
simulacion; no recalcula esas curvas ni modifica ningun otro archivo.

Mundo: empty. Buggy: buggy. ROS2 Lyrical + ros_gz_bridge para /scan y
/model/buggy/odometry. Probar con Gazebo visible y Ctrl+C a mano.

Sin argumentos -> diagnostico, no envia comandos.
--colocar       -> recolocar al inicio de la etapa --desde (sin mover aun).
--ejecutar      -> activar el seguimiento automatico desde --desde.
--desde inicio|s1|s2 (predeterminado: inicio).
"""

import argparse
import csv
import math
import re
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

BASE = Path.home() / 'ros2_ws'
RUTA_S1 = BASE / 'ruta_curva_aprendida.csv'
RUTA_S2 = BASE / 'ruta_salida_aprendida.csv'
MUNDO = 'empty'
MODELO = 'buggy'
ENTRE_EJES = .46
MAX_GIRO_FISICO = .223402

# Coordenadas de los inicios que se probaron previamente en Gazebo.
PUNTOS_INICIALES = {
    'inicio': (4.0961, -0.4815, -math.pi / 2),
    's1': (4.10, 7.00, -math.pi / 2),
    's2': (.76, 11.55, -math.pi / 2),
}

TOPICO_LB = '/model/buggy/joint/LB/cmd_vel'
TOPICO_RB = '/model/buggy/joint/RB/cmd_vel'
TOPICO_GIRO = '/model/buggy/joint/tubochasis/0/cmd_pos'


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_desde_cuaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def publicar_gz(topico, valor):
    proc = subprocess.run(
        ['gz', 'topic', '-t', topico, '-m', 'gz.msgs.Double',
         '-p', f'data: {valor:.6f}'],
        capture_output=True, text=True, timeout=5)
    if proc.returncode != 0:
        raise RuntimeError(f'Error publicando {topico}: {proc.stderr or proc.stdout}')


def detener():
    # Los comandos de velocidad de ruedas son independientes.
    for topico, valor in ((TOPICO_LB, 0.), (TOPICO_RB, 0.), (TOPICO_GIRO, 0.)):
        try:
            publicar_gz(topico, valor)
        except Exception as exc:
            print(f'ADVERTENCIA al detener {topico}: {exc}', flush=True)


def mandar(velocidad, direccion):
    # Con la orientacion de ejes del modelo: LB negativa / RB positiva
    # da marcha hacia adelante. Direccion positiva = izquierda.
    publicar_gz(TOPICO_GIRO, direccion)
    publicar_gz(TOPICO_LB, -velocidad)
    publicar_gz(TOPICO_RB, velocidad)


def pose_gazebo():
    p = subprocess.run(['gz', 'model', '-m', MODELO, '--pose'],
                       capture_output=True, text=True, timeout=15)
    if p.returncode:
        raise RuntimeError('No pude consultar el modelo en Gazebo: ' + p.stderr)
    patron = (r'(?m)^\s*\[\s*([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s*\]'
              r'\s*\n\s*\[\s*([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s*\]')
    m = re.search(patron, p.stdout)
    if m is None:
        raise RuntimeError('Gazebo devolvio una pose no reconocida:\n' + p.stdout)
    return tuple(float(x) for x in m.groups())


def colocar(nombre):
    x, y, yaw = PUNTOS_INICIALES[nombre]
    print(f'Ubicando buggy al inicio de {nombre}: X={x:.3f}, Y={y:.3f} ...')
    detener()
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "{MODELO}" '
           f'position: {{x: {x}, y: {y}, z: 0.30}} '
           f'orientation: {{x: 0, y: 0, z: {qz:.10f}, w: {qw:.10f}}}')
    p = subprocess.run([
        'gz', 'service', '-s', f'/world/{MUNDO}/set_pose',
        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '4000', '--req', req],
        capture_output=True, text=True, timeout=14)
    if p.returncode != 0 or 'data: true' not in p.stdout.lower():
        raise RuntimeError('No se pudo colocar el buggy:\n' + p.stdout + p.stderr)
    time.sleep(1.5)
    px, py, pz, _, _, yaw_actual = pose_gazebo()
    print(f'Pose real: X={px:.3f}, Y={py:.3f}, Z={pz:.3f}, yaw={math.degrees(yaw_actual):.1f}°')
    print('Comproba que quede bien apoyado en el suelo antes de --ejecutar.')


class Sensores(Node):
    def __init__(self):
        super().__init__('buggy_recorrido_integrado')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST,
                         depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock = threading.Lock()
        self.odom = None
        self.t_odom = 0.
        self.min_scan = None
        self.t_scan = 0.
        self.create_subscription(Odometry, '/model/buggy/odometry',
                                 self.recibir_odom, qos)
        self.create_subscription(LaserScan, '/scan', self.recibir_lidar, qos)

    def recibir_odom(self, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.odom = (p.x, p.y, yaw_desde_cuaternion(msg.pose.pose.orientation))
            self.t_odom = time.monotonic()

    def recibir_lidar(self, msg):
        validos = [v for v in msg.ranges if math.isfinite(v)
                   and msg.range_min <= v <= msg.range_max]
        with self.lock:
            self.min_scan = min(validos) if validos else None
            self.t_scan = time.monotonic()

    def foto(self):
        with self.lock:
            return self.odom, self.t_odom, self.min_scan, self.t_scan


def cargar_csv(archivo):
    if not archivo.is_file():
        raise FileNotFoundError(f'No se encontro {archivo}')
    with archivo.open(newline='') as f:
        filas = list(csv.DictReader(f))
    if len(filas) < 5:
        raise ValueError(f'Ruta demasiado corta: {archivo}')
    distancias = [float(f['s_m']) for f in filas]
    puntos = [(float(f['x_m']), float(f['y_m'])) for f in filas]
    if any(distancias[i] <= distancias[i-1] for i in range(1, len(distancias))):
        raise ValueError(f'El CSV debe tener s_m estrictamente creciente: {archivo}')
    return distancias, puntos


def crear_recta():
    # Primer pasillo de la ruta demostrada; se extiende un poco tras Y=7
    # para que la anticipacion del controlador no termine antes del corte.
    x, y0, _ = PUNTOS_INICIALES['inicio']
    ys = [y0 + i*.10 for i in range(81)]
    puntos = [(4.10, y) for y in ys]
    distancias = [i*.10 for i in range(len(ys))]
    return distancias, puntos


def interp(distancias, xy, objetivo):
    objetivo = min(max(distancias[0], objetivo), distancias[-1])
    for j in range(1, len(distancias)):
        if distancias[j] >= objetivo:
            delta = distancias[j]-distancias[j-1]
            a = (objetivo-distancias[j-1]) / max(delta, 1e-9)
            return (xy[j-1][0] + a*(xy[j][0]-xy[j-1][0]),
                    xy[j-1][1] + a*(xy[j][1]-xy[j-1][1]))
    return xy[-1]


def preparar_etapas(args):
    s1 = cargar_csv(RUTA_S1)
    s2 = cargar_csv(RUTA_S2)
    recta = crear_recta()
    # Comprobaciones para no integrar accidentalmente otras rutas.
    if math.hypot(s1[1][0][0]-4.10, s1[1][0][1]-7.00) > .05:
        raise RuntimeError('La primera S ya no empieza en (4.10, 7.00).')
    if math.hypot(s2[1][0][0]-.76, s2[1][0][1]-11.55) > .05:
        raise RuntimeError('La segunda S ya no empieza en (0.76, 11.55).')
    return [
        dict(nombre='Primer pasillo', clave='inicio', ruta=recta,
             destino=(4.10, 7.00), corte_y=7.00, velocidad=args.velocidad_recta,
             tiempo_max=240., tolerancia_pose=.22, tolerancia_yaw=math.radians(12)),
        dict(nombre='Primera S', clave='s1', ruta=s1,
             destino=(.76, 11.55), corte_y=11.55, velocidad=args.velocidad_s1,
             tiempo_max=250., tolerancia_pose=.20, tolerancia_yaw=math.radians(15)),
        dict(nombre='Segunda S / salida', clave='s2', ruta=s2,
             destino=s2[1][-1], corte_y=None, velocidad=args.velocidad_s2,
             tiempo_max=270., tolerancia_pose=.19, tolerancia_yaw=math.radians(15)),
    ]


def esperar_sensores(nodo, timeout=14):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        odo, t_odo, lidar, t_scan = nodo.foto()
        now = time.monotonic()
        if odo is not None and lidar is not None and now-t_odo < 1.5 and now-t_scan < 2:
            return odo
        time.sleep(.1)
    raise RuntimeError('No llegan odometria y RPLIDAR. Revisa el puente ROS/Gazebo.')


def verificar_seguridad(nodo, umbral):
    odo, t_odo, lidar, t_scan = nodo.foto()
    now = time.monotonic()
    if odo is None or now-t_odo > 1.5:
        raise RuntimeError('Se perdio la odometria; se detiene el buggy.')
    if lidar is None or now-t_scan > 2:
        raise RuntimeError('Se perdio el RPLIDAR; se detiene el buggy.')
    if lidar < umbral:
        raise RuntimeError(f'Obstaculo detectado a {lidar:.3f} m (umbral {umbral:.2f}).')
    return odo, lidar


def seguir_etapa(etapa, nodo, args, log):
    # Detener y reanclar coordenadas en CADA etapa, sin teletransportar.
    detener()
    time.sleep(.75)
    odo_inicial = esperar_sensores(nodo)
    px, py, pz, roll, pitch, yaw_gz = pose_gazebo()
    # Releer la odometria despues de consultar la pose de Gazebo.
    odo_inicial = esperar_sensores(nodo)
    clave = etapa['clave']
    esperado_x, esperado_y, yaw_esperado = PUNTOS_INICIALES[clave]
    error_pos = math.hypot(px-esperado_x, py-esperado_y)
    error_yaw = abs(envolver(yaw_gz-yaw_esperado))
    print(f'\n--- {etapa["nombre"].upper()} ---')
    print(f'Gazebo: X={px:.3f}, Y={py:.3f}, yaw={math.degrees(yaw_gz):.1f}°')
    print(f'Diferencia con inicio esperado: {error_pos:.3f} m / '
          f'{math.degrees(error_yaw):.1f}°', flush=True)
    if error_pos > etapa['tolerancia_pose'] or error_yaw > etapa['tolerancia_yaw']:
        raise RuntimeError(
            f'No coincide el comienzo de {etapa["nombre"]}. '
            'Detenido para evitar una trayectoria equivocada. '
            'Revisa la posicion real antes de continuar.')
    if abs(roll) > .18 or abs(pitch) > .18:
        raise RuntimeError('Buggy inclinado. Verifica las ruedas y el suelo.')
    if pz > .24 or pz < .08:
        raise RuntimeError(f'Altura inesperada Z={pz:.3f} m: revisa que este apoyado.')

    distancias, xy = etapa['ruta']
    desfase_yaw = envolver(yaw_gz - odo_inicial[2])
    c, s = math.cos(desfase_yaw), math.sin(desfase_yaw)
    idx = 0
    t0 = time.monotonic()
    ultima_impresion = -1e8

    while True:
        now = time.monotonic()
        if now - t0 > etapa['tiempo_max']:
            raise RuntimeError(f'Excedido tiempo maximo de {etapa["nombre"]}.')
        odo, lidar_min = verificar_seguridad(nodo, args.distancia_min)
        dx, dy = odo[0]-odo_inicial[0], odo[1]-odo_inicial[1]
        x = px + c*dx - s*dy
        y = py + s*dx + c*dy
        rumbo = envolver(odo[2]+desfase_yaw+math.pi)

        desde = max(0, idx-5)
        hasta = min(len(xy), idx+22)
        mejor = min(range(desde, hasta),
                    key=lambda j: math.hypot(xy[j][0]-x, xy[j][1]-y))
        idx = max(idx, mejor)
        desviacion = math.hypot(xy[mejor][0]-x, xy[mejor][1]-y)
        falta = math.hypot(xy[-1][0]-x, xy[-1][1]-y)
        if desviacion > args.desviacion_max:
            raise RuntimeError(f'Desviacion {desviacion:.3f} m en {etapa["nombre"]}; deteniendo.')

        # Cortar el primer pasillo y la primera S por su posicion Y,
        # preservando su control de curva completo hasta ese punto.
        if etapa['corte_y'] is not None and y >= etapa['corte_y']:
            print(f'\nETAPA COMPLETADA: {etapa["nombre"]} / X={x:.3f}, Y={y:.3f}')
            detener()
            return
        if etapa['corte_y'] is None and falta < args.tolerancia_final and idx >= len(xy)-6:
            print(f'\nSALIDA ALCANZADA: X={x:.3f}, Y={y:.3f}')
            detener()
            return

        obj_x, obj_y = interp(distancias, xy, distancias[idx]+args.lookahead)
        angulo_obj = math.atan2(obj_y-y, obj_x-x)
        err_angulo = envolver(angulo_obj-rumbo)
        distancia_obj = max(.08, math.hypot(obj_x-x, obj_y-y))
        curvatura = 2*math.sin(err_angulo)/distancia_obj
        direccion = math.atan(ENTRE_EJES*curvatura)
        direccion = max(-args.giro_max, min(args.giro_max, direccion))

        vel = etapa['velocidad']
        if clave != 'inicio' and distancias[idx] >= distancias[-1]-.65:
            vel = min(vel, .50)
        if lidar_min < .30:
            vel = min(vel, .55)
        if clave == 'inicio' and y > 6.7:
            vel = min(vel, .55)
        if clave == 's1' and y > 11.25:
            vel = min(vel, .50)

        mandar(vel, direccion)
        log.writerow([etapa['nombre'], f'{now-t0:.3f}', f'{x:.4f}',
                      f'{y:.4f}', f'{math.degrees(rumbo):.2f}',
                      f'{distancias[idx]:.3f}', f'{desviacion:.3f}',
                      f'{math.degrees(direccion):.2f}', f'{lidar_min:.3f}'])
        if now-ultima_impresion > 1.5:
            print(f'{etapa["nombre"]}: X={x:.2f} Y={y:.2f} '
                  f'avance={distancias[idx]:.2f} m '
                  f'desviacion={desviacion:.2f} m '
                  f'giro={math.degrees(direccion):+.1f}° '
                  f'LiDAR={lidar_min:.2f} m', flush=True)
            ultima_impresion = now
        time.sleep(.13)


def principal(args):
    etapas = preparar_etapas(args)
    inicio_idx = {'inicio': 0, 's1': 1, 's2': 2}[args.desde]
    print('RECORRIDO INTEGRADO DEL BUGGY')
    print(f'Inicio seleccionado: {args.desde}')
    print(f'Ruta S1: {RUTA_S1}')
    print(f'Ruta S2: {RUTA_S2}')
    for etapa in etapas[inicio_idx:]:
        s, puntos = etapa['ruta']
        print(f'  {etapa["nombre"]}: {len(puntos)} puntos; '
              f'desde {puntos[0]} hasta {puntos[-1]}')
    if args.colocar:
        colocar(args.desde)
        return
    if not args.ejecutar:
        print('\nMODO DIAGNOSTICO: no se ha movido el buggy.')
        print('Para ejecutarlo: python3 ~/ros2_ws/recorrido_completo_buggy.py --ejecutar')
        return

    # No empezar a publicar movimiento si los sensores no estan listos.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nodo = Sensores()
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    hilo = threading.Thread(target=executor.spin, daemon=True)
    hilo.start()
    archivo_log = BASE / ('registro_recorrido_completo_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
    try:
        esperar_sensores(nodo)
        with archivo_log.open('w', newline='') as f:
            log = csv.writer(f)
            log.writerow(['etapa', 'tiempo_etapa_s', 'x_m', 'y_m', 'rumbo_deg',
                          'avance_m', 'desviacion_m', 'manubrio_deg', 'lidar_min_m'])
            print(f'\nRegistro: {archivo_log}')
            print('INICIO AUTOMATICO. Ctrl+C para detener.\n')
            for etapa in etapas[inicio_idx:]:
                seguir_etapa(etapa, nodo, args, log)
                f.flush()
        print('\n===================================')
        print('RECORRIDO COMPLETO FINALIZADO')
        print('===================================')
    finally:
        print('\nDeteniendo ruedas y centrando manubrio...', flush=True)
        detener()
        executor.shutdown()
        hilo.join(timeout=4.)
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    grupo = p.add_mutually_exclusive_group()
    grupo.add_argument('--ejecutar', action='store_true', help='Mover el buggy automaticamente')
    grupo.add_argument('--colocar', action='store_true', help='Teletransportar al comienzo de --desde, SOLO antes de empezar')
    p.add_argument('--desde', choices=['inicio', 's1', 's2'], default='inicio',
                   help='Etapa inicial (default: inicio)')
    p.add_argument('--velocidad-recta', type=float, default=.75)
    p.add_argument('--velocidad-s1', type=float, default=.75)
    p.add_argument('--velocidad-s2', type=float, default=.70)
    p.add_argument('--lookahead', type=float, default=.55)
    p.add_argument('--giro-max', type=float, default=.22)
    p.add_argument('--distancia-min', type=float, default=.22)
    p.add_argument('--desviacion-max', type=float, default=.27)
    p.add_argument('--tolerancia-final', type=float, default=.14)
    args = p.parse_args()
    if min(args.velocidad_recta, args.velocidad_s1, args.velocidad_s2,
           args.lookahead, args.giro_max, args.distancia_min,
           args.desviacion_max, args.tolerancia_final) <= 0:
        p.error('Las velocidades, limites y distancias deben ser positivos.')
    if args.giro_max > MAX_GIRO_FISICO:
        p.error('El modelo limita el giro a 0.223402 rad.')
    try:
        principal(args)
    except KeyboardInterrupt:
        print('\nInterrumpido por el usuario. Buggy detenido.')
    except Exception as exc:
        print(f'\nERROR: {exc}')
        if args.ejecutar:
            print('Comproba en Gazebo que ambas ruedas hayan quedado detenidas.')
        raise SystemExit(1)
