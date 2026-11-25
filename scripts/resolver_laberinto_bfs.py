from pathlib import Path
from collections import deque
import csv
import math

import numpy as np
import matplotlib.pyplot as plt


# CONFIGURACION DEL LABERINTO
ARCHIVO = Path.home() / "ros2_ws/models/mapa_lab/meshes/mapa_lab.obj"

ESCALA = 0.001          # Fusion 360: mm -> metros
RESOLUCION = 0.10       # Cada celda mide 10 cm
ALTURA_MINIMA = 0.08    # Detectar superficies verticales
MARGEN_BUGGY = 0.20     # Distancia de seguridad a paredes

CARPETA_SALIDA = Path.home() / "ros2_ws"



# 1. LEER EL MAPA OBJ


print("\nLeyendo laberinto 3D...")

vertices = []
caras = []

with open(ARCHIVO, "r", errors="replace") as archivo:
    for linea in archivo:
        partes = linea.strip().split()

        if not partes:
            continue

        if partes[0] == "v" and len(partes) >= 4:
            vertices.append(
                tuple(float(v) * ESCALA for v in partes[1:4])
            )

        elif partes[0] == "f":
            indices = []

            for elemento in partes[1:]:
                indice = int(elemento.split("/")[0])
                if indice < 0:
                    indice = len(vertices) + indice
                else:
                    indice -= 1

                indices.append(indice)

            for k in range(1, len(indices) - 1):
                caras.append((
                    indices[0],
                    indices[k],
                    indices[k + 1]
                ))

print(f"Vertices: {len(vertices)}")
print(f"Triangulos: {len(caras)}")



# 2. EXTRAER LAS PAREDES VERTICALES


segmentos = []

for a, b, c in caras:
    puntos = [vertices[a], vertices[b], vertices[c]]

    alturas = [p[2] for p in puntos]

    # Descartar superficies horizontales
    if max(alturas) - min(alturas) < ALTURA_MINIMA:
        continue

    # Proyectar triangulo sobre XY
    puntos_xy = [(p[0], p[1]) for p in puntos]

    # Tomar el segmento mas largo de la proyeccion
    pares = [(0, 1), (1, 2), (2, 0)]

    i, j = max(
        pares,
        key=lambda par: math.dist(
            puntos_xy[par[0]],
            puntos_xy[par[1]]
        )
    )

    if math.dist(puntos_xy[i], puntos_xy[j]) > 0.001:
        segmentos.append((puntos_xy[i], puntos_xy[j]))

if not segmentos:
    raise RuntimeError("No se detectaron paredes verticales en el OBJ.")

print(f"Segmentos de paredes: {len(segmentos)}")



# 3. CREAR CUADRICULA DE OCUPACION


puntos = [p for segmento in segmentos for p in segmento]

xmin = min(p[0] for p in puntos) - RESOLUCION
xmax = max(p[0] for p in puntos) + RESOLUCION

ymin = min(p[1] for p in puntos) - RESOLUCION
ymax = max(p[1] for p in puntos) + RESOLUCION

columnas = math.ceil((xmax - xmin) / RESOLUCION)
filas = math.ceil((ymax - ymin) / RESOLUCION)

ocupado = np.zeros((filas, columnas), dtype=bool)


def coordenada_a_celda(x, y):
    columna = int((x - xmin) / RESOLUCION)
    fila = int((y - ymin) / RESOLUCION)
    return fila, columna


def celda_a_coordenada(fila, columna):
    x = xmin + (columna + 0.5) * RESOLUCION
    y = ymin + (fila + 0.5) * RESOLUCION
    return x, y


# Dibujar paredes sobre la cuadricula
for p1, p2 in segmentos:
    longitud = math.dist(p1, p2)
    pasos = max(1, math.ceil(longitud / (RESOLUCION / 2)))

    for k in range(pasos + 1):
        t = k / pasos

        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])

        fila, columna = coordenada_a_celda(x, y)

        if 0 <= fila < filas and 0 <= columna < columnas:
            ocupado[fila, columna] = True



# 4. APLICAR MARGEN DE SEGURIDAD


radio_celdas = math.ceil(MARGEN_BUGGY / RESOLUCION)

relleno = np.pad(
    ocupado,
    radio_celdas,
    mode="constant",
    constant_values=False
)

mapa = ocupado.copy()

for dy in range(-radio_celdas, radio_celdas + 1):
    for dx in range(-radio_celdas, radio_celdas + 1):

        distancia = math.hypot(dx, dy) * RESOLUCION

        if distancia > MARGEN_BUGGY:
            continue

        mapa |= relleno[
            radio_celdas + dy:radio_celdas + dy + filas,
            radio_celdas + dx:radio_celdas + dx + columnas
        ]


print(f"\nCuadricula creada: {columnas} x {filas}")
print(f"Resolucion: {RESOLUCION} m")
print("0 = Libre")
print("1 = Obstaculo")



# 5. SELECCIONAR INICIO Y META


fig, ax = plt.subplots(figsize=(11, 8))

extension = (
    xmin,
    xmin + columnas * RESOLUCION,
    ymin,
    ymin + filas * RESOLUCION
)

ax.imshow(
    mapa.astype(int),
    origin="lower",
    extent=extension,
    cmap="gray_r",
    interpolation="nearest"
)

ax.set_title("BFS - Clic 1: INICIO | Clic 2: META")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_aspect("equal")
ax.grid(alpha=0.2)

plt.tight_layout()

print("\nSelecciona DOS puntos en la ventana:")
print("1. INICIO del buggy")
print("2. SALIDA del laberinto")

seleccion = plt.ginput(2, timeout=-1)

if len(seleccion) != 2:
    raise SystemExit("No se seleccionaron los dos puntos.")

inicio = coordenada_a_celda(*seleccion[0])
meta = coordenada_a_celda(*seleccion[1])


def es_libre(celda):
    fila, columna = celda

    return (
        0 <= fila < filas
        and 0 <= columna < columnas
        and not mapa[fila, columna]
    )


if not es_libre(inicio):
    raise SystemExit("El INICIO esta dentro de una zona bloqueada.")

if not es_libre(meta):
    raise SystemExit("La META esta dentro de una zona bloqueada.")



# 6. ALGORITMO BFS


def bfs(inicio, meta):

    cola = deque([inicio])
    anteriores = {inicio: None}

    movimientos = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1)
    ]

    while cola:
        actual = cola.popleft()

        if actual == meta:
            break

        fila, columna = actual

        for df, dc in movimientos:
            vecino = (fila + df, columna + dc)

            if vecino not in anteriores and es_libre(vecino):
                anteriores[vecino] = actual
                cola.append(vecino)

    if meta not in anteriores:
        return None

    camino = []
    actual = meta

    while actual is not None:
        camino.append(actual)
        actual = anteriores[actual]

    camino.reverse()

    return camino


ruta = bfs(inicio, meta)

if ruta is None:
    print("\nBFS: NO SE ENCONTRO UNA RUTA.")
    print("Revisa inicio, meta o el margen de seguridad.")
    plt.show()
    raise SystemExit(1)



# 7. MOSTRAR Y GUARDAR EL RECORRIDO


coordenadas = [celda_a_coordenada(*celda) for celda in ruta]

xs = [p[0] for p in coordenadas]
ys = [p[1] for p in coordenadas]

ax.plot(xs, ys, "b-", linewidth=2, label="Ruta BFS")

ax.plot(
    seleccion[0][0],
    seleccion[0][1],
    "go",
    markersize=10,
    label="Inicio"
)

ax.plot(
    seleccion[1][0],
    seleccion[1][1],
    "ro",
    markersize=10,
    label="Meta"
)

ax.legend()
ax.set_title("Laberinto resuelto mediante BFS")

imagen = CARPETA_SALIDA / "mapa_bfs.png"
datos = CARPETA_SALIDA / "ruta_bfs.csv"

fig.savefig(imagen, dpi=180)

with open(datos, "w", newline="") as archivo:
    escritor = csv.writer(archivo)
    escritor.writerow(["x", "y"])
    escritor.writerows(coordenadas)

print("\n========== RESULTADO BFS ==========")
print(f"Celdas recorridas: {len(ruta)}")
print(f"Longitud aproximada: {(len(ruta)-1)*RESOLUCION:.2f} m")
print(f"Mapa guardado: {imagen}")
print(f"Ruta guardada: {datos}")
print("===================================")

plt.show()
