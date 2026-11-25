import subprocess

# Topicos de control del buggy
LB = "/model/buggy/joint/LB/cmd_vel"
RB = "/model/buggy/joint/RB/cmd_vel"
DIRECCION = "/model/buggy/joint/tubochasis/0/cmd_pos"

# Valores que ya probamos en Gazebo
VELOCIDAD = 1.0
ANGULO = 0.20


def enviar(topico, valor):
    subprocess.run([
        "gz", "topic",
        "-t", topico,
        "-m", "gz.msgs.Double",
        "-p", f"data: {valor}"
    ], check=True)


def mover(velocidad):
    # Los ejes traseros tienen sentidos opuestos
    enviar(LB, -velocidad)
    enviar(RB, velocidad)


def girar(angulo):
    # Cambiar la posicion del manubrio
    enviar(DIRECCION, angulo)


print("\n===== CONTROL DEL BUGGY =====")
print("W = Avanzar")
print("S = Retroceder")
print("A = Girar hacia un lado")
print("D = Girar hacia el otro lado")
print("C = Centrar direccion")
print("X = Detener")
print("Q = Salir")
print("============================")

try:
    mover(0.0)
    girar(0.0)

    while True:
        comando = input("\nComando: ").lower().strip()

        if comando == "w":
            mover(VELOCIDAD)
            print("Avanzando")

        elif comando == "s":
            mover(-VELOCIDAD)
            print("Retrocediendo")

        elif comando == "a":
            girar(ANGULO)
            print("Girando direccion +0.20")

        elif comando == "d":
            girar(-ANGULO)
            print("Girando direccion -0.20")

        elif comando == "c":
            girar(0.0)
            print("Direccion centrada")

        elif comando == "x":
            mover(0.0)
            print("Detenido")

        elif comando == "q":
            break

        else:
            print("Comando no reconocido")

except KeyboardInterrupt:
    print("\nControl interrumpido")

finally:
    mover(0.0)
    girar(0.0)
    print("Buggy detenido")
