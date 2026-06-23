from controller import Robot
import math

mapa_facil = [
    [0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

mapa_laberinto = [
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
    [1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0],
    [1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0],
    [1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0],
    [0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0],
    [0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
]

ESCENARIOS = {
    "facil": {"mapa": mapa_facil, "inicio": (0, 0), "meta": (15, 15)},
    "laberinto": {"mapa": mapa_laberinto, "inicio": (0, 0), "meta": (15, 3)},
}
ESCENARIO = "laberinto"

mapa   = ESCENARIOS[ESCENARIO]["mapa"]
inicio = ESCENARIOS[ESCENARIO]["inicio"]
meta   = ESCENARIOS[ESCENARIO]["meta"]

robot = Robot()
timestep = int(robot.getBasicTimeStep())
motor_izq = robot.getDevice("left wheel motor")
motor_der = robot.getDevice("right wheel motor")
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)
v_max = 6.28
enc_izq = robot.getDevice("left wheel sensor")
enc_der = robot.getDevice("right wheel sensor")
enc_izq.enable(timestep)
enc_der.enable(timestep)
sensor_izq = robot.getDevice("ps5")
sensor_izq.enable(timestep)
sensor_front_izq = robot.getDevice("ps7")
sensor_front_izq.enable(timestep)
sensor_front_der = robot.getDevice("ps0")
sensor_front_der.enable(timestep)
sensor_der = robot.getDevice("ps2")
sensor_der.enable(timestep)
radio_ruedas: float = 0.0205
dist_ruedas: float = 0.058

def distancia_manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def aestrella(mapa: list, inicio, meta):
    filas: int = len(mapa)
    columnas: int = len(mapa[0])
    abierto: list = [inicio]
    ruta: dict = {}
    g: dict = {inicio: 0}
    f: dict = {inicio: distancia_manhattan(inicio, meta)}

    while abierto:
        actual = abierto[0]
        for celda in abierto:
            if f[celda] < f[actual]:
                actual = celda
        abierto.remove(actual)

        if actual == meta:
            camino = []
            while actual in ruta:
                camino.insert(0, actual)
                actual = ruta[actual]
            camino.insert(0, inicio)
            return camino

        fa, ca = actual
        for df, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            vecino = (fa + df, ca + dc)
            vf, vc = vecino

            if not (0 <= vf < filas and 0 <= vc < columnas):
                continue
            if mapa[vf][vc] == 1:
                continue

            g_nuevo = g[actual] + 1
            if vecino not in g or g_nuevo < g[vecino]:
                g[vecino] = g_nuevo
                ruta[vecino] = actual
                f[vecino] = g_nuevo + distancia_manhattan(vecino, meta)
                if vecino not in abierto:
                    abierto.append(vecino)

    return None

camino: list = aestrella(mapa, inicio, meta)
tamano_celda: float = 0.125
mitad_arena: float = 1.0

def celda_a_mundo(celda):
    fila, col = celda
    x = -mitad_arena + (col  + 0.5) * tamano_celda
    y =  mitad_arena - (fila + 0.5) * tamano_celda
    return (x, y)

def mundo_a_celda(x, y):
    col = int(math.floor((x + mitad_arena) / tamano_celda))
    fila = int(math.floor((mitad_arena - y) / tamano_celda))
    col = max(0, min(len(mapa[0]) - 1, col))
    fila = max(0, min(len(mapa) - 1, fila))
    return (fila, col)

def imprimir_mapa(x, y):
    fila_rob, col_rob = mundo_a_celda(x, y)
    print("\n" * 2)
    print("===== NAVEGACIÓN DEL ROBOT =====")

    for f in range(len(mapa)):
        linea = ""
        for c in range(len(mapa[0])):
            if f == fila_rob and c == col_rob:
                linea += "[R]"
            elif (f, c) == meta:
                linea += "[M]"
            elif mapa[f][c] == 1:
                linea += "███"
            else:
                linea += " . "
        print(linea)
    print("================================")
# -------------------------------------

if camino is None:
    print("No hay ruta")
    waypoints = []
else:
    waypoints = [celda_a_mundo(c) for c in camino]

pos = {
    "x": waypoints[0][0],
    "y": waypoints[0][1],
    "angulo": 0.0,
    "izq": 0.0,
    "der": 0.0,
}

def normalizar_angulo(a):
    while a >  math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a

def actualizar_odometria(pos, enc_izq, enc_der):
    delta_izq = enc_izq - pos["izq"]
    delta_der = enc_der - pos["der"]
    pos["izq"], pos["der"] = enc_izq, enc_der

    avance_rueda_izq = delta_izq * radio_ruedas
    avance_rueda_der = delta_der * radio_ruedas
    avance = (avance_rueda_izq + avance_rueda_der) / 2.0
    delta_theta = (avance_rueda_der - avance_rueda_izq) / dist_ruedas

    pos["angulo"] += delta_theta
    pos["x"] += avance * math.cos(pos["angulo"])
    pos["y"] += avance * math.sin(pos["angulo"])

indice_wp = 1
vel = 2.0
vel_giro = 1.0
tolerancia_dist = 0.03
tolerancia_angulo = 0.05
umbral_sensor = 80.0
kp_centrado = 0.005
umbral_pared = 70.0
lectura_centrado = 130.0
corr_max = vel * 0.6

while robot.step(timestep) != -1:
    actualizar_odometria(pos, enc_izq.getValue(), enc_der.getValue())
    imprimir_mapa(pos["x"], pos["y"])
    frente = max(sensor_front_der.getValue(), sensor_front_izq.getValue())

    if indice_wp < len(waypoints):
        objetivo = waypoints[indice_wp]
        dist_x = objetivo[0] - pos["x"]
        dist_y = objetivo[1] - pos["y"]
        distancia = math.hypot(dist_x, dist_y)
        angulo_deseado = math.atan2(dist_y, dist_x)
        print(f"angulo deseado {angulo_deseado}")
        error_ang = normalizar_angulo(angulo_deseado - pos["angulo"])

        if distancia < tolerancia_dist:
            indice_wp += 1

        elif abs(error_ang) > tolerancia_angulo:
            if error_ang > 0:
                motor_izq.setVelocity(-vel_giro)
                motor_der.setVelocity(+vel_giro)
            else:
                motor_izq.setVelocity(+vel_giro)
                motor_der.setVelocity(-vel_giro)

        else:
            if frente > umbral_sensor:
                motor_izq.setVelocity(0.0)
                motor_der.setVelocity(0.0)
            else:
                lat_izq = sensor_izq.getValue()
                lat_der = sensor_der.getValue()
                izq_cerca = lat_izq > umbral_pared
                der_cerca = lat_der > umbral_pared
                correccion = 0.0
                if izq_cerca and der_cerca:
                    correccion = kp_centrado * (lat_izq - lat_der)
                elif izq_cerca:
                    correccion = kp_centrado * (lat_izq - lectura_centrado)
                elif der_cerca:
                    correccion = -kp_centrado * (lat_der - lectura_centrado)
                correccion = max(min(correccion, corr_max), -corr_max)
                motor_izq.setVelocity(vel + correccion)
                motor_der.setVelocity(vel - correccion)
    else:
        motor_izq.setVelocity(0.0)
        motor_der.setVelocity(0.0)
        print("¡Meta alcanzada!")
        break

    print(f"x={pos['x']:.3f} y={pos['y']:.3f} "
          f"angulo={pos['angulo']:.3f} rad ({math.degrees(pos['angulo']):.1f} grados) "
          f"frente={frente:.1f}")
