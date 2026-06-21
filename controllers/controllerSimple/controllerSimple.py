from controller import Robot
import math

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

motorRuedaIzq = robot.getDevice("left wheel motor")
motorRuedaIzq.setPosition(float("inf"))
motorRuedaIzq.setVelocity(0.0)

motorRuedaDer = robot.getDevice("right wheel motor")
motorRuedaDer.setPosition(float("inf"))
motorRuedaDer.setVelocity(0.0)

encoderRuedaIzq = robot.getDevice("left wheel sensor")
encoderRuedaIzq.enable(timeStep)
encoderRuedaDer = robot.getDevice("right wheel sensor")
encoderRuedaDer.enable(timeStep)

distFrontIzq = robot.getDevice("ps7")
distFrontIzq.enable(timeStep)
distFrontDer = robot.getDevice("ps0")
distFrontDer.enable(timeStep)
distLatIzq = robot.getDevice("ps5")
distLatIzq.enable(timeStep)
distLatDer = robot.getDevice("ps2")
distLatDer.enable(timeStep)

radioRuedas = 0.0205
distanciaEjes = 0.052
velocidadMax = 6.28

pos = {
    "theta": 0.0,
    "actual": 0.0,
    "x": -0.95,
    "y": 0.95,
    "encoderIzqPrevio": 0.0,
    "encoderDerPrevio": 0.0,
}

grillaOcupacion = [
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
]

historialFrontIzq = []
historialFrontDer = []
largoFiltro = 5

def calcularAvanceRuedas(pos, posActualIzq, posActualDer):
    deltaIzq = posActualIzq - pos["encoderIzqPrevio"]
    deltaDer = posActualDer - pos["encoderDerPrevio"]
    avanceRuedaIzq = deltaIzq * radioRuedas
    avanceRuedaDer = deltaDer * radioRuedas
    avance = (avanceRuedaIzq + avanceRuedaDer) / 2
    deltaTheta = (avanceRuedaDer - avanceRuedaIzq) / distanciaEjes
    pos["actual"] += avance
    pos["theta"] += deltaTheta
    pos["x"] += avance * math.cos(pos["theta"])
    pos["y"] += avance * math.sin(pos["theta"])
    return avance

def transformarNodoAMetros(nodo):
    tamanoCelda = 0.1
    offsetArena = 1.0
    xMetros = (nodo[1] * tamanoCelda) - offsetArena + (tamanoCelda / 2.0)
    yMetros = offsetArena - (nodo[0] * tamanoCelda) - (tamanoCelda / 2.0)
    return xMetros, yMetros

def aplicarFiltroMediana(historial, nuevaLectura):
    historial.append(nuevaLectura)
    if len(historial) > largoFiltro:
        historial.pop(0)
    historialOrdenado = sorted(historial)
    n = len(historialOrdenado)
    mitad = n // 2
    if n % 2 == 1:
        return historialOrdenado[mitad]
    else:
        return (historialOrdenado[mitad - 1] + historialOrdenado[mitad]) / 2

def algoritmoAStar(grilla, inicio, meta):
    filas = len(grilla)
    columnas = len(grilla[0])
    openSet = [inicio]
    cameFrom = {}
    gScore = {tuple(inicio): 0}
    fScore = {tuple(inicio): math.sqrt((meta[0]-inicio[0])**2 + (meta[1]-inicio[1])**2)}

    while openSet:
        actual = min(openSet, key=lambda n: fScore.get(tuple(n), float("inf")))
        if actual == meta:
            ruta = []
            while tuple(actual) in cameFrom:
                ruta.append(actual)
                actual = cameFrom[tuple(actual)]
            ruta.append(inicio)
            ruta.reverse()
            return ruta
        openSet.remove(actual)
        for deltaFila, deltaCol in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            vecino = [actual[0] + deltaFila, actual[1] + deltaCol]
            if 0 <= vecino[0] < filas and 0 <= vecino[1] < columnas:
                if grilla[vecino[0]][vecino[1]] == 1:
                    continue
                pesoMovimiento = math.sqrt(deltaFila**2 + deltaCol**2)
                tentativeGScore = gScore[tuple(actual)] + pesoMovimiento
                if tentativeGScore < gScore.get(tuple(vecino), float("inf")):
                    cameFrom[tuple(vecino)] = actual
                    gScore[tuple(vecino)] = tentativeGScore
                    fScore[tuple(vecino)] = tentativeGScore + math.sqrt((meta[0]-vecino[0])**2 + (meta[1]-vecino[1])**2)
                    if vecino not in openSet:
                        openSet.append(vecino)
    return []

nodoInicio = [0, 0]
nodoMeta = [19, 19]

print(f"[ASTAR] Calculando ruta de {nodoInicio} a {nodoMeta}...")
caminoCeldas = algoritmoAStar(grillaOcupacion, nodoInicio, nodoMeta)

if not caminoCeldas:
    print("[ASTAR] ERROR: no se encontro ruta!")
else:
    print(f"[ASTAR] Ruta encontrada: {len(caminoCeldas)} celdas")

listaWaypoints = [transformarNodoAMetros(n) for n in caminoCeldas]

indiceWaypointActual = 0
pasoSimulacion = 0
LOG_CADA_N_PASOS = 50
metaAlcanzadaInforme = False
conteoColisionesEvitadas = 0

estadoEscapeReactivo = False
conteoPasosEscape = 0
pasosMinimosEscape = 12
direccionGiroEscape = 1

while robot.step(timeStep) != -1:
    posEncoderIzq = encoderRuedaIzq.getValue()
    posEncoderDer = encoderRuedaDer.getValue()
    calcularAvanceRuedas(pos, posEncoderIzq, posEncoderDer)
    pos["encoderIzqPrevio"] = posEncoderIzq
    pos["encoderDerPrevio"] = posEncoderDer

    pasoSimulacion += 1

    lecturaIzqRaw = distFrontIzq.getValue()
    lecturaDerRaw = distFrontDer.getValue()
    lecturaIzqFiltrada = aplicarFiltroMediana(historialFrontIzq, lecturaIzqRaw)
    lecturaDerFiltrada = aplicarFiltroMediana(historialFrontDer, lecturaDerRaw)

    if indiceWaypointActual < len(listaWaypoints):
        if not estadoEscapeReactivo:
            if lecturaIzqFiltrada > 140.0 or lecturaDerFiltrada > 140.0:
                estadoEscapeReactivo = True
                conteoPasosEscape = 0
                conteoColisionesEvitadas += 1
                if distLatIzq.getValue() > distLatDer.getValue():
                    direccionGiroEscape = 1
                else:
                    direccionGiroEscape = -1

        if estadoEscapeReactivo:
            conteoPasosEscape += 1
            if direccionGiroEscape == 1:
                vIzq = velocidadMax * 0.4
                vDer = -velocidadMax * 0.4
            else:
                vIzq = -velocidadMax * 0.4
                vDer = velocidadMax * 0.4
            
            if lecturaIzqFiltrada < 80.0 and lecturaDerFiltrada < 80.0 and conteoPasosEscape >= pasosMinimosEscape:
                estadoEscapeReactivo = False
            
            motorRuedaIzq.setVelocity(vIzq)
            motorRuedaDer.setVelocity(vDer)
            continue

        targetX, targetY = listaWaypoints[indiceWaypointActual]
        errorX = targetX - pos["x"]
        errorY = targetY - pos["y"]
        distanciaAlTarget = math.sqrt(errorX**2 + errorY**2)

        if pasoSimulacion % LOG_CADA_N_PASOS == 0:
            print(f"[NAV] paso={pasoSimulacion:05d} | wp={indiceWaypointActual}/{len(listaWaypoints)-1} "
                  f"| robot=({pos['x']:.3f},{pos['y']:.3f}) "
                  f"| dist={distanciaAlTarget:.3f}m")

        umbralLlegada = 0.04 if indiceWaypointActual == (len(listaWaypoints) - 1) else 0.075
        if distanciaAlTarget < umbralLlegada:
            indiceWaypointActual += 1
            continue

        anguloDeseado = math.atan2(errorY, errorX)
        errorAngular = anguloDeseado - pos["theta"]
        errorAngular = math.atan2(math.sin(errorAngular), math.cos(errorAngular))

        if abs(errorAngular) > 0.35:
            kpRadianes = 4.0
            omega = kpRadianes * errorAngular
            vIzq = -omega * (distanciaEjes / 2.0)
            vDer = omega * (distanciaEjes / 2.0)
        else:
            vLinea = velocidadMax * 0.75
            kpAjusteAngular = 2.5
            omegaAjuste = kpAjusteAngular * errorAngular
            vIzq = vLinea - omegaAjuste * (distanciaEjes / 2.0)
            vDer = vLinea + omegaAjuste * (distanciaEjes / 2.0)

        vIzq = max(min(vIzq, velocidadMax), -velocidadMax)
        vDer = max(min(vDer, velocidadMax), -velocidadMax)
        motorRuedaIzq.setVelocity(vIzq)
        motorRuedaDer.setVelocity(vDer)
    else:
        motorRuedaIzq.setVelocity(0.0)
        motorRuedaDer.setVelocity(0.0)
        if not metaAlcanzadaInforme:
            print(f"\n=================================================")
            print(f"[NAV] META GLOBAL COMPLETADA EXITOSAMENTE")
            print(f"Pasos totales de control: {pasoSimulacion}")
            print(f"Posicion final del robot -> X: {pos['x']:.4f} | Y: {pos['y']:.4f}")
            print(f"Distancia acumulada odométrica: {pos['actual']:.4f} metros")
            print(f"Intervenciones reactivas locales: {conteoColisionesEvitadas}")
            print(f"=================================================\n")
            metaAlcanzadaInforme = True