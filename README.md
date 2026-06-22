# Navegación Autónoma con Planificación de Rutas en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150  
**Línea:** A — Planificación de rutas con A*

**Integrantes:** Hans Silva · Inti Liberona · Kevin Alvarez · Renato Mujica · Zhiheng Lei

---

## Objetivo

Implementar un sistema de navegación autónoma para un robot e-puck en Webots, capaz de planificar una ruta desde una posición inicial hasta una meta usando el algoritmo **A*** sobre una grilla de ocupación 2D, con evitación reactiva de obstáculos en tiempo real.

---

## Robot, Sensores y Actuadores

| Componente | Detalle |
|---|---|
| Robot | e-puck (diferencial) |
| Motores | `left wheel motor` / `right wheel motor` |
| Encoders | `left wheel sensor` / `right wheel sensor` |
| Sensores de distancia | `ps0` (frontal der), `ps2` (lateral der), `ps5` (lateral izq), `ps7` (frontal izq) |
| Radio de ruedas | 0.0205 m |
| Distancia entre ejes | 0.052 m |
| Velocidad máxima | 6.28 rad/s |

---

## Escenario de Prueba

- Arena de **2 m × 2 m** discretizada en grilla de ocupación completa.
- Pocos obstáculos distribuidos, ruta de baja complejidad.
- **Inicio:** `(-0.95, 0.95)` → **Meta:** `(0.775, -0.925)` *(esquina superior izquierda a inferior derecha)*

---

## Algoritmo Implementado

### Planificación — A*

La grilla de ocupación (`0` = libre, `1` = obstáculo) representa el entorno. A* calcula la ruta óptima desde el nodo inicio al nodo meta usando **distancia euclidiana** como heurística, con movimientos en **8 direcciones** (incluyendo diagonales).

**Conversión nodo → metros:**

```python
def transformarNodoAMetros(nodo):
    tamanoCelda = 0.05
    offsetArena = 1.0
    xMetros = (nodo[1] * tamanoCelda) - offsetArena + (tamanoCelda / 2.0)
    yMetros = offsetArena - (nodo[0] * tamanoCelda) - (tamanoCelda / 2.0)
    return xMetros, yMetros
```

### Seguimiento de Waypoints

El robot avanza por cada waypoint usando **control proporcional**:

- Calcula el error angular entre orientación actual y dirección al waypoint.
- Si `abs(errorAngular) > 0.35`: gira en el lugar (control puro de orientación).
- Si `abs(errorAngular) <= 0.35`: avanza con corrección angular suave.
- Waypoint alcanzado cuando `distanciaAlTarget < umbralLlegada` (0.05 m intermedios, 0.03 m meta final).

### Evitación Reactiva

Si un sensor frontal supera el umbral de **150 unidades**, el robot entra en modo de escape:

1. Determina la dirección libre comparando sensores laterales.
2. Gira en el lugar mediante control reactivo.
3. Retoma la ruta planificada al despejarse la vía.

> Si se detecta un bloqueo cíclico recurrente en el mismo waypoint (`conteoEscapesEnWp >= 3`), se saltea dicho punto para evitar bucles infinitos.

### Diagrama de Flujo

```
Inicio
  │
  ▼
Cargar grilla de ocupación
  │
  ▼
Ejecutar A*(inicio, meta) → lista de waypoints
  │
  ├─ Sin ruta → detener
  │
  ▼
Loop de control (cada timestep)
  │
  ├─ Leer encoders → actualizar odometría (x, y, theta)
  ├─ Leer sensores → filtro de mediana (ventana 5)
  │
  ├─ ¿Sensor frontal > 150? ─Sí─→ Modo escape reactivo (giro)
  │                                      │
  │                                      └─ ¿Despejado y pasos completados? → volver a planificado
  │
  ├─ Calcular error de posición y angular al waypoint actual
  ├─ Aplicar control proporcional a motores con saturación física
  ├─ ¿Distancia < umbral? → avanzar al siguiente waypoint o saltear si está bloqueado
  │
  └─ ¿Todos los waypoints completados? → detener y reportar
```

---

## Relación con Laboratorios

### Laboratorio 1 — Control Cinemático Diferencial

Se reutiliza el modelo diferencial para convertir velocidades lineales y angulares en velocidades de rueda:

```python
vIzq = vLinea - omegaAjuste * (distanciaEjes / 2.0)
vDer = vLinea + omegaAjuste * (distanciaEjes / 2.0)
vIzq = max(min(vIzq, velocidadMax), -velocidadMax)
vDer = max(min(vDer, velocidadMax), -velocidadMax)
```

### Laboratorio 2 — Percepción y Estimación

- **Encoders:** Estiman posición y orientación en cada paso de simulación mediante odometría directa.
- **Filtro de mediana:** Suaviza lecturas ruidosas de sensores de proximidad antes de usarlas en el lazo cerrado.

**Odometría (ecuaciones del proyecto):**

```python
def calcularAvanceRuedas(pos, posActualIzq, posActualDer):
    deltaIzq = posActualIzq - pos["encoderIzqPrevio"]
    deltaDer = posActualDer - pos["encoderDerPrevio"]
    avanceRuedaIzq = deltaIzq * radioRuedas
    avanceRuedaDer = deltaDer * radioRuedas
    avance = (avanceRuedaIzq + avanceRuedaDer) / 2
    deltaTheta = (avanceRuedaDer - avanceRuedaIzq) / distanciaEjes
    thetaMedio = pos["theta"] + deltaTheta / 2.0
    pos["x"] += avance * math.cos(thetaMedio)
    pos["y"] += avance * math.sin(thetaMedio)
    pos["theta"] += deltaTheta
    pos["actual"] += abs(avance)
    return avance
```

---

## Resultados y Métricas

| Métrica | Valor |
|---|---|
| Nodo inicio | [0, 0] |
| Nodo meta | [38, 35] |
| Celdas en la ruta A* | 122 celdas |
| Pasos totales de control | 19,000 pasos |
| Tiempo de simulación estimado | 608.0 s (19 000 pasos × 0.032 s) |
| Posición final (Odometría) | X: 0.8010 m · Y: -0.9197 m |
| Posición teórica esperada | X: 0.7750 m · Y: -0.9250 m |
| Distancia odométrica acumulada | 6.5935 m |
| Intervenciones reactivas locales | 6 eventos |

---

## Análisis Técnico del Comportamiento

### Comportamiento del Lazo de Control

Durante la ejecución, el robot alternó dinámicamente entre seguimiento de trayectoria y maniobras reactivas:

- **Conducción estable:** Entre las iteraciones iniciales y el paso 14 000, el robot mantuvo una progresión continua a través de los waypoints. El error de distancia convergió de manera constante, validando el controlador proporcional.
- **Saturación local e intervención anti-bloqueo:** Entre los pasos 14 500 y 14 950, el robot experimentó una condición de atrapamiento cerca del waypoint 85. El sistema activó la lógica de escape y saltó al waypoint 86, recuperando la autonomía de navegación.

### Error Odométrico y Deriva

Comparando la posición final estimada ($X: 0.8010,\ Y: -0.9197$) con el centro físico del nodo meta ($X: 0.7750,\ Y: -0.9250$):

$$E_{pos} = \sqrt{(0.8010 - 0.7750)^2 + (-0.9197 - (-0.9250))^2} \approx 0.0265\ \text{m}$$

Un error de **2.65 cm** tras recorrer más de 6.5 metros demuestra que la integración del modelo cinemático diferencial a 32 ms mantiene niveles estables de precisión para trayectorias complejas de mediana escala.

---

## Cómo Ejecutar

1. Abrir **Webots**.
2. Cargar el mundo: `worlds/pistaSimple.wbt`.
3. Validar que el controlador asignado al e-puck sea `controllers/controllerSimple/controllerSimple.py`.
4. Ejecutar la simulación. La consola imprimirá:
   - Ruta completa calculada por A* con los valores de grilla.
   - Estado de la navegación e índice del waypoint activo cada 50 pasos.
   - Reporte de métricas físicas al cumplir la meta global.

**Requisitos:** Webots con entorno Python 3 y la librería `controller` vinculada.

---

## Conclusiones

- El algoritmo A* sobre grilla discretizada (celdas de 0.05 m) genera trayectorias viables y óptimas en entornos con pasillos definidos.
- El filtro de mediana de ventana fija (largo 5) remueve el ruido impulsivo de los sensores IR sin comprometer el tiempo de respuesta de la evitación.
- El mecanismo anti-bloqueo ante waypoints cíclicos dota al robot de robustez frente a limitaciones de mapas estáticos.

### Limitaciones

- La grilla de ocupación **estática** no se reconfigura frente a obstáculos imprevistos, delegando la evasión al control reactivo.
- El retraso del filtro de mediana introduce una pequeña latencia que limita la velocidad lineal máxima segura.

### Mejoras Posibles

- Implementar **replanificación local dinámica** para recalcular la ruta global ante sensores activados.
- Incorporar un **Filtro de Kalman** para corregir la deriva geométrica inherente a la odometría pura.
