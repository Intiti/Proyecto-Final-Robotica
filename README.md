<img width="700" height="600" alt="image" src="https://github.com/user-attachments/assets/e4d0c57e-8b1f-4f77-8431-f2314c355220" /># Navegación Autónoma con Planificación de Rutas en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150  
**Línea:** A — Planificación de rutas con A*

**Integrantes:**
- Hans Silva
- Inti Liberona
- Kevin Alvarez
- Renato Mujica
- Zhiheng Lei

---

## Objetivo

Implementar un sistema de navegación autónoma para un robot e-puck en Webots, capaz de planificar una ruta desde una posición inicial hasta una meta usando el algoritmo A* sobre una grilla de ocupación 2D, con evitación reactiva de obstáculos en tiempo real.

---

## Robot, sensores y actuadores

- **Robot:** e-puck (diferencial)
- **Motores:** `left wheel motor` / `right wheel motor`
- **Encoders:** `left wheel sensor` / `right wheel sensor`
- **Sensores de distancia:** `ps0` (frontal der), `ps2` (lateral der), `ps5` (lateral izq), `ps7` (frontal izq)
- **Radio de ruedas:** 0.0205 m
- **Distancia entre ejes:** 0.052 m
- **Velocidad máxima:** 6.28 rad/s

---

## Escenarios de prueba

### Escenario simple
- Arena de **2 m × 2 m** discretizada en una grilla **20×20** (celdas de 0.1 m)
- Pocos obstáculos distribuidos, ruta con baja complejidad
- Inicio: `(-0.95, 0.95)` → Meta: `(0.95, -0.95)` (esquina superior izquierda a inferior derecha)

### Escenario complejo
> *(en desarrollo — mayor densidad de obstáculos, pasillos estrechos)*

---

## Algoritmo implementado

### Planificación — A*

La grilla de ocupación (`0` = libre, `1` = obstáculo) representa el entorno. A* calcula la ruta óptima desde el nodo inicio al nodo meta usando distancia euclidiana como heurística. Permite movimientos en 8 direcciones (incluyendo diagonales).

**Conversión nodo → metros:**
```
xMetros = (col × 0.1) - 1.0 + 0.05
yMetros = 1.0 - (fila × 0.1) - 0.05
```

### Seguimiento de waypoints
<img width="700" height="600" alt="image" src="https://github.com/user-attachments/assets/639103cf-5e97-42c6-8058-ded57272ec54" />

El robot avanza por cada waypoint de la ruta planificada usando control proporcional:

1. Calcula error angular entre orientación actual y dirección al waypoint
2. Si `|error| > 0.35 rad`: gira en el lugar (control puro de orientación)
3. Si `|error| ≤ 0.35 rad`: avanza con corrección angular suave
4. Waypoint alcanzado cuando `distancia < umbral` (0.075 m intermedios, 0.04 m meta final)

### Evitación reactiva

Si un sensor frontal supera el umbral de **140 unidades**, el robot entra en modo de escape:
- Determina la dirección libre comparando sensores laterales
- Gira en el lugar durante al menos 12 pasos de control
- Retoma la ruta planificada al despejarse la vía

---

## Diagrama de flujo

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
  ├─ ¿Sensor frontal > 140? ─Sí─→ Modo escape reactivo (giro)
  │                                       │
  │                                       └─ ¿Despejado y pasos ≥ 12? → volver a planificado
  │
  ├─ Calcular error de posición y angular al waypoint actual
  ├─ Aplicar control proporcional a motores
  ├─ ¿Distancia < umbral? → avanzar al siguiente waypoint
  │
  └─ ¿Todos los waypoints completados? → detener y reportar
```

---

## Relación con laboratorios

### Laboratorio 1 — Control cinemático diferencial
Se reutiliza el modelo diferencial para convertir velocidades lineales y angulares en velocidades de rueda. La función de seguimiento de waypoints aplica directamente las ecuaciones:

```
v = (vr + vl) / 2
ω = (vr - vl) / L
```

### Laboratorio 2 — Percepción y estimación
- **Encoders:** estiman posición y orientación en cada paso via odometría
- **Filtro de mediana:** suaviza lecturas ruidosas de los sensores de proximidad antes de usarlas para detectar obstáculos

**Odometría (ecuaciones del proyecto):**
```
Δs  = (Δsr + Δsl) / 2
Δφ  = (Δsr - Δsl) / L
x_k = x_{k-1} + Δs · cos(φ_{k-1} + Δφ/2)
y_k = y_{k-1} + Δs · sin(φ_{k-1} + Δφ/2)
φ_k = φ_{k-1} + Δφ
```

---

## Resultados y métricas

| Métrica | Escenario simple |
|---|---|
| Nodo inicio | `[0, 0]` |
| Nodo meta | `[19, 19]` |
| Celdas en la ruta A* | — |
| Tiempo hasta la meta | — |
| Distancia odométrica acumulada | — |
| Intervenciones reactivas | — |
| Colisiones | — |

> Los valores se completan con los datos registrados en consola durante la simulación.

<img width="800" height="400" alt="image" src="https://github.com/user-attachments/assets/196573d0-9c94-48e5-80b2-e29457f598cc" />

---

## Cómo ejecutar

1. Abrir **Webots**
2. Cargar el mundo: `worlds/pistaSimple.wbt`
3. El controlador asignado al robot es `controllers/controllerSimple/controllerSimple.py`
4. Ejecutar la simulación — la consola muestra:
   - Ruta calculada por A* (cantidad de celdas)
   - Estado de navegación cada 50 pasos
   - Reporte final al llegar a la meta

**Requisitos:** Webots instalado con soporte para Python y el módulo `controller` disponible.

---

## Conclusiones

- El algoritmo A* sobre grilla 20×20 calcula rutas eficientes para el escenario simple
- El filtro de mediana reduce el ruido en los sensores de proximidad, evitando falsas detecciones
- La evitación reactiva complementa la planificación global ante obstáculos no modelados
- El error odométrico se acumula con la distancia recorrida; en recorridos largos puede desviarse del waypoint

**Limitaciones:**
- La grilla de ocupación es estática; no se actualiza con nuevas lecturas de sensores en tiempo real
- La evitación reactiva puede desorientar al robot si el obstáculo persiste varios pasos consecutivos

**Mejoras posibles:**
- Actualizar dinámicamente la grilla con lecturas de sensores y replanificar
- Incorporar filtro de Kalman para reducir el error acumulado de odometría
- Añadir un segundo escenario con mayor complejidad y comparar métricas
