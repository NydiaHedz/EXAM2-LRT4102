# Simulador de Seguimiento de Línea con Controlador PID usando TurtleSim

## 1. Importación de bibliotecas y definición de clase

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import matplotlib.pyplot as plt

class LineFollowerSimulatorPID(object):
    ...
````

* **math**: funciones matemáticas (trigonometría, conversiones grados↔radianes).
* **rospy**: interfaz ROS en Python.
* **Twist**: mensajes de velocidades lineales y angulares.
* **Pose**: mensajes de posición (x, y) y orientación (θ).
* **Spawn, Kill**: servicios de TurtleSim para crear y eliminar tortugas.
* **matplotlib.pyplot**: para graficar trayectorias y errores.

La clase principal `LineFollowerSimulatorPID` agrupa toda la lógica del simulador.

---

## 2. Callbacks y funciones auxiliares

### 2.1 Callback de posición de `turtle1`

```python
def _pose1_cb(self, msg):
    if hasattr(self, 'start_time1') and self.start_time1 is not None:
        t = rospy.Time.now().to_sec() - self.start_time1
        self.path1.append((t, msg.x, msg.y, msg.theta))
```

Registra tiempo relativo, posición `(x,y)` y orientación `θ` de la primera tortuga.

### 2.2 Callback de posición de `turtle2`

```python
def _pose2_cb(self, msg):
    self.pose2 = msg
```

Guarda la pose actual de la segunda tortuga.

### 2.3 Función `move`

```python
def move(self, linear_speed, angular_speed, duration, publisher):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start) < duration:
        publisher.publish(twist)
        rate.sleep()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    publisher.publish(twist)
```

Envía un comando de velocidad durante `duration` segundos y luego detiene la tortuga.

---

## 3. Generación de trayectoria (turtle1)

Dentro de `run()`:

1. Se espera y registra la pose inicial de `turtle1`.
2. Se avanza **4 s** en línea recta a velocidad `1.0`:

   ```python
   self.move(1.0, 0.0, 4.0, self.vel_pub1)
   ```
3. Se elimina `turtle1` usando el servicio `kill`.

---

## 4. Generación y seguimiento de trayectoria con la segunda tortuga

1. Se crea `turtle2` en la posición inicial de `turtle1`:

   ```python
   self.spawn_turtle(x0, y0, theta0, 'turtle2')
   ```
2. Suscripción a `/turtle2/pose` para leer su pose.
3. Parámetros del controlador **PID**:

   * `V_CONST = 1.0`
   * `K_P = 0.8`
   * `K_I = 0.02`
   * `K_D = 0.01`
   * `MAX_ANG_VEL = 30°/s`
   * `DIST_TH = 0.1`
4. Algoritmo de seguimiento:

   * Calcula distancia y error angular al **punto final** de la trayectoria.
   * Integra y deriva el error para obtener:

     ```python
     integral   += error * dt
     derivative  = (error - last_error) / dt
     omega       = K_P*error + K_I*integral + K_D*derivative
     omega       = clamp(omega, -MAX_ANG_VEL, MAX_ANG_VEL)
     ```
   * Publica `Twist(linear.x=V_CONST, angular.z=omega)`.
5. Condiciones de parada:

   * `dist < DIST_TH` (llegó al objetivo).
   * `pose2.x < 0.5 or >10.5` o `pose2.y <0.5 or >10.5` (cerca de la pared).

---

## 5. Gráficos finales de trayectorias y error angular

* **`plot_paths()`**: grafica la trayectoria de `turtle1` (línea recta).
* **`plot_angular_error()`**: muestra la evolución del error angular (°) vs tiempo.

Ambas incluyen etiquetas, leyenda y cuadrícula.

---

## 6. Ejecución principal

```python
if __name__ == '__main__':
    try:
        simulator = LineFollowerSimulatorPID()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
```

Inicializa y ejecuta el simulador, manejando interrupciones (Ctrl+C).

---

## Resumen del funcionamiento y equivalencia con señales reales de encoder

1. **Generación de trayectoria**
   `turtle1` almacena posiciones y orientaciones, simulando lecturas de encoder.

2. **Simulación del entorno**
   `turtle2` se inicia en la misma pose inicial y replica el seguimiento.

3. **Control PID de seguimiento**
   Ajusta las velocidades lineal y angular con bucle proporcional–integral–derivativo.

4. **Analogía con encoders**
   Waypoints de `turtle1` equivalen a pulsos de encoder en un robot real.

5. **Adaptación dinámica de velocidad**
   El término derivativo suaviza la corrección de rumbo ante cambios bruscos.

6. **Umbral de proximidad**
   `DIST_TH` actúa como margen de encoder para considerar punto alcanzado.

7. **Detección de límites**
   Evita choque contra los bordes del entorno (análogo a topes mecánicos).

8. **Evaluación mediante gráficas**
   Permite ajustar y validar el comportamiento PID antes de implementarlo en hardware.

---

**Conclusión**
Esta versión con controlador PID ofrece un seguimiento de línea más estable y robusto en TurtleSim, manteniendo la analogía con el control mediante encoders en robots reales y añadiendo prevención de choques contra las paredes.
