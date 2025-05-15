# Simulador de Seguimiento de Línea usando TurtleSim

## 1. Importación de bibliotecas y definición de clase

```python
import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import matplotlib.pyplot as plt
```

- **math**: Contiene funciones matemáticas, útiles para conversiones y cálculos trigonométricos.
- **rospy**: Biblioteca para interactuar con ROS desde Python.
- **Twist**: Define velocidades lineales y angulares del robot.
- **Pose**: Representa la posición (x, y) y orientación angular (θ) del robot.
- **Spawn, Kill**: Servicios de turtlesim para crear y eliminar tortugas.
- **matplotlib.pyplot**: Permite generar gráficas de trayectorias y errores.

La clase principal `LineFollowerSimulator` agrupa toda la lógica de simulación:

```python
class LineFollowerSimulator(object):
    def __init__(self):
        rospy.init_node('line_follower_simulator', anonymous=True)
        self.spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        self.kill_turtle = rospy.ServiceProxy('/kill', Kill)
        self.pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/turtle1/pose', Pose, self._pose1_cb)
        self.path1 = []
        self.start_time1 = None
        # Inicialización adicional...
```

## 2. Callbacks y funciones auxiliares

### Callback de posición (Pose) de la primera tortuga (`turtle1`):

```python
def _pose1_cb(self, msg):
    if hasattr(self, 'start_time1') and self.start_time1 is not None:
        t = rospy.Time.now().to_sec() - self.start_time1
        self.path1.append((t, msg.x, msg.y, msg.theta))
```

- Al recibir información de posición, registra el tiempo actual junto con coordenadas y orientación.

### Función `move`

```python
def move(self, linear_speed, angular_speed, duration, publisher):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    start = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start < duration:
        publisher.publish(twist)
    twist.linear.x = 0
    twist.angular.z = 0
    publisher.publish(twist)
```

- Ejecuta un movimiento específico durante un tiempo determinado y luego detiene la tortuga.

## 3. Generación de trayectoria (turtle1)

En el método `run()`:

1. Espera la pose inicial de `turtle1` y registra su posición inicial.
2. Realiza movimientos predefinidos:
   - Movimiento rectilíneo durante **3 s** con velocidad lineal **1.0**.
   - Giro de **90°** (π/2 radianes) con velocidad angular de **30°/s**.
   - Nuevo movimiento rectilíneo durante otros **3 s**.
3. La posición después de cada movimiento queda almacenada.
4. Finalmente elimina a `turtle1` mediante el servicio `kill`.

## 4. Generación y seguimiento de trayectoria con la segunda tortuga

1. Se crea una segunda tortuga (`turtle2`) exactamente en la posición inicial de la primera.
2. Se suscribe al tópico `/turtle2/pose` para registrar la trayectoria seguida.
3. Parámetros del controlador:
   - `V_CONST = 1.0`  
   - `K_P = 0.5`  
   - `MAX_ANG_VEL`: Límite de velocidad angular máxima.
   - `DIST_TH = 0.15`: Umbral de proximidad al punto objetivo.
4. Algoritmo de seguimiento:
   - Selecciona el siguiente waypoint guardado de `turtle1`.
   - Calcula la distancia y el error angular hacia dicho waypoint.
   - Aplica control proporcional: `omega = K_P * error` (limitado por `MAX_ANG_VEL`).
   - Calcula la velocidad lineal como `V_CONST * cos(error)`.
   - Publica el mensaje Twist correspondiente y avanza hacia el waypoint.
   - Guarda los errores para su posterior análisis.

## 5. Gráficos finales de trayectorias y error angular

- `plot_paths()`: Muestra gráficamente las trayectorias de ambas tortugas.
- `plot_angular_error()`: Muestra cómo evoluciona el error angular a través del tiempo.
- Ambas gráficas incluyen etiquetas, leyendas y cuadrículas para facilitar el análisis.

## 6. Ejecución principal

```python
if __name__ == '__main__':
    try:
        simulator = LineFollowerSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
```

- Inicializa el simulador y ejecuta el método principal.
- Maneja adecuadamente interrupciones externas como Ctrl+C.

---

## Resumen del funcionamiento y equivalencia con señales reales de encoder

1. **Generación de trayectoria**:  
   La primera tortuga genera una trayectoria definida almacenando posiciones y orientaciones que simulan señales de encoders reales.

2. **Simulación del entorno**:  
   Posteriormente, la segunda tortuga inicia en la misma posición inicial de la primera y sigue la trayectoria almacenada.

3. **Control proporcional de seguimiento**:  
   El control implementado para el seguimiento es proporcional, ajustando las velocidades lineales y angulares en función del error angular y la distancia al objetivo.

4. **Analogía con encoders reales**:  
   La trayectoria guardada por la primera tortuga es análoga a una serie de señales provenientes de encoders en robots reales, indicando desplazamiento y orientación.

5. **Adaptación dinámica de velocidad**:  
   La velocidad lineal se adapta según el error angular, imitando estrategias reales en robots seguidores de líneas que ajustan su avance dependiendo de cuánto se desvían de la trayectoria.

6. **Umbral de proximidad como conteo de encoder**:  
   El parámetro `DIST_TH` funciona como una representación virtual del margen aceptable para considerar alcanzado un punto, equivalente a utilizar cuentas de encoder en aplicaciones reales para avanzar entre segmentos.

7. **Evaluación mediante gráficas**:  
   Las gráficas generadas permiten evaluar la precisión del controlador, semejante a la verificación en robots reales para ajustar parámetros y garantizar precisión en la navegación.

**Conclusión:**  
El script desarrollado simula de forma efectiva un sistema de seguimiento de línea con lógica equivalente a la utilizada en robots reales, considerando explícitamente conceptos de control posicional basado en señales provenientes de encoders. Representa una herramienta útil para validar la lógica de control antes de implementarla en sistemas reales.

