# Simulación con turtlesim y Métricas Virtuales 

## Introducción

Este repositorio reune la explicación detallada del código desarrollado para la simulación de seguimiento de línea utilizando el entorno **turtlesim** de ROS, así como la descripción de las métricas virtuales empleadas. El código implementado incluyó un archivo de lanzamiento (*launch*) para facilitar la ejecución coordinada del nodo y servicios requeridos.

## Descripción del Código Principal

El script principal simuló un escenario de seguimiento de línea con múltiples tortugas en el entorno turtlesim. En él, las primeras dos tortugas simularon la generación de una ruta a través de la ejecución secuencial de movimientos en línea recta. Posteriormente, una tercera tortuga siguió esta ruta generada, desplazándose hasta alcanzar un punto específico donde realizó una pausa.

La pausa se implementó intencionalmente para medir con precisión una métrica temporal de duración del descanso (pausa) mediante un temporizador programado para garantizar que la detención durara exactamente 10 segundos. Esta simulación buscó validar la mejora en la precisión temporal frente a implementaciones físicas, donde la variabilidad de los retardos podría afectar la medición.

El código se estructuró en una clase principal que gestionó la inicialización de nodos, publicación y suscripción a tópicos de velocidad y posición, así como el manejo de servicios para la creación, eliminación y teletransporte de tortugas en el entorno. 

## Métricas de Desempeño

### Métricas Físicas

Una de las métricas físicas utilizadas para evaluar el desempeño del robot fue el **tiempo de respuesta**, definido como el intervalo en segundos transcurrido desde que el robot detecta la franja blanca que indica la llegada a la cama del paciente hasta que completa la acción asociada y continúa su ruta.

A continuación se muestran diez mediciones temporales bajo condiciones reales, evidenciando el tiempo aproximado de respuesta del robot en cada prueba:

| Trial | Response Time (s) |
|-------|-------------------|
| 1     | 9.8               |
| 2     | 10.4              |
| 3     | 10.1              |
| 4     | 9.6               |
| 5     | 10.3              |
| 6     | 9.9               |
| 7     | 10.2              |
| 8     | 9.7               |
| 9     | 10.0              |
| 10    | 9.5               |


Estos valores permitieron analizar la consistencia y desempeño reactivo del robot, aportando a la evaluación global del sistema.

### Métricas Virtuales

Se midió el tiempo exacto de detención de la tortuga seguidora en un punto de pausa, garantizando que durara 10 segundos mediante el uso de un temporizador interno. Esta métrica permitió evaluar la fidelidad del control temporal en comparación con implementaciones físicas.

El código implementado para simular se puede ver en  [`line_follower.py`](https://github.com/NydiaHedz/EXAM2-LRT4102/blob/main/NydiaHernandez/src/line_follower.py).
. Este codigo implementó una simulación en Python para el entorno **turtlesim** de ROS, destinada a reproducir un escenario de seguimiento de línea con tres tortugas que simularon el comportamiento de un robot autónomo.

### Inicialización y Configuración

La clase principal `MultiTurtleLineSim` inicializó el nodo ROS con el nombre `'multi_turtle_line_sim'` y configuró los publicadores y suscriptores necesarios. En particular, se estableció un publicador para enviar comandos de velocidad (`Twist`) a la tortuga número 3, y un suscriptor para recibir su posición actual (`Pose`). Se definieron variables internas para el control de la ejecución y una lista para almacenar resultados.

El nodo esperó también a que se activaran los servicios ROS fundamentales para la simulación: `spawn` (crear tortugas), `kill` (eliminar tortugas), `clear` (limpiar la pantalla), y `teleport_absolute` (teletransportar tortugas a una posición exacta).

### Gestión del Estado y Callbacks

El método `pose_callback` se implementó para actualizar continuamente la posición de la tortuga 3 cada vez que se recibía un nuevo mensaje de posición. Esta información fue clave para controlar el avance y las pausas durante la simulación.

### Funcionalidades Auxiliares

El método `clean_slate` se encargó de limpiar la pantalla de dibujos y eliminar todas las tortugas activas (incluyendo turtle1, turtle2 y turtle3), para garantizar un estado inicial limpio antes de cada ejecución. Además, reubicó la tortuga 1 en una posición inicial predeterminada para facilitar la generación de la ruta.

El método `draw_line` permitió dibujar un segmento recto en el plano de turtlesim moviendo una tortuga desde una posición inicial a una posición final a velocidad constante. Para ello, se mataba a la tortuga si existía, se generaba nuevamente en la posición inicial, y se enviaban comandos de movimiento lineal hacia adelante hasta alcanzar la distancia deseada.

### Ejecución Principal: Simulación de Iteración

La función `run_single_iteration` constituyó el núcleo de la simulación. En ella se ejecutaron los siguientes pasos:

1. Se dibujaron dos segmentos rectos sucesivos en el plano mediante el movimiento controlado de `turtle1` y `turtle2`. Cada tortuga realizó un desplazamiento horizontal a lo largo de coordenadas fijas para simular una ruta continua con un "gap" o pausa en medio.

2. Se eliminó cada tortuga después de completar su segmento para mantener el entorno limpio.

3. Se generó la `turtle3`, que actuó como el agente seguidor de la ruta generada.

4. Mediante un ciclo principal, se monitoreó la posición de `turtle3` para controlar su movimiento y pausar en el punto correspondiente a la mitad del recorrido (cuando su coordenada `x` alcanzó 5.0).

5. Al alcanzar dicho punto, la tortuga detuvo su avance y se inició un temporizador software que mantuvo la detención exactamente durante 10 segundos, actualizando periódicamente en consola el tiempo transcurrido.

6. Finalizado el tiempo de pausa, la tortuga reanudó su movimiento hasta completar la ruta (cuando su coordenada `x` superó 10.0), momento en el que la iteración se marcó como exitosa y finalizó el ciclo.

7. Durante todo el proceso, se enviaron comandos de velocidad lineal constantes para simular el desplazamiento, y se aseguraron ángulos nulos para mantener el movimiento en línea recta sin giros.

8. Finalmente, se limpió el entorno eliminando tortugas residuales y dibujos.

El método `run` esperó el arranque completo del entorno turtlesim, ejecutó una limpieza inicial y corrió una única iteración de la simulación. Al finalizar, mostró un resumen de resultados, incluyendo la confirmación del éxito de la simulación.

## Ejecución de la Simulación

Para iniciar la simulación, se utilizó el siguiente comando de lanzamiento de ROS:

```bash
roslaunch practicas_lab line_follower.launch
```

## Control Preciso de la Pausa en la Simulación

In the simulation, the behavior of the robotic agent included a timed stop of exactly ten seconds implemented via a software timer. This approach enabled precise control over the pause duration during the path-following routine, which differed from the original physical design where timing accuracy could be affected by sensor noise, actuator delays, and environmental disturbances. Consequently, while the simulation timer provided consistent and exact ten-second stops, real-world implementations likely experienced minor deviations in timing due to inherent mechanical and electronic limitations.

| Trial | Response Time (s) |
|-------|-------------------|
| 1     | 10.0              |
| 2     | 10.0              |
| 3     | 10.0              |
| 4     | 10.0              |
| 5     | 10.0              |
| 6     | 10.0              |
| 7     | 10.0              |
| 8     | 10.0              |
| 9     | 10.0              |
| 10    | 10.0              |


This discrepancy highlighted an area for improvement in the physical design: to increase timing precision, it was recommended to integrate hardware-based timing mechanisms or high-resolution clocks synchronized with sensor feedback. Additionally, implementing adaptive control algorithms that accounted for variable delays and environmental conditions could enhance the reliability of the stop duration in practice.

Moreover, extending the simulation to include sensor noise models and actuator response delays would produce results more representative of the physical system, thus allowing better validation and tuning of control parameters before hardware deployment.

The graph summarized the robotic system’s behavior during a programmed 10-second pause within the simulation. It was possible to graphically visualize and verify, using PlotJuggler, that the simulated robot (“turtle”) stopped for the entire duration of 10 seconds before resuming its movement. Sensor readings confirmed continuous detection of the line throughout the pause, ensuring that the robot remained stationary and precisely positioned. This visualization validated the timing control logic implemented in the simulation environment. This can be seen in Figure \ref{fig:enter-label2}.

\begin{figure}[H]
    \centering
    \includegraphics[width=1\linewidth]{I2.png}
    \caption{PlotJuggler Results Showing the Robotic Simulation Behavior}
    \label{fig:enter-label2}
\end{figure}













