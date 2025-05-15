# Simulation with turtlesim and Virtual Metrics
Author: Nydia Hernández Bravo 

## Introduction

This repository gathered a detailed explanation of the code developed for the line-following simulation using the **turtlesim** environment of ROS, along with the description of the virtual metrics employed. The implementation included a launch file to facilitate the coordinated execution of the required node and services.

## Main Code Description

The main script simulated a line-following scenario with multiple turtles in turtlesim. The first two turtles generated a path by sequentially moving along straight-line segments. Subsequently, a third turtle followed this generated path, moving until reaching a designated pause point.

The pause was implemented intentionally to precisely measure a temporal metric, ensuring the stop lasted exactly 10 seconds via a software timer. This simulation validated the improvement in timing accuracy compared to physical implementations, where variable delays could affect the measurement.

The code was structured in a main class managing node initialization, publishing and subscribing to velocity and pose topics, as well as handling services to create, delete, and teleport turtles within the environment.

## Detailed Explanation of the Code

The simulation code can be reviewed at [`line_follower.py`](https://github.com/NydiaHedz/EXAM2-LRT4102/blob/main/NydiaHernandez/src/line_follower.py).

### Initialization and Setup

The `MultiTurtleLineSim` class initialized the ROS node `'multi_turtle_line_sim'` and configured the necessary publishers and subscribers to control the following turtle and receive its position. It also waited for essential services: spawning, killing, clearing, and teleporting turtles.

### State Management and Callbacks

A callback function updated turtle 3’s position continuously, enabling reactive control of its movement and pauses.

### Auxiliary Functions

The `clean_slate` function reset the environment by removing all turtles and drawings. The `draw_line` function allowed each turtle to draw a straight line by moving from a start to an end position at a constant speed.

### Iteration Simulation

In `run_single_iteration`, two straight segments were drawn by turtles 1 and 2, respectively, which were removed after each segment. Turtle 3 then followed the generated path. Upon reaching halfway, it paused for exactly 10 seconds controlled by a software timer, then resumed movement until completing the path. Velocity commands ensured steady, straight movement without rotation. The environment was cleaned after completion.

The `run` method awaited full turtlesim startup, performed initial cleanup, executed one iteration, and displayed a final success summary.

## Performance Metrics

### Physical Metrics

One key physical metric used to evaluate the robot was the **response time**, defined as the interval in seconds from when the robot detects the white stripe indicating arrival at the patient’s bed until it completes the associated action and continues its route.

Below are ten measurements under real conditions showing the approximate response time for each trial:

| Trial | Response Time (s) |
| ----- | ----------------- |
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

These results allowed for analysis of the robot’s consistency and reactive performance, contributing to the overall system evaluation.

### Virtual Metrics

The exact stop time of the following turtle was measured at the pause point, ensuring a precise 10-second duration using an internal timer, which enabled evaluation of temporal control fidelity compared to the physical system.

## Simulation Execution

To start the simulation, the following ROS launch command was used:

```bash
roslaunch practicas_lab line_follower.launch
```

## Precise Pause Control in Simulation

In the simulation, the robotic agent included a timed stop of exactly ten seconds implemented via a software timer. This enabled precise control over the pause duration during the path-following routine, contrasting with the original physical design where timing accuracy could be affected by sensor noise, actuator delays, and environmental disturbances. Consequently, while the simulation timer ensured consistent exact stops, real-world implementations would likely experience minor timing deviations due to mechanical and electronic limitations.

| Trial | Response Time (s) |
| ----- | ----------------- |
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

This discrepancy highlighted a design improvement area: to increase timing precision, it is recommended to integrate hardware-based timing mechanisms or high-resolution clocks synchronized with sensor feedback. Additionally, adaptive control algorithms accounting for variable delays and environmental conditions could enhance reliability in practice.

Extending the simulation to include sensor noise models and actuator response delays would yield results more representative of the physical system, enabling better validation and parameter tuning before hardware deployment.

The graph below summarizes the robotic system’s behavior during the programmed 10-second pause in simulation. Using PlotJuggler, it was possible to visualize and verify that the simulated robot (“turtle”) remained stopped for the entire pause duration before resuming movement. Sensor readings confirmed continuous line detection, ensuring the robot remained stationary and correctly positioned, thereby validating the timing control logic.

![Simulation Results in PlotJuggler](https://github.com/NydiaHedz/EXAM2-LRT4102/raw/main/NydiaHernandez/media/I2.png)

Aquí tienes una conclusión formal y técnica, en inglés, adecuada para cerrar el reporte o sección sobre la simulación:

---

## Conclusion

The simulation developed using the ROS turtlesim environment effectively demonstrated the feasibility of precise temporal control in an autonomous line-following task. By implementing a software timer to enforce an exact 10-second pause, the simulation achieved a level of timing accuracy that surpasses typical physical implementations, which are subject to noise, actuator delays, and environmental disturbances.

The performance metrics obtained from both physical tests and virtual simulations provided valuable insights into the system’s consistency and responsiveness. The virtual metrics, in particular, confirmed the robustness of the control logic under ideal conditions, highlighting opportunities to improve the physical system’s timing precision through hardware synchronization and adaptive control strategies.


