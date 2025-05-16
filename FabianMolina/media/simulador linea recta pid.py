#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import matplotlib.pyplot as plt

class LineFollowerSimulatorPID(object):
    def __init__(self):
        rospy.init_node('line_follower_simulator_pid', anonymous=True)
        rospy.loginfo("[Simulator PID] Nodo inicializado")

        rospy.wait_for_service('spawn')
        rospy.wait_for_service('kill')
        self.spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        self.kill_turtle  = rospy.ServiceProxy('kill', Kill)

        self.vel_pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown() and self.vel_pub1.get_num_connections() == 0:
            rospy.sleep(0.1)

        self.path1 = []
        self.errors = []
        self.time_errors = []
        self.pose2 = None

        rospy.Subscriber('/turtle1/pose', Pose, self._pose1_cb)

    def _pose1_cb(self, msg):
        if hasattr(self, 'start_time1') and self.start_time1 is not None:
            t = rospy.Time.now().to_sec() - self.start_time1
            self.path1.append((t, msg.x, msg.y, msg.theta))

    def _pose2_cb(self, msg):
        self.pose2 = msg

    def move(self, linear_speed, angular_speed, duration, publisher):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        rate = rospy.Rate(5)
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start) < duration:
            publisher.publish(twist)
            rate.sleep()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        publisher.publish(twist)

    def plot_paths(self):
        if not self.path1:
            rospy.logwarn("[Simulator PID] Trayectoria incompleta, no hay datos para graficar.")
            return
        xs1, ys1 = zip(*[(x, y) for _, x, y, _ in self.path1])
        plt.figure()
        plt.plot(xs1, ys1, label='Tortuga 1')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Trayectoria turtle1 (línea recta)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def plot_angular_error(self):
        if not self.errors:
            rospy.logwarn("[Simulator PID] No hay errores registrados para graficar.")
            return
        plt.figure()
        plt.plot(self.time_errors, self.errors, label='Error angular (°)')
        plt.axhline(0, linestyle='--', label='Sin error')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Error (°)')
        plt.title('Error angular vs Tiempo (PID)')
        plt.legend()
        plt.grid(True)
        plt.show()

    def run(self):
        rospy.sleep(1.0)
        rospy.loginfo("[Simulator PID] Esperando pose inicial de turtle1...")
        initial_pose = rospy.wait_for_message('/turtle1/pose', Pose)
        self.path1 = [(0.0, initial_pose.x, initial_pose.y, initial_pose.theta)]
        self.start_time1 = rospy.Time.now().to_sec()

        # Trayectoria solo línea recta con turtle1 (menos tiempo para evitar choque)
        self.move(1.0, 0.0, 4.0, self.vel_pub1)  # Avanza 4 segundos en línea recta

        rospy.sleep(1.0)
        try:
            self.kill_turtle('turtle1')
        except rospy.ServiceException:
            rospy.logwarn("[Simulator PID] No se pudo matar turtle1.")

        # Inicializar turtle2 en la posición inicial
        _, x0, y0, theta0 = self.path1[0]
        self.spawn_turtle(x0, y0, theta0, 'turtle2')
        rospy.sleep(1.0)

        vel_pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown() and vel_pub2.get_num_connections() == 0:
            rospy.sleep(0.1)
        rospy.Subscriber('/turtle2/pose', Pose, self._pose2_cb)

        # Parámetros controlador PID suavizados
        V_CONST     = 1.0
        K_P         = 0.8
        K_I         = 0.02
        K_D         = 0.01
        MAX_ANG_VEL = math.radians(30)
        DIST_TH     = 0.1

        self.errors = []
        self.time_errors = []
        integral = 0.0
        last_error = 0.0

        t_start2 = rospy.Time.now().to_sec()

        rate = rospy.Rate(20)
        target_point = self.path1[-1]  # Último punto de la línea recta

        while not rospy.is_shutdown():
            if self.pose2 is None:
                rate.sleep()
                continue

            t_rel = rospy.Time.now().to_sec() - t_start2

            # Guardar trayectoria para graficar
            if not hasattr(self, 'path2'):
                self.path2 = []
            self.path2.append((t_rel, self.pose2.x, self.pose2.y, self.pose2.theta))

            # Detectar si está cerca de paredes (límites de turtlesim 0.5 a 10.5)
            if self.pose2.x < 0.5 or self.pose2.x > 10.5 or self.pose2.y < 0.5 or self.pose2.y > 10.5:
                rospy.logwarn("[Simulator PID] Tortuga cerca de la pared, deteniendo.")
                break

            # Calcular error angular respecto al punto final
            _, x_t, y_t, _ = target_point
            dx, dy = x_t - self.pose2.x, y_t - self.pose2.y
            dist = math.hypot(dx, dy)

            angle_to_target = math.atan2(dy, dx)
            error = math.atan2(math.sin(angle_to_target - self.pose2.theta),
                               math.cos(angle_to_target - self.pose2.theta))

            # PID
            dt = 1.0 / 20.0
            integral += error * dt
            derivative = (error - last_error) / dt
            last_error = error

            omega = K_P * error + K_I * integral + K_D * derivative
            omega = max(min(omega, MAX_ANG_VEL), -MAX_ANG_VEL)

            # Guardar error para gráfica en grados
            self.errors.append(math.degrees(error))
            self.time_errors.append(t_rel)

            # Comando de movimiento
            twist2 = Twist()
            twist2.linear.x = V_CONST
            twist2.angular.z = omega
            vel_pub2.publish(twist2)

            # Detener si está cerca del objetivo final
            if dist < DIST_TH:
                rospy.loginfo("[Simulator PID] Llegó al punto objetivo final.")
                break

            rate.sleep()

        vel_pub2.publish(Twist())
        rospy.loginfo("[Simulator PID] Seguimiento finalizado.")
        self.plot_paths()
        self.plot_angular_error()

if __name__ == '__main__':
    try:
        simulator = LineFollowerSimulatorPID()
        simulator.run()
    except rospy.ROSInterruptException:
        pass