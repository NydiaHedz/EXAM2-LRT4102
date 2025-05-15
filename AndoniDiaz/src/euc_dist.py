#!/usr/bin/env python
# -- coding: utf-8 --

import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import matplotlib.pyplot as plt

class LineFollowerSimulator(object):
    def _init_(self):
        rospy.init_node('line_follower_simulator', anonymous=True)
        rospy.loginfo("[Simulator] Nodo inicializado")

        rospy.wait_for_service('spawn')
        rospy.wait_for_service('kill')
        self.spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        self.kill_turtle  = rospy.ServiceProxy('kill', Kill)

        self.vel_pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown() and self.vel_pub1.get_num_connections() == 0:
            rospy.sleep(0.1)

        self.path1 = []
        self.path2 = []
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
        if not self.path1 or not self.path2:
            rospy.logwarn("[Simulator] Trayectorias incompletas, no hay datos para graficar.")
            return
        xs1, ys1 = zip(*[(x, y) for _, x, y, _ in self.path1])
        xs2, ys2 = zip(*[(x, y) for _, x, y, _ in self.path2])
        plt.figure()
        plt.plot(xs1, ys1, label='Tortuga 1')
        plt.plot(xs2, ys2, label='Tortuga 2')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Trayectorias')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def plot_angular_error(self):
        if not self.errors:
            rospy.logwarn("[Simulator] No hay errores registrados para graficar.")
            return
        plt.figure()
        plt.plot(self.time_errors, self.errors, label='Error angular (°)')
        plt.axhline(0, linestyle='--', label='Sin error')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Error (°)')
        plt.title('Error angular vs Tiempo')
        plt.legend()
        plt.grid(True)
        plt.show()

    def run(self):
        rospy.sleep(1.0)
        rospy.loginfo("[Simulator] Esperando pose inicial de turtle1...")
        initial_pose = rospy.wait_for_message('/turtle1/pose', Pose)
        self.path1 = [(0.0, initial_pose.x, initial_pose.y, initial_pose.theta)]
        self.start_time1 = rospy.Time.now().to_sec()

        # Generar trayectoria con turtle1
        self.move(1.0, 0.0, 3.0, self.vel_pub1)
        ang_rad = math.radians(90)
        ang_vel = math.radians(30)
        self.move(1.0, ang_vel, ang_rad / ang_vel, self.vel_pub1)
        self.move(1.0, 0.0, 3.0, self.vel_pub1)

        rospy.sleep(1.0)
        try:
            self.kill_turtle('turtle1')
        except rospy.ServiceException:
            rospy.logwarn("[Simulator] No se pudo matar turtle1.")

        # Inicializar turtle2
        _, x0, y0, theta0 = self.path1[0]
        self.spawn_turtle(x0, y0, theta0, 'turtle2')
        rospy.sleep(1.0)

        vel_pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown() and vel_pub2.get_num_connections() == 0:
            rospy.sleep(0.1)
        rospy.Subscriber('/turtle2/pose', Pose, self._pose2_cb)

        # Parámetros de control
        V_CONST     = 1.0
        K_P         = 0.5
        MAX_ANG_VEL = math.radians(20)
        DIST_TH     = 0.15
        LOOKAHEAD   = 1

        self.path2 = []
        self.errors = []
        self.time_errors = []
        t_start2 = rospy.Time.now().to_sec()

        rate = rospy.Rate(20)
        idx = 0
        while not rospy.is_shutdown() and idx < len(self.path1) - 1:
            if self.pose2 is None:
                rate.sleep()
                continue

            t_rel = rospy.Time.now().to_sec() - t_start2
            self.path2.append((t_rel, self.pose2.x, self.pose2.y, self.pose2.theta))

            # Objetivo inmediato
            _, x_t, y_t, _ = self.path1[idx + 1]
            dx, dy = x_t - self.pose2.x, y_t - self.pose2.y
            dist = math.hypot(dx, dy)

            # Error angular
            angle_to_target = math.atan2(dy, dx)
            error = math.atan2(math.sin(angle_to_target - self.pose2.theta),
                               math.cos(angle_to_target - self.pose2.theta))
            error_deg = math.degrees(error)

            self.errors.append(error_deg)
            self.time_errors.append(t_rel)

            # Control P angular
            omega = max(min(K_P * error, MAX_ANG_VEL), -MAX_ANG_VEL)
            twist2 = Twist()
            # Modulación lineal según alineación
            twist2.linear.x = V_CONST * max(0.0, math.cos(error))
            twist2.angular.z = omega
            vel_pub2.publish(twist2)

            # Avanzar waypoint si cerca
            if dist < DIST_TH:
                idx += 1

            rate.sleep()

        vel_pub2.publish(Twist())
        rospy.loginfo("[Simulator] Seguimiento completado.")
        self.plot_paths()
        self.plot_angular_error()

if _name_ == '_main_':
    try:
        simulator = LineFollowerSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
