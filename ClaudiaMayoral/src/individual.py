#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, degrees

class TurtleRotationProportionalControl:
    def __init__(self):
        # Inicializa el nodo ROS para controlar la tortuga
        rospy.init_node('control_tortuga_rotacion', anonymous=True)
        
        # Me suscribo para recibir la posición actual de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publico comandos para mover la tortuga (velocidad angular)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Frecuencia a la que voy a publicar los comandos (10 veces por segundo)
        self.rate = rospy.Rate(10)
        
        self.current_theta = 0  # Aquí guardo el ángulo actual de la tortuga

    def pose_callback(self, pose):
        # Cada vez que llega la posición de la tortuga actualizo el ángulo
        self.current_theta = pose.theta

    def rotate_turtle_to_target(self, target_theta):
        # Constante para controlar la velocidad de rotación (puede ajustarse)
        Kp = 5.0  # Le pongo 5 para que gire más rápido
        
        # El ángulo objetivo lo convierto de grados a radianes
        target_theta = radians(target_theta)
        
        # Calculo inicial del error entre posición deseada y actual
        error_theta = target_theta - self.current_theta
        
        while not rospy.is_shutdown():
            # Calculo el error actual (qué tanto falta para llegar al ángulo deseado)
            error_theta = target_theta - self.current_theta

            # Ajusto el error para que siempre esté en el rango de -pi a pi
            # Esto hace que la tortuga gire por el camino más corto
            error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159

            # La velocidad angular será proporcional al error (más error, más rápido gira)
            vel_z = Kp * error_theta

            # Creo el mensaje para mandar la velocidad angular a la tortuga
            twist_msg = Twist()
            twist_msg.angular.z = vel_z
            
            # Mando el comando para que la tortuga gire
            self.velocity_publisher.publish(twist_msg)

            # Imprimo en consola el error actual en grados, para saber qué tan cerca está
            rospy.loginfo(f"Error de ángulo: {degrees(error_theta):.2f}°")
            
            # Si el error es pequeño (menos de 0.1 radianes), considero que ya llegó
            if abs(error_theta) < 0.1:
                rospy.loginfo(f"Ángulo objetivo alcanzado: {degrees(target_theta):.2f}°")
                break

            # Espero para mantener la frecuencia de 10 Hz
            self.rate.sleep()

        # Cuando ya llegó, detengo la rotación (velocidad angular en cero)
        twist_msg.angular.z = 0
        self.velocity_publisher.publish(twist_msg)

        # Espero 10 segundos antes de hacer que regrese al ángulo cero
        rospy.loginfo("Esperando 10 segundos antes de regresar a cero grados...")
        rospy.sleep(10)

    def rotate_back_to_zero(self):
        # Función para hacer que la tortuga vuelva a la posición inicial (0 grados)
        target_theta = 0
        self.rotate_turtle_to_target(target_theta)

    def get_target_angle_from_user(self):
        # Pido al usuario que ingrese el ángulo al que quiere que gire la tortuga
        print("Ingrese el ángulo objetivo de rotación (en grados):")
        return float(input("Ángulo deseado (grados): "))

    def rotate_turtle_interactively(self):
        # Bucle principal que corre hasta que se cierre ROS
        while not rospy.is_shutdown():
            # Leo el ángulo deseado del usuario
            target_angle = self.get_target_angle_from_user()

            # Mando a la tortuga a girar hasta ese ángulo
            self.rotate_turtle_to_target(target_angle)

            # Después de esperar 10 segundos, regreso a cero grados
            self.rotate_back_to_zero()

if __name__ == '__main__':
    try:
        # Creo el objeto que controla la tortuga y ejecuto el método principal
        turtle_rotation_control = TurtleRotationProportionalControl()
        turtle_rotation_control.rotate_turtle_interactively()
    except rospy.ROSInterruptException:
        # Si se cierra ROS limpio y salgo sin error
        pass
