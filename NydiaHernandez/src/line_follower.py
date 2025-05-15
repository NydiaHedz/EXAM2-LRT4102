#!/usr/bin/env python3

"""
This script simulates a line-following scenario using ROS turtlesim. The first two turtles
simulate the generation of a path by moving along straight line segments. The third turtle
follows this generated route until it reaches a pause point.

The purpose of this simulation is to measure the pause duration metric precisely by using
a timer that ensures the stop lasts exactly 10 seconds. This setup verifies improved timing
accuracy compared to hardware implementations with variable delays.

Author: Nydia Hernandez Bravo
Date: May 14, 2025
"""

import rospy
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import time

class MultiTurtleLineSim:
    def __init__(self):
        rospy.init_node('multi_turtle_line_sim')
        
        # Publisher and subscriber for turtle3
        self.cmd_pub = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle3/pose', Pose, self.pose_callback)

        self.pose = None
        self.rate = rospy.Rate(10)
        self.results = []

        # Wait for required services
        rospy.wait_for_service('spawn')
        rospy.wait_for_service('kill')
        rospy.wait_for_service('clear')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        
        self.spawn_srv = rospy.ServiceProxy('spawn', Spawn)
        self.kill_srv = rospy.ServiceProxy('kill', Kill)
        self.clear_srv = rospy.ServiceProxy('clear', Empty)
        self.teleport_srv = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

    def pose_callback(self, data):
        # Update the current pose of turtle3
        self.pose = data

    def clean_slate(self):
        """Clear all turtles and drawings, reset turtle1 position"""
        try:
            # Clear all drawings on screen
            self.clear_srv()
            
            # Kill all turtles except turtle1
            for name in ['turtle1', 'turtle2', 'turtle3']:
                try:
                    self.kill_srv(name)
                except:
                    pass
            
            # Teleport turtle1 to initial position
            self.teleport_srv(5.5, 5.5, 0)
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"Cleanup error: {e}")

    def draw_line(self, turtle_name, start_x, start_y, end_x, speed=1.0):
        """Draw a straight line by moving a turtle from start_x to end_x at fixed y"""
        pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(20)

        # Kill turtle if it exists
        try:
            self.kill_srv(turtle_name)
        except:
            pass

        # Spawn turtle at starting position
        self.spawn_srv(start_x, start_y, 0, turtle_name)

        # Move turtle forward in a straight line
        distance = 0.0
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0

        while not rospy.is_shutdown() and distance < (end_x - start_x):
            pub.publish(cmd)
            rate.sleep()
            distance += speed / 20.0

        # Stop the turtle at the end position
        cmd.linear.x = 0.0
        pub.publish(cmd)

    def run_single_iteration(self, iteration):
        """Run one full simulation iteration"""
        stopped = False
        stop_start_time = None
        has_stopped = False
        moving_after_stop = False
        initial_x_after_stop = None
        path_complete = False
        last_print_time = 0

        # Draw first segment with turtle1
        self.draw_line('turtle1', 0.0, 5.0, 5.0)
        self.kill_srv('turtle1')

        # Draw second segment with turtle2
        self.draw_line('turtle2', 5.2, 5.0, 10.0)
        self.kill_srv('turtle2')

        # Spawn turtle3 to follow the path
        self.spawn_srv(0.0, 5.0, 0, 'turtle3')

        while not rospy.is_shutdown() and not path_complete:
            if self.pose is None:
                self.rate.sleep()
                continue

            cmd = Twist()

            if self.pose.x >= 10.0:
                path_complete = True
                self.results.append(f"Iteration {iteration}: Success")
                cmd.linear.x = 0
                break

            elif self.pose.x >= 5.0 and not has_stopped:
                stopped = True
                has_stopped = True
                stop_start_time = time.time()
                cmd.linear.x = 0
                print("\nStopped at gap. Timer: 0.0/10.0s", end='', flush=True)
            
            elif stopped:
                current_time = time.time()
                elapsed = current_time - stop_start_time
                
                # Update timer display every 0.1 seconds
                if current_time - last_print_time >= 0.1:
                    print(f"\rStopped at gap. Timer: {elapsed:.1f}/10.0s", end='', flush=True)
                    last_print_time = current_time
                
                if elapsed >= 10.0:
                    stopped = False
                    moving_after_stop = True
                    initial_x_after_stop = self.pose.x
                    cmd.linear.x = 1.0
                    print("\nResuming movement after 10 seconds stop")
                else:
                    cmd.linear.x = 0
            
            elif moving_after_stop:
                distance_moved = self.pose.x - initial_x_after_stop
                if distance_moved >= 0.2:
                    moving_after_stop = False
                    cmd.linear.x = 1.0
                else:
                    cmd.linear.x = 1.0
            
            else:
                cmd.linear.x = 1.0

            cmd.angular.z = 0
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        # Cleanup turtles and drawings
        self.clean_slate()

    def run(self):
        # Wait for turtlesim to fully start
        rospy.sleep(2)

        # Initial cleanup
        self.clean_slate()

        # Run a single iteration
        print("\nStarting iteration 1")
        self.run_single_iteration(1)

        # Print final results summary
        print("\n=== FINAL RESULTS ===")
        for result in self.results:
            print(result)
        print("====================")

if __name__ == '__main__':
    try:
        sim = MultiTurtleLineSim()
        sim.run()
    except rospy.ROSInterruptException:
        print("\nSimulation terminated by user")
