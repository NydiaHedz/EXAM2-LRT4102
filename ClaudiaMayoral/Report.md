
# Report on Results and Analysis of the Virtual Metric in the Turtlesim Simulation. 
## Claudia Fernanda Mayoral Sánchez 175787
## 1. Code

The presented code effectively implements a virtual angular control metric using a proportional controller, striking a balance between precision and simplicity. Constant reading of the angular position and dynamic error calculation enable fine control that emulates a physical encoder’s function.

The inclusion of the return-to-zero procedure and wait period represent practical measures to avoid cumulative errors and simulate real dispensing scenarios, thus ensuring safety and reliability in medication delivery.

This Turtlesim implementation not only validates the technical feasibility but also provides a flexible platform for tuning parameters, analyzing system behavior, and preparing for real hardware deployment.

Would you like me to prepare a line-by-line code breakdown for a specific function or integrate this with graphical and statistical results next?





Dijiste:
y pasalo a formato .md 


ChatGPT dijo:
Claro, aquí tienes el informe en formato Markdown (.md):

markdown
Copiar
# Detailed Report with Code Explanation on the Virtual Metric in Turtlesim

## Introduction

This project addresses a fundamental limitation in the stepper motor used in the autonomous medication dispensing robot: the lack of a physical encoder for position feedback. To compensate, a proportional controller (P-controller) was implemented within the Turtlesim simulation environment in ROS to emulate encoder behavior by continuously measuring and correcting the angular position with precision. Below, the control logic is described alongside key code excerpts demonstrating how the virtual metric was realized.

---

## 1. ROS Node Initialization and Setup

```python
rospy.init_node('control_tortuga_rotacion', anonymous=True)
self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
self.rate = rospy.Rate(10)
Explanation:
The ROS node is initialized to control the turtle. It subscribes to the /turtle1/pose topic to continuously receive updates on the turtle's current position and orientation, storing the angle in self.current_theta via the pose_callback method. Concurrently, it publishes angular velocity commands to the /turtle1/cmd_vel topic. The update rate is set to 10 Hz, balancing responsiveness and computational load.
```
## 2. Continuous Reading of Angular Position
```python

def pose_callback(self, pose):
    self.current_theta = pose.theta
```
Explanation:
Every time a position update arrives, pose_callback updates the current angle self.current_theta. This simulates the continuous reading an encoder would provide in a real motor system, enabling real-time feedback control.

## 3. Angular Error Calculation and Proportional Control
```python

error_theta = target_theta - self.current_theta
error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159
vel_z = Kp * error_theta
```
Explanation:
The angular error is computed as the difference between the target angle target_theta (converted to radians) and the current angle. The error is then wrapped within 
[
−
𝜋
,
𝜋
]
[−π,π] to ensure the turtle always rotates via the shortest path. The angular velocity command is proportional to this error by multiplying it by the gain constant Kp, which dictates the speed of correction.

This mechanism effectively simulates an encoder’s feedback function, providing dynamic error measurement and enabling continuous position correction.

## 4. Publishing Movement Commands and Precision Check
```python

twist_msg = Twist()
twist_msg.angular.z = vel_z
self.velocity_publisher.publish(twist_msg)

if abs(error_theta) < 0.1:
    rospy.loginfo(f"Target angle reached: {degrees(target_theta):.2f}°")
    break
```
Explanation:
A Twist message containing the calculated angular velocity is published to move the turtle. When the angular error falls below a predefined threshold (0.1 radians), the target angle is considered reached, and the rotation loop breaks to prevent unnecessary oscillations.

This stopping criterion mimics the precision of a motor with a physical encoder that can halt at an exact angular position.

## 5. Return-to-Zero Mechanism
```python
Copiar
def rotate_back_to_zero(self):
    self.rotate_turtle_to_target(0)
```
Explanation:
To prevent cumulative errors over multiple rotations, the turtle is programmed to return to a zero-degree reference position after each rotation. This ensures every new rotation command starts from a known, precise reference point, maintaining alignment and minimizing drift.

## 6. User Interaction for Target Angle Input
```python

def get_target_angle_from_user(self):
    print("Enter target rotation angle (degrees):")
    return float(input("Desired angle (degrees): "))
```
Explanation:
This method simulates the system receiving instructions from the robot’s internal controller or software, which calculates how many degrees the motor must rotate to align the dispenser compartment correctly. Here, the user inputs the desired angle, representing the internal calculation determining the motor’s required rotation.

## 7. Full Rotation Cycle and Simulated Waiting
```python

self.rotate_turtle_to_target(target_angle)
self.rotate_back_to_zero()
```
Additionally, a symbolic 10-second pause is introduced after reaching the target angle:

```python

rospy.loginfo("Waiting 10 seconds before returning to zero degrees...")
rospy.sleep(10)
```
Explanation:
The cycle consists of rotating to the target angle, waiting a fixed time to simulate medication dispensing, and returning to zero degrees. This prevents error accumulation and models real-world operation conditions.

# Report on Results and Analysis of the Virtual Metric in the Turtlesim Simulation
## 2. Description

## Introduction

In the context of developing the autonomous medication dispensing robot, one of the main technical challenges was the absence of a physical encoder on the stepper motor responsible for rotating the dispensing compartment. To overcome this limitation, a proportional control system (P-controller) was implemented in a simulated environment using Turtlesim, which emulates the behavior of a virtual encoder, allowing precise measurement and correction of the angular position.

This report details the experimental results, analysis, and conclusions obtained during the evaluation of the performance of this virtual metric, as well as the strategies implemented to mitigate cumulative errors and ensure system reliability in a hospital environment.

## Results and Analysis

### Part 1: Angular Precision Evaluation

Multiple tests were conducted focusing on achieving a target rotation of 90 degrees. During these tests, angular position and error data were recorded via console outputs, and graphs were generated using the Pyplot library to visualize the controller's behavior.

* **Observed angular error:** The proportional controller exhibited an angular error ranging approximately between 0.30° and 1.18°, an adequate precision level for a simple controller of this nature.
* **Graph analysis:** The generated curves show that the turtle reaches the target position with minor oscillations and stable convergence around the desired angle.
* **Improvement potential:** By adjusting the proportional gain (Kp), the error can be slightly reduced, although it remains within the expected margin.

This first phase validated the basic effectiveness of the proportional controller to reach angular positions with minimal error, sufficient for dispensing applications with moderate precision requirements.

### Part 2: Cumulative Error Problem

In the original simulation, after each rotation, the turtle requested a new target angle from the user and rotated directly from its current position, which led to progressive accumulation of errors.

* **Impact of cumulative error:**  
  After approximately 10 to 20 rotations, the dispenser door began to misalign with the correct compartment, affecting system accuracy.  
  Between 45 and 60 rotations, this misalignment could result in incorrect medication delivery to different patients, a critical risk in hospital applications.

This finding underscored the need for a solution to control and limit the effect of cumulative angular error.

### Part 3: Implementation of Return to Initial Position

To solve the identified problem, a strategy was adopted where the turtle is programmed to return to its initial position (0 degrees) after each rotation, before receiving the next angular instruction.

* **Benefits:**  
  This reset mechanism prevents error accumulation, maintaining precise alignment of the dispenser over multiple delivery cycles.  
  Safety and reliability are prioritized over energy efficiency, as the additional movement nearly doubles energy consumption per cycle.

* **Symbolic pause:**  
  A 10-second pause was added after each delivery, simulating the estimated time a patient takes to take their medication. This pause serves as a placeholder for future validation under real conditions.

## Justification and Conclusions

* **Reliability:**  
  The average angular error remains below 1 degree, ensuring reliable performance for a standard isolation room with 20 to 25 patients, with no significant risk of delivery errors.

* **Failure reduction:**  
  The encoder simulation together with the proportional controller and return-to-zero implementation significantly reduces the likelihood of critical errors, increasing dispensing safety.

* **Safety against clinical risks:**  
  By preventing cumulative errors from affecting alignment, the integrity of the process is protected and incorrect medication administration is avoided.

## Future Improvement Proposals

* **Interchangeable trays:**  
  It is proposed to introduce trays with compartments of various sizes adaptable to the number of patients, ranging from small units with 3 compartments for classrooms or small groups, up to trays with 50 compartments for large rooms or pandemic situations.

* **Ease of handling:**  
  These trays should be easily interchangeable by nursing staff to optimize maintenance and system adaptability.

* **Balance between capacity and risk:**  
  Fewer compartments reduce the chance of delivery error, although the system remains safe and efficient even at maximum capacity, which is crucial for high-demand scenarios.

* **Data-based adjustments:**  
  It is recommended to perform tests in real hospital environments to precisely calibrate pauses and validate controller behavior under operational conditions.

## Personal Conclusion

This project helped me better understand the importance of having a reliable feedback system to precisely control movement in robots, especially in sensitive applications like medication dispensing. Since the motor lacked a physical encoder, simulating a proportional controller in Turtlesim was a good alternative to maintain control and avoid cumulative errors.

During testing, I saw that although the controller is simple, with some adjustments and strategies like always returning to zero degrees, it achieves the necessary precision for the system to work well and be safe in a hospital environment.

Additionally, this project made me realize how fundamental ROS is as a platform for robot programming and how control system design is key to making systems work reliably. It became clear that while solutions can be simple, their correct implementation is crucial for practical applications. This work lays a solid foundation to move forward toward the physical implementation of the robot and to continue improving it through real-world testing.

# References: 
OpenAI. (2024). ChatGPT (modelo GPT-4) [Modelo de lenguaje AI]. https://chat.openai.com/

Forbes Mexico (2020). More Hospitals andBeds to Attend COVID-19 in CDMX. Retrieved from: https://forbes.com.mx/noticias-mas-hospitales-y-camas-para-atender-covid-19-cdmx/

Instructables (2023). A Simple Touchless Candy Dispenser. Retrieved from: https://www.instructables.com/A-Simple-Touchless-Candy-Dispenser/


