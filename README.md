# AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management

## **Problem Statement**
The problem involves using directional aids to detect defective and non defective packages. 

![Arena](https://github.com/ashiqrahmana/AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management/blob/main/Images/arena.jpeg)

### **Scenario**
The robot starts at the home location S. At every odd intersection, there will be a directional aid in the form of a blue triangle. Using the aid, the robot decides to move left or right initially and takes that direction. Proceeding to lanes A or B, the bot should stop at stations A and B, which contain ArUco tags. The defective and non-defective packages are identified and noted. This procedure is repeated until the no man's zone is reached, where the bot is turned using the directional markers until the markers are detected, and then the bot proceeds to the station as shown in the arena.

## **Hardware**

**Controller** : Raspberry Pi, Parallax Propeller

**Sensors** : 
| Sensor | Model | Use |
|--------|-----|----|
| IR Sensor | QTI | For Line following |
| Camera | Pi-Camera | For Directional Aids and packages |

**Actuator**
Parallax Continious Servo, Parallax Positional Servo

**Circuit Diagram**
![Circuit](https://github.com/ashiqrahmana/AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management/blob/main/Images/circuit.jpeg)
![Servo Circuit](https://github.com/ashiqrahmana/AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management/blob/main/Images/servo.jpeg)


## **Uniqueness**
Using Visual Servoing for the Auto Inventory Bot

Visual servoing is a technique that utilizes visual feedback from a camera to control the motion of a robot. In the context of the Auto Inventory Bot, visual servoing can be employed to enhance the robot's ability to navigate and interact with the environment more accurately.

The following steps outline the process of using visual servoing for the Auto Inventory Bot:

Object Detection: Utilize computer vision techniques to detect and locate the objects of interest (e.g., packages, directional aids, ArUco tags) within the camera's field of view. This can involve techniques like image segmentation, feature extraction, and object recognition.

Visual Servoing Control: Implement a control algorithm that uses the visual feedback to compute the robot's motion commands. This control algorithm typically minimizes the error between the current visual features (object pose) and the desired features (from the desired trajectory).

Feedback Loop: Continuously update the visual feedback, recalculate the error, and adjust the robot's motion accordingly. This feedback loop ensures that the robot can adapt to changes in the environment and maintain accurate control.

By incorporating visual servoing into the Auto Inventory Bot's control system, the robot can enhance its perception capabilities and make more precise and adaptive movements. This can lead to improved navigation, object manipulation, and inventory management within the warehouse environment.
