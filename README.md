# AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management

## **Problem Statement**
The problem involves using directional aids to detect defective and non defective packages. 

![Arena](https://github.com/ashiqrahmana/AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management/blob/main/Images/arena.jpeg)

### **Scenario**
The robot starts at the home location S. 
At every odd intersectiosn there will ber an directional aid in the forma  blue triangle. 
Using the aid to decide to move left or rigth at first and take the direction.
Proceedign to A or B lane the bot shoudl stop at the station A and B which contain aruco tags.
The detective and non defective pacakges for identified and noted.
This procedure is repeated until the no mans zone is reached where using the directional amrkers the bot is turned atill the markers are detected and bot proceeded to station as shown in thr arena.

## **Hardware**

**Controller** : Raspberry Pi, Parallax Propeller

**Sensors** : 
| Sensor | Model | Use |
|--------|-----|----|
| IR Sensor | QTI | For Line following |
| Camera | Pi-Camera | For Directiional Aids and packages |

**Actuator**
Parallax Continious Servo, Parallax Positional Servo

**Circuit Diagram**
![Circuit](https://github.com/ashiqrahmana/AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management/blob/main/Images/circuit.jpeg)
![Servo Circuit](https://github.com/ashiqrahmana/AutoInventoryBot-Autonomous-Robot-for-Warehouse-Inventory-Monitoring-and-Management/blob/main/Images/servo.jpeg)


## **Approach**
Since the problem involves navigating from one node to another, the A* algorithm is employed. This approach helps avoid any hardcoded navigation as the algorithm will automatically generate the next set of nodes that has to be traversed to reach the final node. 

At every node, A* path planner is invoked and the map is updated based on the ultrasonic sensor reading to block off unaccessable nodes. This forces the algorithm to generate alternative path that will get it to the destination. The child nodes are generated based on the accessible nodes from the given node to make sure that the lane constraints are met. 

Once the nodes are generated, we then generate the drive command by comparing the current and the next nodes and update the current node. If the command is a turn, we update both current node and orientation of the robot. This way, we can keep track of vertical lanes and accomodate the lane conditions.
