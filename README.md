# E - Yantra 2020/21: Vargi Bot

An automated warehouse simulation developed with an aim to sort the packages according to their colour in least possible time using robotic arms.

# Task Given

The arena is an automated warehouse setting where essential packages are required to be sent out to different parts of the city.The warehouse will only consist of two industrial robotic arms which will be used by the teams. As the requirements are sent to the warehouse, one robotic arm will identify the packages from a shelf and place them on a conveyor belt and the other robotic arm at the end of the conveyor belt will pick these objects from the conveyor and place them into bins. Each bin represents a destination for the package. As packages are sent out from the warehouse, there will also be alerts sent to the user via email notifying them about the package being shipped from the warehouse.The packages to be delivered have their own priorities.The team which sorts high-priority packages based on their destination in the least time with minimum penalties wins.


# Sections

### Robotic Manipulation: Specified in [node_control_ur5_1.py](https://github.com/MishaelThomas/eYRC-2020-21/blob/main/pkg_task5/scripts/node_control_ur5_1.py) and [node_control_ur5_2_and_belt.py](https://github.com/MishaelThomas/eYRC-2020-21/blob/main/pkg_task5/scripts/node_control_ur5_2_and_belt.py)

Moveit was used as a framework for motion planning of robots.Threading was used to control UR5 arm and conveyer belt simultaneoulsy to decrease the sorting time. An efficient sorting algorithm was implemented to sort the packages according to thier priority.(In the given task priority depends on colour of the package).

### Robotic Perception : Specified in [node_package_color_detector.py](https://github.com/MishaelThomas/eYRC-2020-21/blob/main/pkg_task5/scripts/node_package_color_detector.py)

Image processing algorithms were used to determine colour and location of packages from image of the shelf.Image was converted into HSV format to make colour detection insensitive to variation in light. We did not use QR code to find these information about the packages as QR code is sensitive to variation in light.



### IOT: Specified in [node_ros_iot_bridge.py](https://github.com/MishaelThomas/eYRC-2020-21/blob/main/pkg_ros_iot_bridge/scripts/node_ros_iot_bridge.py),[iot.py](https://github.com/MishaelThomas/eYRC-2020-21/tree/main/pkg_ros_iot_bridge/scripts/pyiot) and [node_update_spreadsheet.py](https://github.com/MishaelThomas/eYRC-2020-21/blob/main/pkg_task5/scripts/node_update_spreadsheets.py)

MQTT protocol was used by warehouse to recieve online orders.HTTPS protocol was used by warehouse to update Inventory Management spreadsheet and to send notification to customers about status of ordered package.

### Dashboard 

Dashboard was used to show live status of warehouse. It was implemented using AJAX ,Javascript and HTML.

### Google App Scripting

Data recieved from warehouse through HTTPS protocol was updated in the Google spreadsheets using Google Apps Scripting. It was also used to send mail notification to customers.

# Technology Stack:

* Python
* Javascript
* HTML
* ROS
* Gazebo
* MQTT
* OpenCV
* MoveIt











