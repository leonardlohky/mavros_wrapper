# ROS MQTT Bridge

Mqtt Communication Interface for ROS

The MqttHandler class provides MQTT message publishing and subscription capabilities. 

Class Methods:
- subscribe('topic_name', callback_function)
- publish(string_message)

Constructor:
- MqttHandler(name, broker='localhost', port=1883, log=False)



### Dependencies
* paho-mqtt
```bash
    pip install paho-mqtt
```


Compile and run the package

1. Open a terminal and execute following command to run the ROS node
    ```bash
       roslaunch ros_mqtt_bridge ros_mqtt_bridge.launch
    ```
2. Open another terminal and listen to the ROS node output
    ```bash
        rostopic echo /point_publisher
    ```
3. Open another terminal and publish an MQTT message
    ```bash
        mosquitto_pub -t 'myTopic' -m '{"x": 1.0, "y": 2.0, "z": 4.0}'
    ```
