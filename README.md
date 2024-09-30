# bsactuator_ros

This is a ROS2 package developed by RoboSapiens for interfacing with the Bambooshoot Actuator.

## Prerequisites

You need to install the `bsactuator` Python library to interact with the hardware.

```bash
pip3 install git+https://github.com/rb-sapiens/bsactuator.git
```

Additionally, you need to place a `udev` file in `/etc/udev/rules.d/` to ensure USB device recognition.

```bash
cd bsactuator_ros
sudo mv bambooshoot_actuator.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Launching the Node

To run the `bsactuator_ros` node, use the following command:

```bash
ros2 launch bsactuator bsactuator_launch.py
```

## Topics

### Subscribers

#### `/bsactuator/set_length`

This topic is used to specify the desired actuator length. The message type is `std_msgs/msg/Int16`.

Example usage:

```bash
ros2 topic pub /bsactuator/set_length std_msgs/msg/Int16 "{data: 300}"
```

#### `/bsactuator/hold`

This topic is used to hold the actuator in place. The message type is `std_msgs/msg/Bool`.

Example usage:

```bash
ros2 topic pub /bsactuator/hold std_msgs/msg/Bool "{data: true}"
```

#### `/bsactuator/release`

This topic is used to release the actuator. The message type is `std_msgs/msg/Bool`.

Example usage:

```bash
ros2 topic pub /bsactuator/release std_msgs/msg/Bool "{data: true}"
```

#### `/bsactuator/reset`

This topic is used to reset the actuator. The message type is `std_msgs/msg/Bool`.

Example usage:

```bash
ros2 topic pub /bsactuator/reset std_msgs/msg/Bool "{data: true}"
```

#### `/bsactuator/stop`

This topic is used to stop the actuator. The message type is `std_msgs/msg/Bool`.

Example usage:

```bash
ros2 topic pub /bsactuator/stop std_msgs/msg/Bool "{data: true}"
```

### Publishers

#### `/bsactuator/length`

Publishes the current length of the actuator in millimeters. The message type is `std_msgs/msg/Int16`.

#### `/bsactuator/status`

Publishes a status message when the actuator reaches the desired length. The message type is `std_msgs/msg/String`.

Example message:

```bash
"Goal reached successfully."
```
