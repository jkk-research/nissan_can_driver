# Nissan CAN driver

ROS2 CAN driver for nissan leaf. Requires a kvaser device and the [kvaser_interface](https://github.com/jkk-research/kvaser_interface) package.

## Usage

```
ros2 launch nissan_can_driver can_driver.launch.py
```

### Launch parameters
| Name                   | Description                                                | Default value |
|------------------------|------------------------------------------------------------|---------------|
| kvaser_hardware_id     | Hardware ID of the kvaser device                           | 11162         |
| autoware_control_input | Whether to use autoware or standard control command inputs | true          |
