# shimmer_ros2

A ROS 2 interface for Shimmer3 EMG sensors. Data can be streamed from multiple Shimmer sensors simultaneously. Synchronization is performed using `rospy.Time.now()` and the internal Shimmer 32 kHz timer.

## Prerequisites
- Turn on your Shimmer3 sensor(s) and ensure they are discoverable via Bluetooth.
- Add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

## Binding Shimmer sensors to RFCOMM

1. Find the MAC addresses of your Shimmer devices:

```bash
hcitool scan
```

2. Run the binding script as root, providing all Shimmer MAC addresses as positional arguments:

```bash
sudo ros2 run shimmer_ros2 bind_shimmer_to_rfcomm.sh <MAC1> [<MAC2> ...]
```

## Streaming EMG data

1. Edit the Shimmer configuration in `config/shimmer_config.yaml`.
2. Launch the EMG publisher (replace `<num>` with the number of sensors):

```bash
ros2 launch shimmer_ros2 publish_emg_data.launch num:=<num>
```

## Visualizing data

Use PlotJuggler to visualize streamed EMG data (configuration in `config/plotjuggler_config.xml`):

```bash
ros2 launch shimmer_ros2 plotjuggler_shimmer.launch
```
