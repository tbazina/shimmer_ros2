#!/usr/bin/env python
## Configure and capture EMG signal from Shimmer3 device.
## Stream acquired emg signal from both channels to the topic emg_stream.

import sys

import rclpy  # type: ignore
import rospy
import serial
from emg_grip_interfaces.msg import Emg
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore

from shimmer_ros2.shimmer_api.emg import ShimmerEMG


class EMGPublisher(Node):
    def __init__(self) -> None:
        super().__init__('emg_publisher')
        self.param_defaults: dict[str, float | int] = {
            'rfcomm_port_num': 0,  # rfcomm port Shimmer MAC adress is bound to
            'gain': 12,
            'emg_data_rate': 125,  # Hz, will be auto set to > sampling_rate
            'sampling_rate': 992.96969699,  # Hz
            'stream_test_signal': False,  #
            'calibrate_test_signal': 4,  # Calibration duration in seconds (0 - no calibration)
            'queue_size': 10,
        }
        # Declare all parameters into a single dict
        self.params: dict[str, float | int] = {
            key: self.declare_parameter(key, default_value).value
            for key, default_value in self.param_defaults.items()
        }
        # Create publisher
        self.publisher_ = self.create_publisher(
            msg_type=Emg,
            topic='emg_stream',
            qos_profile=QoSProfile(depth=self.params['queue_size']),
        )
        # Initialize Shimmer sensor
        self.shimmer = ShimmerEMG(
            port=f'/dev/rfcomm{self.params["rfcomm_port_num"]}',
            node_logger=self.get_logger(),
            node_clock=self.get_clock(),
        )
        # Open and set up Shimmer device
        self.shimmer_emg = self.shimmer.__enter__()
        try:
            self.setup_shimmer()
        except Exception as e:
            self.get_logger().error(f'Failed to set up Shimmer: {e}')
        self.timer = self.create_timer(0.1, self.timer_callback)

    def setup_shimmer(self):
        self.shimmer_emg.get_shimmer_name(echo=True)
        self.shimmer_emg.get_id_and_rev(echo=True)
        self.shimmer_emg.get_charge_status_led(echo=True)
        self.shimmer_emg.get_buffer_size(echo=True)
        self.shimmer_emg.synchronise_system_time(echo=True)
        self.shimmer_emg.set_sampling_rate(sampling_rate=self.params['sampling_rate'])
        self.shimmer_emg.set_emg_data_rate(emg_data_rate=self.params['emg_data_rate'])
        self.shimmer_emg.set_emg_gain(gain=self.params['gain'])
        # Activate EMG sensors
        self.shimmer_emg.activate_emg_sensors()
        # Power down chip 2 and print register settings
        self.shimmer_emg.power_down_chip_2()
        self.shimmer_emg.get_emg_registers(chip_num=1, echo=True)
        # Calibrate using test signal and store calibration parameters
        self.shimmer_emg.calibrate_test_signal(
            duration=self.params['calibrate_test_signal'], echo=True
        )

    def timer_callback(self):
        msg = Emg()
        # Fill in the message fields
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EMGPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


def publish_emg():
    """
    Configure and capture EMG signal from Shimmer3 device.
    Stream acquired emg signal from both channels to the topic emg_stream.

    ROS parameters:
      rfcomm_num (int): rfcomm port Shimmer MAC adress is bound to
      gain (int): Chip gain
      emg_data_rate (int): Internal chip data (sampling) rate in Hz
      sampling_rate (int): Shimmer sampling rate in Hz
      stream_test_signal (bool): Stream test signal (1 Hz square wave, +-1 amplitude)
      calibrate_test_signal (bool): Use test signal to calibrate ADC offset
      queue_size (int): Size of the queue for asynchronous publishing on topic
    """
    # Initialize node and Publisher
    rospy.init_node(
        'shimmer_publisher',
        anonymous=False,
        log_level=rospy.DEBUG,
    )
    try:
        # Get shimmer parameters
        serial_port = f'/dev/rfcomm{rospy.get_param("rfcomm_port")}'
        gain = rospy.get_param('gain')
        emg_data_rate = rospy.get_param('emg_data_rate', 1)  # Hz
        sampling_rate = rospy.get_param('sampling_rate', 50)  # Hz
        stream_test_signal = rospy.get_param('stream_test_signal', False)
        calibrate_test_signal = rospy.get_param('calibrate_test_signal')
        queue_size = rospy.get_param('queue_size', 10)
        # Initialize publisher with queue size
        pub = rospy.Publisher('emg_stream', Emg, queue_size=queue_size)
        # Initialize Shimmer
        Shimmer = ShimmerEMG(serial_port)
        with Shimmer as sh:
            # Display and set configuration parameters
            Shimmer.get_shimmer_name(echo=True)
            Shimmer.get_id_and_rev(echo=True)
            Shimmer.get_charge_status_led(echo=True)
            Shimmer.get_buffer_size(echo=True)
            Shimmer.synchronise_system_time(echo=True)
            Shimmer.set_sampling_rate(sampling_rate=sampling_rate)
            Shimmer.set_emg_data_rate(emg_data_rate=emg_data_rate)
            Shimmer.set_emg_gain(gain=gain)
            # Activate EMG sensors
            Shimmer.activate_emg_sensors()
            # Power down chip 2 and print register settings
            Shimmer.power_down_chip_2()
            Shimmer.get_emg_registers(chip_num=1, echo=True)
            # Calibrate using test signal and store calibration parameters
            Shimmer.calibrate_test_signal(duration=calibrate_test_signal, echo=True)
            # Start streaming signal to topic
            Shimmer.start_streaming_EMG(
                publisher=pub,
                test_signal=stream_test_signal,
                echo=True,
            )
    except serial.SerialTimeoutException:
        sys.exit('Write timeout exceeded!')
    except serial.SerialException:
        sys.exit('The device not found - serial port closed! (check serial_port)')
    except rospy.ROSInterruptException:
        rospy.logwarn('User interrupted execution!')
    except rospy.ROSException:
        print('Could not get parameter names!')
