#!/usr/bin/env python
## Configure and capture EMG signal from Shimmer3 device.
## Stream acquired emg signal from both channels to the topic emg_stream.

import sys
import traceback

import rclpy  # type: ignore
import serial
from emg_grip_interfaces.msg import Emg  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore

from shimmer_ros2.shimmer_api.emg import ShimmerEMG


class EMGPublisher(Node):
    def __init__(self) -> None:
        """
        Configure and capture EMG signal from Shimmer3 device.
        Stream acquired emg signal from both channels to the topic emg_stream.

        ROS parameters:
        rfcomm_port_num (int): rfcomm port Shimmer MAC adress is bound to
        gain (int): Chip gain
        emg_data_rate (int): Internal chip data (sampling) rate in Hz
        sampling_rate (float): Shimmer sampling rate in Hz
        stream_test_signal (bool): Stream test signal (1 Hz square wave, +-1 amplitude)
        zero_test_signal (bool): Use test signal to zero ADC offset
        queue_size (int): Size of the queue for asynchronous publishing on topic
        """
        super().__init__('emg_publisher')
        self.param_defaults: dict[str, float | int] = {
            'rfcomm_port_num': 0,  # rfcomm port Shimmer MAC adress is bound to
            'gain': 12,
            'emg_data_rate': 1,  # Hz, will be auto set to > sampling_rate
            'sampling_rate': 992.96969699,  # Hz
            'stream_test_signal': False,  #
            'zero_test_signal': 4,  # Zeroing duration in seconds (0 - no calibration)
            'queue_size': 10,
        }
        # Declare all parameters into a single dict
        self.params: dict[str, float | int] = {
            key: self.declare_parameter(key, default_value).value
            for key, default_value in self.param_defaults.items()
        }
        # Create publisher
        self.publisher = self.create_publisher(
            msg_type=Emg,
            topic='emg_stream',
            qos_profile=QoSProfile(depth=self.params['queue_size']),
        )

    def initialize_shimmer_start_streaming_emg(self):
        # Initialize Shimmer sensor
        try:
            self.shimmer = ShimmerEMG(
                port=f'/dev/rfcomm{self.params["rfcomm_port_num"]}',
                node_logger=self.get_logger(),
                node_clock=self.get_clock(),
            )
            # Open and set up Shimmer device
            self.shimmer_emg = self.shimmer.__enter__()
            self.setup_shimmer()
            self.get_logger().info('Publishing EMG data. Press Ctrl+C to stop ...')
            # Create timer using period from sampling_rate
            self.timer = self.create_timer(
                timer_period_sec=1.0 / self.params['sampling_rate'],
                callback=self.publish_sensor_value,
            )
        except serial.SerialTimeoutException:
            sys.exit('Write timeout exceeded!')
        except serial.SerialException:
            sys.exit('The device not found - serial port closed! (check serial_port)')
        except (KeyboardInterrupt, ExternalShutdownException):
            self.get_logger().warn('User interrupted execution!')
        except Exception as e:
            self.get_logger().error(f'Failed to set up Shimmer: {e}')
            self.get_logger().debug(traceback.format_exc())

    def setup_shimmer(self):
        self.shimmer_emg.get_shimmer_name(echo=True)
        self.shimmer_emg.get_id_and_rev(echo=True)
        self.shimmer_emg.get_charge_status_led(echo=True)
        self.shimmer_emg.get_buffer_size(echo=True)
        self.shimmer_emg.synchronize_system_time(echo=True)
        self.shimmer_emg.set_sampling_rate(sampling_rate=self.params['sampling_rate'])
        self.shimmer_emg.set_emg_data_rate(emg_data_rate=self.params['emg_data_rate'])
        self.shimmer_emg.set_emg_gain(gain=self.params['gain'])
        # Activate EMG sensors
        self.shimmer_emg.activate_emg_sensors()
        # Power down chip 2 and print register settings
        self.shimmer_emg.power_down_chip_2()
        self.shimmer_emg.get_emg_registers(chip_num=1, echo=True)
        # zero using test signal and store calibration parameters
        self.shimmer_emg.zero_test_signal(
            duration=self.params['zero_test_signal'], echo=True
        )
        self.shimmer_emg.start_streaming_emg(
            publisher=self.publisher,
            test_signal=self.params['stream_test_signal'],
            echo=True,
        )

    def publish_sensor_value(self):
        # Read data from Shimmer and publish
        self.shimmer_emg.read_publish_single_point()

    def destroy_node(self):
        """
        Called exactly once, from any thread, when SIGINT or launch
        shutdown occurs. Cancel timers first, then stop Shimmer, then
        close the serial port. Do NOT use ROS logging / publishers after
        rclpy is shut down.
        """
        # Use print() in exceptions because logger may already be invalid
        # Cancel the publish timer so no more callbacks run
        if hasattr(self, 'timer'):
            print('Cancelling publish timer …', file=sys.stdout)
            self.timer.cancel()

        # Destroy publisher
        try:
            print('Destroying publisher …', file=sys.stdout)
            self.destroy_publisher(self.publisher)
        except Exception as e:
            print(f'Error destroying publisher: {e}', file=sys.stderr)

        # Stop streaming and close Shimmer
        try:
            print('Stopping Shimmer EMG stream …', file=sys.stdout)
            self.shimmer_emg.stop_streaming()
        except Exception as e:
            print(f'Error stopping streaming: {e}', file=sys.stderr)

        try:
            print('Closing Shimmer port …', file=sys.stdout)
            self.shimmer.__exit__(None, None, None)
        except Exception as e:
            print(f'Error closing Shimmer port: {e}', file=sys.stderr)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: EMGPublisher = EMGPublisher()
    try:
        node.initialize_shimmer_start_streaming_emg()
        # Set to run destroy_node on shutdown
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
