#!/usr/bin/env python
## Configure and capture EMG signal from Shimmer3 device.
## Stream acquired emg signal from both channels to the topic emg_stream.

import sys

import rospy
import serial
from emg_grip_interfaces.msg import Emg

from shimmer_ros2.shimmer_api.emg import ShimmerEMG


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
