#!/usr/bin/env python

import collections

import numpy as np
import rclpy  # type: ignore
import scipy.fft
from emg_grip_interfaces.msg import Emg, Fft  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.publisher import Publisher  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore


class EMGProcessor(Node):
    def __init__(self) -> None:
        """Read raw EMG from emg_stream topic, process it and republish to two topics:
        1) emg_fft topic with FFT and iFFT data
        2) emg_processed topic with processed EMG (rectified and smoothed)
        """
        super().__init__('emg_processor')
        self.param_defaults: dict[str, float | int | bool | list] = {
            'sampling_rate': 992.96969699,  # Hz
            'window_size_fft_relative': 0.5,  # fraction of sampling rate
            'window_size_ema_relative': 0.304138,  # fraction of sampling rate
            'band_stop': [
                0.0,
                0.01,
            ],  # list of low, high ranges [low, high, low, high ...]
            'hpf_cutoff': 0.1,  # Hz, high pass filter cutoff frequency
            'lpf_cutoff': 203.0,  # Hz, low pass filter cutoff frequency
            'fft_mask': [
                0.0,
                0.01,
                0.0,
            ],  # list of low, high, multiplier [low, high, multiplier,...]
            'rectify': True,  # rectify signal after iFFT
            'smooth': True,  # smooth signal after rectification
            'ema_decay': 2.5e-4,  # EMA decay factor for smoothing
            'queue_size': 10,  # queue size for FFT and smoothing
        }
        # TODO: get sampling__rate from shimmer publisher topic
        # Declare all parameters into a single dict
        self.params: dict[str, float | int | bool | list] = {
            key: self.declare_parameter(key, default_value).value
            for key, default_value in self.param_defaults.items()
        }
        # Compute window sizes and round FFT window to nearest even number
        self.window_size_fft: int = round(
            self.params['window_size_fft_relative'] * self.params['sampling_rate']  # type: ignore
        )
        self.window_size_ema: int = round(
            self.params['window_size_ema_relative'] * self.params['sampling_rate']  # type: ignore
        )
        self.window_size_fft += self.window_size_fft % 2  # type: ignore
        # If band_stop is not empty, convert to list of lists
        # [[low, high], [low, high], ...]
        if self.params['band_stop']:
            self.band_stop: list[list[float]] = [
                self.params['band_stop'][i : i + 2]  # type: ignore
                for i in range(0, len(self.params['band_stop']), 2)  # type: ignore
            ]
        # If fft_mask is not empty, convert to list of lists
        # [[low, high, multiplier], [low, high, multiplier], ...]
        if self.params['fft_mask']:
            self.fft_mask: list[list[float]] = [
                self.params['fft_mask'][i : i + 3]  # type: ignore
                for i in range(0, len(self.params['fft_mask']), 3)  # type: ignore
            ]
        # Assign filter parameters to variables
        self.sampling_rate: float = self.params['sampling_rate']  # type: ignore
        self.hpf_cutoff: float = self.params['hpf_cutoff']  # type: ignore
        self.lpf_cutoff: float = self.params['lpf_cutoff']  # type: ignore
        self.ema_decay: float = self.params['ema_decay']  # type: ignore
        self.get_logger().info(f'Sampling rate: {self.sampling_rate}')
        self.get_logger().info(f'FFT window size: {self.window_size_fft}')
        self.get_logger().info(f'EMA window size: {self.window_size_ema}')
        if self.params['band_stop']:
            self.get_logger().info(f'Band-stop frequencies: {self.band_stop}')
        self.get_logger().info(f'High pass cutoff frequency: {self.hpf_cutoff}')
        self.get_logger().info(f'Low pass cutoff frequency: {self.lpf_cutoff}')
        if self.params['fft_mask']:
            self.get_logger().info(f'FFT custom mask length: {len(self.fft_mask)}')
        self.get_logger().info(f'Rectify signal: {self.params["rectify"]}')
        self.get_logger().info(f'Smooth signal: {self.params["smooth"]}')
        self.get_logger().info(f'EMA decay factor: {self.ema_decay}')
        self.get_logger().info(f'Queue size: {self.params["queue_size"]}')

        # Initialize empty lists and msgs
        self.emg_ch1: collections.deque = collections.deque()
        self.emg_ch2: collections.deque = collections.deque()
        self.timestamp: collections.deque = collections.deque()
        self.frame_id: str = ''
        self.fft_msg: Fft = Fft()

        # Create publishers
        self.publisher_fft: Publisher = self.create_publisher(
            msg_type=Fft,
            topic='emg_fft',
            qos_profile=QoSProfile(depth=self.params['queue_size']),
        )
        self.publisher_processed: Publisher = self.create_publisher(
            msg_type=Emg,
            topic='emg_processed',
            qos_profile=QoSProfile(depth=self.params['queue_size']),
        )

        # Create subscriber to raw EMG topic
        self.subscription = self.create_subscription(
            msg_type=Emg,
            topic='emg_stream',
            callback=self.process_raw_emg_msgs,
            qos_profile=QoSProfile(depth=self.params['queue_size']),
        )
        self.get_logger().info('Processing EMG data. Press Ctrl+C to stop ...')

    def process_raw_emg_msgs(self, emg_msg) -> None:
        """Process raw EMG messages.

        Args:
            emg_data (Emg): The raw EMG data message.
        """
        # Acquire shimmer name only from first message
        if not self.frame_id:
            self.frame_id = emg_msg.header.frame_id
        # Append EMG data to channel lists
        self.emg_ch1.append(emg_msg.emg_ch1)
        self.emg_ch2.append(emg_msg.emg_ch2)
        # Timestamp of data (keep only last one)
        self.timestamp.append(emg_msg.header.stamp)
        if len(self.emg_ch1) >= self.window_size_fft:
            # Perform FFT, filtering and iFFT when enough data is collected
            self.fft_filter_publish()
            # Clean EMG and timestamp data lists
            for data_queue in (self.emg_ch1, self.emg_ch2, self.timestamp):
                data_queue.clear()

    def fft_filter_publish(self) -> None:
        """Perform FFT, filtering, iFFT, rectify, smooth and publish processed EMG."""
        # Compute fft for each channel
        emg_ch1_fft = scipy.fft.rfft(self.emg_ch1)
        emg_ch2_fft = scipy.fft.rfft(self.emg_ch2)
        # Calculate frequencies
        freqs = scipy.fft.rfftfreq(n=len(self.emg_ch1), d=1.0 / self.sampling_rate)
        # High-pass filtering
        if self.hpf_cutoff:
            emg_ch1_fft[freqs <= self.hpf_cutoff] = 0.0
            emg_ch2_fft[freqs <= self.hpf_cutoff] = 0.0
        # Low-pass filtering
        if self.lpf_cutoff:
            emg_ch1_fft[freqs >= self.lpf_cutoff] = 0.0
            emg_ch2_fft[freqs >= self.lpf_cutoff] = 0.0
        # Filter out band stop frequencies
        if self.params['band_stop']:
            for band in self.band_stop:
                emg_ch1_fft[np.logical_and(freqs >= band[0], freqs <= band[1])] = 0.0
                emg_ch2_fft[np.logical_and(freqs >= band[0], freqs <= band[1])] = 0.0
        # Multiply frequency ranges with custom provided multiplier
        if self.params['fft_mask']:
            for mask in self.fft_mask:
                emg_ch1_fft[np.logical_and(freqs >= mask[0], freqs <= mask[1])] *= mask[
                    2
                ]
                emg_ch2_fft[np.logical_and(freqs >= mask[0], freqs <= mask[1])] *= mask[
                    2
                ]

        # Compute inverse FFT to reconstruct filtered signal
        emg_ch1_ifft = scipy.fft.irfft(emg_ch1_fft)
        emg_ch2_ifft = scipy.fft.irfft(emg_ch2_fft)
        # rospy.logdebug(f'IFFT len: {emg_ch1_ifft.shape}')
        # rospy.logdebug(f'Windows size: {self.window_size}')

        # Populate ROS msg with FFT and iFFT data and publish
        if not self.fft_msg.header.frame_id:
            self.fft_msg.header.frame_id = self.frame_id
        # Frequencies and original timestamp
        self.fft_msg.fft_freqs = freqs.tolist()
        self.fft_msg.orig_timestamp = self.timestamp
        self.fft_msg.header.stamp = self.timestamp.pop()
        # Channel 1
        self.fft_msg.amp_emg_ch1 = np.abs(emg_ch1_fft).tolist()
        self.fft_msg.ifft_emg_ch1 = emg_ch1_ifft.tolist()
        # Channel 2
        self.fft_msg.amp_emg_ch2 = np.abs(emg_ch2_fft).tolist()
        self.fft_msg.ifft_emg_ch2 = emg_ch2_ifft.tolist()

        self.publisher_fft.publish(self.fft_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: EMGProcessor = EMGProcessor()
    try:
        # node.initialize_shimmer_start_streaming_emg()
        # Set to run destroy_node on shutdown
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
