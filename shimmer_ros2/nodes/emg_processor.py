#!/usr/bin/env python

import collections
import copy
import itertools

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
        # Compute window sizes, round FFT window to nearest even number
        self.window_size_fft: int = round(
            self.params['window_size_fft_relative'] * self.params['sampling_rate']  # type: ignore
        )
        self.window_size_fft += self.window_size_fft % 2  # type: ignore
        self.window_size_ema: int = round(
            self.params['window_size_ema_relative'] * self.params['sampling_rate']  # type: ignore
        )
        self.ema_decay: float = self.params['ema_decay']  # type: ignore
        # Create EMA window using decay factor
        self.window_ema = np.array(
            [(1 - self.ema_decay) ** i for i in range(self.window_size_ema)],
            dtype=np.float32,
        )
        self.window_ema /= np.sum(self.window_ema)

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

        # Initialize empty lists and msgs for FFT
        self.emg_ch1: collections.deque = collections.deque()
        self.emg_ch2: collections.deque = collections.deque()
        self.timestamp: collections.deque = collections.deque()
        self.frame_id: str = ''
        self.fft_msg: Fft = Fft()
        # Initialize empty lists and msgs for processed EMG - more than 1 batch is
        # stored to allow for sliding window smoothing
        self.processed_emg_msg: Emg = Emg()
        self.emg_ch1_ifft_hist: list[np.ndarray] = []
        self.emg_ch2_ifft_hist: list[np.ndarray] = []
        self.timestamp_hist: list[collections.deque] = []

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
        # Timestamp of data
        self.timestamp.append(emg_msg.header.stamp)
        if len(self.emg_ch1) >= self.window_size_fft:
            # Perform FFT, filtering and iFFT when enough data is collected
            self.fft_filter_publish()
            # Append iFFT data and orig_timestamp to history for smoothing
            self.emg_ch1_ifft_hist.append(np.asarray(self.fft_msg.ifft_emg_ch1))
            self.emg_ch2_ifft_hist.append(np.asarray(self.fft_msg.ifft_emg_ch2))
            self.timestamp_hist.append(
                copy.deepcopy(self.fft_msg.orig_timestamp.copy())
            )
            # Clean EMG and timestamp data lists
            for data_queue in (self.emg_ch1, self.emg_ch2, self.timestamp):
                data_queue.clear()
            if len(self.timestamp_hist) == 2:
                # Append window_size-1 samples from second last IFFT sample to last
                # to enable continuous windowing
                self.rectify_smooth_publish(
                    emg_ch1_extend_batch=np.fromiter(
                        itertools.chain(
                            self.emg_ch1_ifft_hist[0][-self.window_size_ema + 1 :],
                            self.emg_ch1_ifft_hist[1],
                        ),
                        dtype=np.float32,
                    ),
                    emg_ch2_extend_batch=np.fromiter(
                        itertools.chain(
                            self.emg_ch2_ifft_hist[0][-self.window_size_ema + 1 :],
                            self.emg_ch2_ifft_hist[1],
                        ),
                        dtype=np.float32,
                    ),
                    # Only keep timestamp from last samples
                    timestamp=self.timestamp_hist[1],
                )
                # Delete first list elements to move last batch to first position
                del (
                    self.emg_ch1_ifft_hist[0],
                    self.emg_ch2_ifft_hist[0],
                    self.timestamp_hist[0],
                )

    def fft_filter_publish(self) -> None:
        """Perform FFT, filtering, iFFT, rectify, smooth and publish processed EMG."""
        # Compute fft for each channel
        emg_ch1_fft: np.ndarray = scipy.fft.rfft(self.emg_ch1)
        emg_ch2_fft: np.ndarray = scipy.fft.rfft(self.emg_ch2)
        # Calculate frequencies
        freqs: np.ndarray = scipy.fft.rfftfreq(
            n=len(self.emg_ch1), d=1.0 / self.sampling_rate
        )
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
        emg_ch1_ifft: np.ndarray = scipy.fft.irfft(emg_ch1_fft)
        emg_ch2_ifft: np.ndarray = scipy.fft.irfft(emg_ch2_fft)

        # Populate ROS msg with FFT and iFFT data and publish
        if not self.fft_msg.header.frame_id:
            self.fft_msg.header.frame_id = self.frame_id
        # Frequencies and original timestamp
        self.fft_msg.fft_freqs = freqs.tolist()
        self.fft_msg.orig_timestamp = self.timestamp
        # Keep last timestamp only for header
        self.fft_msg.header.stamp = self.timestamp[-1]
        # Channel 1
        self.fft_msg.amp_emg_ch1 = np.abs(emg_ch1_fft).tolist()
        self.fft_msg.ifft_emg_ch1 = emg_ch1_ifft.tolist()
        # Channel 2
        self.fft_msg.amp_emg_ch2 = np.abs(emg_ch2_fft).tolist()
        self.fft_msg.ifft_emg_ch2 = emg_ch2_ifft.tolist()

        if rclpy.ok():
            self.publisher_fft.publish(self.fft_msg)

    def rectify_smooth_publish(
        self,
        emg_ch1_extend_batch: np.ndarray,
        emg_ch2_extend_batch: np.ndarray,
        timestamp: collections.deque,
    ) -> None:
        """Rectify and smooth EMG signal and publish to emg_processed topic.

        Args:
            emg_ch1_extend_batch (np.ndarray): Extended batch with EMG channel 1 data.
            emg_ch2_extend_batch (np.ndarray): Extended batch with EMG channel 2 data.
            timestamp (collections.deque): Timestamps (not extended) corresponding to
                the EMG data.
        """
        # Initialize processed_emg for both channels
        processed_emg_ch1: np.ndarray = emg_ch1_extend_batch
        processed_emg_ch2: np.ndarray = emg_ch2_extend_batch
        # Rectify signal
        if self.params['rectify']:
            processed_emg_ch1 = np.abs(emg_ch1_extend_batch, dtype=np.float32)
            processed_emg_ch2 = np.abs(emg_ch2_extend_batch, dtype=np.float32)

        # Create 2D array with windowed signal for smoothing
        if self.params['smooth']:
            windowed_emg_ch1: np.ndarray = rolling_window(
                processed_emg_ch1, self.window_size_ema
            )
            windowed_emg_ch2: np.ndarray = rolling_window(
                processed_emg_ch2, self.window_size_ema
            )
            # Smooth signal using EMA
            processed_emg_ch1 = np.average(
                windowed_emg_ch1, axis=-1, weights=self.window_ema
            )
            processed_emg_ch2 = np.average(
                windowed_emg_ch2, axis=-1, weights=self.window_ema
            )

        # Populate ROS EMG msg with clean EMG data and publish
        if not self.processed_emg_msg.header.frame_id:
            self.processed_emg_msg.header.frame_id = self.frame_id
        if rclpy.ok():
            # Convert processed_emg to list and publish each sample with its timestamp
            processed_emg_ch1 = processed_emg_ch1.astype(np.float32).tolist()
            processed_emg_ch2 = processed_emg_ch2.astype(np.float32).tolist()
            for i in range(len(timestamp)):
                self.processed_emg_msg.header.stamp = timestamp[i]
                if self.params['smooth']:
                    # Smoothing reduces length of output by window_size-1 (discard
                    # extended samples)
                    self.processed_emg_msg.emg_ch1 = processed_emg_ch1[i]
                    self.processed_emg_msg.emg_ch2 = processed_emg_ch2[i]
                else:
                    # Without smoothing, just discard first window_size-1 samples
                    self.processed_emg_msg.emg_ch1 = processed_emg_ch1[
                        self.window_size_ema - 1 + i
                    ]
                    self.processed_emg_msg.emg_ch2 = processed_emg_ch2[
                        self.window_size_ema - 1 + i
                    ]
                self.publisher_processed.publish(self.processed_emg_msg)


def rolling_window(emg_arr: np.ndarray, window_size: int) -> np.ndarray:
    """Apply rolling window to 1D numpy array and return with shape without copying.
    [length - window_size + 1, window_size]

    Args:
        emg_arr (np.array): emg signal to apply rolling window on
        window (int): size of the window

    Returns:
        2D np.array: array with window_size in each row
    """
    shape: tuple = emg_arr.shape[:-1] + (
        emg_arr.shape[-1] - window_size + 1,
        window_size,
    )
    strides: tuple = emg_arr.strides + (emg_arr.strides[-1],)
    return np.lib.stride_tricks.as_strided(emg_arr, shape=shape, strides=strides)


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
