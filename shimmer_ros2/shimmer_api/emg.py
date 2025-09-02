# /usr/bin/python3

import struct
import sys
from collections import defaultdict, deque
from datetime import datetime

import rospy
import serial
from emg_grip_interfaces.msg import Emg


class ShimmerEMG:
    def __init__(self, port) -> None:
        # Serial port
        self.port = port

        # // Packet Types// Packet Types
        self.packet_type = {
            'ACK_COMMAND_PROCESSED': 0xFF,
            'GET_SAMPLING_RATE_COMMAND': 0x03,
            'SET_SAMPLING_RATE_COMMAND': 0x05,
            'INQUIRY_COMMAND': 0x01,
            'GET_BUFFER_SIZE_COMMAND': 0x36,
            'BUFFER_SIZE_RESPONSE': 0x35,
            'SET_INFOMEM_COMMAND': 0x8C,
            'GET_INFOMEM_COMMAND': 0x8E,
            'GET_CHARGE_STATUS_LED_COMMAND': 0x32,
            'CHARGE_STATUS_LED_RESPONSE': 0x31,
            'SET_SENSORS_COMMAND': 0x08,
            'SET_EXG_REGS_COMMAND': 0x61,
            'GET_DAUGHTER_CARD_ID_COMMAND': 0x66,
            'DAUGHTER_CARD_ID_RESPONSE': 0x65,
            'GET_EXG_REGS_COMMAND': 0x63,
            'EXG_REGS_RESPONSE': 0x62,
            'GET_SHIMMERNAME_COMMAND': 0x7B,
            'SHIMMERNAME_RESPONSE': 0x7A,
            'START_STREAMING_COMMAND': 0x07,
            'STOP_STREAMING_COMMAND': 0x20,
            'DATA_PACKET': 0x00,
            'SET_RWC_COMMAND': 0x8F,
            'RWC_RESPONSE': 0x90,
            'GET_RWC_COMMAND': 0x91,
            'SET_CONFIGTIME_COMMAND': 0x85,
            'CONFIGTIME_RESPONSE': 0x86,
            'GET_CONFIGTIME_COMMAND': 0x87,
            'SET_SHIMMERNAME_COMMAND': 0x79,
            'SHIMMERNAME_RESPONSE': 0x7A,
            'GET_SHIMMERNAME_COMMAND': 0x7B,
        }

        # EMG gain configuration bytes
        self.emg_gain_config = defaultdict(
            lambda: 0b110,  # default 12
            {1: 0b001, 2: 0b010, 3: 0b011, 4: 0b100, 6: 0b000, 8: 0b101, 12: 0b110},
        )
        # Default gain value
        self.emg_gain_config_default = 12

        # EMG multiplexer settings for both channels
        self.emg_mux_config = defaultdict(
            lambda: 0b1001,  # Route INxP and INxN to channel x inputs
            {
                'normal': 0b0000,  # Normal electrode input (default)
                'shorted': 0b0001,  # Input shorted (for offset measurements and power down channel)
                'test': 0b0101,  # Test signal
                'measure_EMG': 0b1001,  # Route INxP and INxN to channel x inputs
            },
        )

        # EMG data rate configuration bytes
        self.emg_data_rate_config = defaultdict(
            lambda: 0x02,  # 500 Hz
            {
                125: 0x00,
                250: 0x01,
                500: 0x02,
                1000: 0x03,
                2000: 0x04,
                4000: 0x05,
            },
        )

        # EMG Right-Leg Drive (Common-mode Rejection) settings
        # RLD2N, RLD2P, RLD1N and RLD1P should be 0 for both chips
        self.emg_rld_config = 0b0000

        # PDB_RLD: RLD buffer power bit determines the RLD buffer power state
        # 0 = powered down (default), 1 = enabled
        # RLD_LOFF_SENSE: RLD lead-off sense function
        # 0 = RLD lead-off sense is disabled (default)
        # 1 = RLD lead-off sense is enabled
        self.emg_pbd_rld_loff_sense_config = {'on': 0b10, 'off': 0b00}

        # RLDREF_INT: RLDREF signal
        # 0 = RLDREF signal fed externally
        # 1 = RLDREF signal (AVDD – AVSS) / 2 generated internally (default)
        self.emg_rldref_int_config = {'external': 0, 'internal': 1}

        # INT_TEST: Test signal selection
        # This bit determines whether the test signal is turned on or off.
        # 0 = Off (default)
        # 1 = On; amplitude = ±(VREFP – VREFN) / 2400
        # TEST_FREQ: Test signal frequency
        # This bit determines the test signal frequency.
        # 0 = At dc (default)
        # 1 = Square wave at 1 Hz
        self.test_signal_config = {'DC': 0b10, 'square': 0b11, 'off': 0b00}

        # EMG 24 bit sensor activation
        self.emg_24bit = [0x18, 0x00, 0x00]

        # Signal calibration parameters
        self.adc_offset_ch1 = 0
        self.adc_offset_ch2 = 0
        self.adc_sensitivity = 2420 / (2**23 - 1)

        # Dictionary to store possible values for charge status LED
        self.charge_status_dict = defaultdict(
            lambda: 'UNDEFINED',
            {
                0: 'FULL',
                1: 'MEDIUM',
                2: 'LOW',
            },
        )

        # Signal calibrated indicator
        self.signal_calibrated = False

    def open_port(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=115200,
            timeout=5.0,
            write_timeout=5.0,
        )
        rospy.loginfo(f'Port {self.port} opened: {self.ser.is_open}')
        self.ser.reset_input_buffer()
        return self.ser

    def __enter__(self):
        return self.open_port()

    def close_port(self):
        if self.ser.is_open:
            self.ser.close()
            rospy.loginfo(f'Port {self.port} opened: {self.ser.is_open}')

    def __exit__(self, exception_type, exception_value, exception_traceback):
        # Exception handling
        self.close_port()

    def wait_for_ack(self):
        """
        Acknowledge the error-free receipt of transmitted data.
        """
        if self.ser.is_open:
            ddata = ''
            ack = struct.pack('B', self.packet_type['ACK_COMMAND_PROCESSED'])
            while ddata != ack:
                ddata = self.ser.read(1)
        else:
            raise serial.SerialException

    def set_sampling_rate(self, sampling_rate):
        """Set Shimmer3 sampling rate to desired value.

        Args:
            sampling_rate (int): Sampling rate in Hz.
        """
        try:
            sampling_rate = int(sampling_rate)
            if not (0.6 <= sampling_rate <= 1024):
                raise ValueError
            if self.ser.is_open:
                # Calculate from Hz to ms
                sampling_period = round(32768 / sampling_rate)
                # Send sampling rate command bitwise & and >> due to alignment issues
                self.ser.write(
                    struct.pack(
                        'BBB',
                        self.packet_type['SET_SAMPLING_RATE_COMMAND'],
                        (sampling_period & 0xFF),
                        ((sampling_period & 0xFF00) >> 8),
                    )
                )
                self.wait_for_ack()
                # Read and print set sampling rate
                self.get_sampling_rate(echo=True)
            else:
                raise serial.SerialException
        except ValueError as e:
            sys.exit(
                'set_sampling_rate not an integer or between 1 and 1024 Hz{}'.format(e)
            )

    def get_sampling_rate(self, echo=False):
        """Read and print sampling rate"""
        if self.ser.is_open:
            # Send sampling rate command
            self.ser.write(
                struct.pack('B', self.packet_type['GET_SAMPLING_RATE_COMMAND'])
            )
            self.wait_for_ack()
            # Read incoming data 1 identifying byte, 2 for uint sampling rate
            data = self.ser.read(size=3)
            clock_wait = struct.unpack('H', data[1:3])[0]
            self.sampling_rate = 32768 / clock_wait
            if echo:
                rospy.loginfo(f'Shimmer sampling rate: {self.sampling_rate:.4f} Hz')
            return self.sampling_rate
        else:
            raise serial.SerialException

    def get_buffer_size(self, echo=False):
        """Read and print buffer size"""
        if self.ser.is_open:
            # Send get buffer size command
            self.ser.write(
                struct.pack('B', self.packet_type['GET_BUFFER_SIZE_COMMAND'])
            )
            self.wait_for_ack()
            # Read incoming data 1 identifying byte, 2 for uint sampling rate
            data = self.ser.read(size=2)
            if data[0] == self.packet_type['BUFFER_SIZE_RESPONSE']:
                self.buffer_size = data[1]
                if echo:
                    rospy.loginfo(f'Buffer size: {self.buffer_size}')
                return self.buffer_size
            else:
                rospy.logerr('Did not recieve BUFFER_SIZE_RESPONSE')
        else:
            raise serial.SerialException

    def get_charge_status_led(self, echo=False):
        """Read and print charge status LED"""
        if self.ser.is_open:
            # send get charge status led command
            self.ser.write(
                struct.pack('B', self.packet_type['GET_CHARGE_STATUS_LED_COMMAND'])
            )
            self.wait_for_ack()
            # Read incoming data 1 identifying byte, 1 charge status
            data = self.ser.read(size=2)
            if data[0] == self.packet_type['CHARGE_STATUS_LED_RESPONSE']:
                charge_status_num = data[1]
                self.charge_status = self.charge_status_dict[charge_status_num]
                if echo:
                    rospy.loginfo(f'Charge status: {self.charge_status}')
                return self.charge_status
            else:
                rospy.logerr('Did not recieve CHARGE_STATUS_LED_RESPONSE')
        else:
            raise serial.SerialException

    def get_id_and_rev(self, echo=True):
        """Get the daughter card ID byte (SR number) and Revision number"""
        if self.ser.is_open:
            self.ser.write(
                struct.pack(
                    'BBB', self.packet_type['GET_DAUGHTER_CARD_ID_COMMAND'], 0x02, 0x00
                )
            )
            self.wait_for_ack()
            # Read incoming data byte 3 - serial number, byte 4 revision
            data = self.ser.read(size=4)
            if data[0] == self.packet_type['DAUGHTER_CARD_ID_RESPONSE']:
                self.serial_number = data[2]
                self.revision_number = data[3]
                if echo:
                    rospy.loginfo(
                        f'Device: SR{self.serial_number}-{self.revision_number}'
                    )
                return (self.serial_number, self.revision_number)
            else:
                rospy.logerr('Did not recieve DAUGHTER_CARD_ID_RESPONSE')
        else:
            raise serial.SerialException

    def get_shimmer_name(self, echo=True):
        """Get the Shimmer name"""
        if self.ser.is_open:
            self.ser.write(
                struct.pack('B', self.packet_type['GET_SHIMMERNAME_COMMAND'])
            )
            self.wait_for_ack()
            # Read 2 incoming data bytes: 0 - response, 1 - length, 2: encoded name
            data = self.ser.read(2)
            if data[0] == self.packet_type['SHIMMERNAME_RESPONSE']:
                # Read entire name length
                data = self.ser.read(data[1])
                self.shimmer_name = data.decode('ascii')
                if echo:
                    rospy.loginfo(f'Device name: {self.shimmer_name}')
                return self.shimmer_name
            else:
                rospy.logerr('Did not recieve SHIMMERNAME_RESPONSE')
        else:
            raise serial.SerialException

    def set_shimmer_name(self, shimmer_name):
        """Set the Shimmer name to string with 11 characters"""
        if len(shimmer_name) > 11:
            sys.exit('Desire shimmer name too long (must be <= 11 characters)')
        if self.ser.is_open:
            self.ser.write(
                struct.pack(
                    'B' * (len(shimmer_name) + 2),
                    self.packet_type['SET_SHIMMERNAME_COMMAND'],
                    len(shimmer_name),
                    *shimmer_name.encode('ascii'),
                )
            )
            self.wait_for_ack()
            self.get_shimmer_name()
        else:
            raise serial.SerialException

    def set_emg_gain(self, gain, echo=True):
        """Set gain to variable"""
        # Check if valid gain input
        if gain in self.emg_gain_config.keys():
            self.emg_gain = gain
        else:
            self.emg_gain = self.emg_gain_config_default
        if echo:
            rospy.loginfo(f'EMG gain: {self.emg_gain}')
        self.emg_gain_packet = self.emg_gain_config[self.emg_gain]
        rospy.logdebug(f'{self.emg_gain_packet:03b}')
        return self.emg_gain

    def set_emg_data_rate(self, emg_data_rate, echo=True):
        """Set emg chip data rate (samples per second) to variable"""
        # Check if valid data rate input
        if emg_data_rate in self.emg_data_rate_config.keys():
            self.emg_data_rate = emg_data_rate
        else:
            self.sampling_rate = self.get_sampling_rate()
            # Set the data rate to first value higher than Shimmer sampling rate
            self.emg_data_rate = min(
                [i for i in self.emg_data_rate_config.keys() if i > self.sampling_rate]
            )
        if echo:
            rospy.loginfo(f'EMG chip data rate: {self.emg_data_rate} Hz')
        self.emg_data_rate_packet = self.emg_data_rate_config[self.emg_data_rate]
        return self.emg_data_rate

    def get_emg_registers(self, chip_num=0, echo=False):
        """Get byte values for all 10 registers for chip_num on EMG unit"""
        if chip_num not in [0, 1]:
            sys.exit('Wrong chip number specified. Must be 0 or 1')
        if self.ser.is_open:
            self.ser.write(
                struct.pack(
                    'B' * 4, self.packet_type['GET_EXG_REGS_COMMAND'], chip_num, 0, 10
                )
            )
            self.wait_for_ack()
            # Read incoming data bytes (EXG_REGS_RESPONSE + number of bytes + 10 registers)
            data = self.ser.read(size=12)
            if data[0] == self.packet_type['EXG_REGS_RESPONSE']:
                emg_regs = list(struct.unpack('B' * 10, data[2:]))
                # Store only chip1 registers
                if chip_num == 0:
                    self.emg_regs = emg_regs
                if echo:
                    rospy.logdebug(
                        f'EMG register settings for chip {chip_num + 1}:\n'
                        f'\tCONFIG1: {emg_regs[0]:08b}\n'
                        f'\tCONFIG2: {emg_regs[1]:08b}\n'
                        f'\tLOFF: {emg_regs[2]:08b}\n'
                        f'\tCH1SET: {emg_regs[3]:08b}\n'
                        f'\tCH2SET: {emg_regs[4]:08b}\n'
                        f'\tRLD_SENS: {emg_regs[5]:08b}\n'
                        f'\tLOFF_SENS: {emg_regs[6]:08b}\n'
                        f'\tLOFF_STAT: {emg_regs[7]:08b}\n'
                        f'\tRESP1: {emg_regs[8]:08b}\n'
                        f'\tRESP2: {emg_regs[9]:08b}\n'
                    )
                return emg_regs
            else:
                rospy.logerr('Did not recieve EXG_REGS_RESPONSE')
        else:
            raise serial.SerialException

    def power_down_chip_2(self, echo=True):
        """
        Chip 2 should be powered down for EMG signal acquisition.
        Input multiplexer should be set to input shorted configuration.
        Bit 7 in CH1SET and CH2SET bytes should be set to 1 (Channel x power-down).
        """
        # Get EMG registers for chip 2
        chip_num = 1
        # self.get_emg_registers(chip_num=chip_num, echo=True)
        self.chip2_emg_regs = [None] * 10
        # CONFIG1 byte - Data Rate
        self.chip2_emg_regs[0] = self.emg_data_rate_packet
        # CONFIG2 byte - Test signals
        self.chip2_emg_regs[1] = 0b10101000 | self.test_signal_config['off']
        # LOFF byte
        self.chip2_emg_regs[2] = 0b00010000
        # CH1SET - Bit7 to 1, gain to default and mux to shorted
        self.chip2_emg_regs[3] = (
            0b1 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config['shorted']
        )
        # CH2SET - Bit7 to 1, gain to default and mux to shorted
        self.chip2_emg_regs[4] = (
            0b1 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config['shorted']
        )
        # RLD_SENS byte - all 8 bits to 0
        self.chip2_emg_regs[5] = (
            0b00 << 6
            | self.emg_pbd_rld_loff_sense_config['off'] << 4
            | self.emg_rld_config
        )
        # LOFF_SENS and LOFF_STAT bytes
        self.chip2_emg_regs[6], self.chip2_emg_regs[7] = 0b0, 0b0
        # RESP1 byte
        self.chip2_emg_regs[8] = 0b10
        # RESP 2 byte
        self.chip2_emg_regs[9] = self.emg_rldref_int_config['external'] << 1 | 0b1
        # rospy.loginfo('EMG register settings for chip {}:\n'.format(chip_num+1),
        #       '\tCONFIG1: {:08b}\n'.format(self.chip2_emg_regs[0]),
        #       '\tCONFIG2: {:08b}\n'.format(self.chip2_emg_regs[1]),
        #       '\tLOFF: {:08b}\n'.format(self.chip2_emg_regs[2]),
        #       '\tCH1SET: {:08b}\n'.format(self.chip2_emg_regs[3]),
        #       '\tCH2SET: {:08b}\n'.format(self.chip2_emg_regs[4]),
        #       '\tRLD_SENS: {:08b}\n'.format(self.chip2_emg_regs[5]),
        #       '\tLOFF_SENS: {:08b}\n'.format(self.chip2_emg_regs[6]),
        #       '\tLOFF_STAT: {:08b}\n'.format(self.chip2_emg_regs[7]),
        #       '\tRESP1: {:08b}\n'.format(self.chip2_emg_regs[8]),
        #       '\tRESP2: {:08b}\n'.format(self.chip2_emg_regs[9]),
        # )
        # Write configuration to chip 2 register
        if self.ser.is_open:
            # Send set registers command, chip to write to, starting byte,
            # number of bytes to write and unpack a list with registers to write
            self.ser.write(
                struct.pack(
                    'B' * 14,
                    self.packet_type['SET_EXG_REGS_COMMAND'],
                    chip_num,
                    0,
                    10,
                    *self.chip2_emg_regs,
                )
            )
            self.wait_for_ack()
            if echo:
                rospy.loginfo(f'Chip {chip_num + 1} powered down for EMG measurement!')
            return self.chip2_emg_regs
        else:
            raise serial.SerialException

    def activate_emg_sensors(self, echo=True):
        """Set the 24 bit EXG sensors"""
        sensors = [self.packet_type['SET_SENSORS_COMMAND']] + self.emg_24bit
        if self.ser.is_open:
            # Send set sensors command and emg_24 bit bytes to activate sensor
            self.ser.write(struct.pack('B' * 4, *sensors))
            self.wait_for_ack()
            if echo:
                rospy.loginfo('24 bit EMG sensor activated!')
        else:
            raise serial.SerialException

    def set_emg_registers(self, test_signal=False, echo=True):
        """Set the EMG registers for chip 1 (EMG signal or test signal)"""
        # Get EMG registers for chip 0
        chip_num = 0
        if test_signal:
            # Set mux config and test signal byte
            mux = 'test'
            test = 'square'
        else:
            mux = 'measure_EMG'
            test = 'off'
        # Initialize list for EMG registers
        self.chip1_emg_regs = [None] * 10
        # CONFIG1 byte - Data Rate
        self.chip1_emg_regs[0] = self.emg_data_rate_packet
        # CONFIG2 byte - Test signals
        self.chip1_emg_regs[1] = 0b10101000 | self.test_signal_config[test]
        # LOFF byte
        self.chip1_emg_regs[2] = 0b00010000
        # CH1SET - Bit7 to 0, gain to defined and mux to test/measure_EMG
        self.chip1_emg_regs[3] = (
            0b0 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config[mux]
        )
        # CH2SET - Bit7 to 0, gain to defined and mux to test/measure_EMG
        self.chip1_emg_regs[4] = (
            0b0 << 7 | self.emg_gain_packet << 4 | self.emg_mux_config[mux]
        )
        # RLD_SENS byte - PBD_RLD to on
        self.chip1_emg_regs[5] = (
            0b00 << 6
            | self.emg_pbd_rld_loff_sense_config['on'] << 4
            | self.emg_rld_config
        )
        # LOFF_SENS and LOFF_STAT bytes
        self.chip1_emg_regs[6], self.chip1_emg_regs[7] = 0b0, 0b0
        # RESP1 byte
        self.chip1_emg_regs[8] = 0b10
        # RESP 2 byte - RLDREF_INT to internal
        self.chip1_emg_regs[9] = self.emg_rldref_int_config['internal'] << 1 | 0b1
        # rospy.logdebug(
        #   f'EMG register settings for chip {chip_num+1}:\n'
        #       f'\tCONFIG1: {self.chip1_emg_regs[0]:08b}\n'
        #       f'\tCONFIG2: {self.chip1_emg_regs[1]:08b}\n'
        #       f'\tLOFF: {self.chip1_emg_regs[2]:08b}\n'
        #       f'\tCH1SET: {self.chip1_emg_regs[3]:08b}\n'
        #       f'\tCH2SET: {self.chip1_emg_regs[4]:08b}\n'
        #       f'\tRLD_SENS: {self.chip1_emg_regs[5]:08b}\n'
        #       f'\tLOFF_SENS: {self.chip1_emg_regs[6]:08b}\n'
        #       f'\tLOFF_STAT: {self.chip1_emg_regs[7]:08b}\n'
        #       f'\tRESP1: {self.chip1_emg_regs[8]:08b}\n'
        #       f'\tRESP2: {self.chip1_emg_regs[9]:08b}\n'
        # )
        # Write configuration to chip 1 register
        if self.ser.is_open:
            # Send set registers command, chip to write to, starting byte,
            # number of bytes to write and unpack a list with registers to write
            self.ser.write(
                struct.pack(
                    'B' * 14,
                    self.packet_type['SET_EXG_REGS_COMMAND'],
                    chip_num,
                    0,
                    10,
                    *self.chip1_emg_regs,
                )
            )
            self.wait_for_ack()
            if echo:
                rospy.loginfo(f'Chip {chip_num + 1} EMG register settings written!')
            return self.chip1_emg_regs
        else:
            raise serial.SerialException

    def start_streaming_EMG(self, publisher, test_signal=False, echo=True):
        """
        Start streaming EMG signal from both channels to desired publisher.
        Set chip 1 settings,
        Start streaming EMG signal
        Args:
            test_signal (bool, optional): stream/test/acquired signal. Defaults to False.
            echo (bool, optional): print info to console. Defaults to True.
        """
        # Calibration constants and name
        self.calibration_constant = self.adc_sensitivity / self.emg_gain
        calibration_constant = self.calibration_constant
        adc_offset_ch1 = self.adc_offset_ch1
        adc_offset_ch2 = self.adc_offset_ch2
        shimmer_name = self.shimmer_name
        # Set the chip 1 configuration registers
        self.set_emg_registers(test_signal=test_signal)
        self.get_emg_registers(chip_num=0, echo=echo)
        # Read incoming data
        # 1 byte packet type, 3 bytes timestamp, 14 bytes EMG data
        framesize = 18
        # Firmware clock in 1/32768 sec
        clock_step = 32768
        # ROS message
        emg_data = Emg()
        emg_data.header.frame_id = shimmer_name
        # Iterator
        sig_iter = 0
        # Send start streaming command
        self.send_streaming_command('start', echo=True)
        # Sleep for 0.5 sec before data acquisition to make sure clock ticks are
        # sufficiently large
        rospy.sleep(0.5)
        try:
            while not rospy.is_shutdown():
                data = self.ser.read(size=framesize)
                if data[0] == self.packet_type['DATA_PACKET']:
                    # Convert bytes to internal clock_ticks
                    clock_ticks = int.from_bytes(data[1:4], 'little')
                    # Resync with ROS time or handle clock_tick overflow
                    # after after 3 bytes max uint - 16777215
                    # max time: 511.9999 sec or 8.5333 min
                    # (exploit OR evaluating only first condition if True)
                    if sig_iter == 0 or clock_ticks <= clock_ticks_ref:
                        # Set starting time
                        time_start = rospy.Time.now()
                        # Set now clock ticks to zero reference
                        clock_ticks_ref = clock_ticks
                    # Convert to duration in secs and nsecs
                    duration = rospy.Duration.from_sec(
                        (clock_ticks - clock_ticks_ref) / clock_step
                    )
                    emg_data.header.stamp = time_start + duration
                    # Convert chip 1, channel 1 and 2 data to integer values
                    c1ch1 = int.from_bytes(data[5:8], byteorder='big', signed=True)
                    c1ch2 = int.from_bytes(data[8:11], byteorder='big', signed=True)
                    # Calibrate emg channels using constant and offset:
                    c1ch1 = c1ch1 * calibration_constant - adc_offset_ch1
                    c1ch2 = c1ch2 * calibration_constant - adc_offset_ch2
                    # Publish acquired data to topic
                    emg_data.emg_ch1 = c1ch1
                    emg_data.emg_ch2 = c1ch2
                    emg_data.header.seq = sig_iter
                    publisher.publish(emg_data)
                    # Increment ID
                    sig_iter += 1
                else:
                    rospy.logerr(f'Did not recieve DATA_PACKET: {data}')
        except rospy.ROSInterruptException:
            # Send stop streaming command
            self.send_streaming_command('stop', echo=True)
            # Necessary to wait a bit before flushing input
            rospy.sleep(0.1)
            self.ser.reset_input_buffer()
            raise rospy.ROSInterruptException
        # Send stop streaming command
        self.send_streaming_command('stop', echo=True)
        # Necessary to wait a bit before flushing input
        rospy.sleep(0.1)
        self.ser.reset_input_buffer()
        return

    def send_streaming_command(self, command='start', echo=False):
        """Send start streaming command to Shimmer3"""
        if command == 'start':
            command_type = 'START_STREAMING_COMMAND'
            print_msg = 'Starting data stream!'
        else:
            command_type = 'STOP_STREAMING_COMMAND'
            print_msg = 'Stopping data stream!'
        if self.ser.is_open:
            self.ser.write(struct.pack('B', self.packet_type[command_type]))
            self.wait_for_ack()
            if echo:
                rospy.loginfo(print_msg)
        else:
            raise serial.SerialException

    def calibrate_test_signal(self, duration=5, echo=True):
        """Calibrate ADC_offset using square test signal"""
        # Calibration constant
        self.calibration_constant = self.adc_sensitivity / self.emg_gain
        calibration_constant = self.calibration_constant
        if echo:
            rospy.loginfo(f'Calibration constant: {self.calibration_constant:.6e}')
        if duration <= 0:
            return
        # Length of signal should correspond to duration in seconds
        signal_length = round(self.sampling_rate * duration)
        # Set the chip 1 configuration registers to stream test signal
        self.set_emg_registers(test_signal=True)
        self.get_emg_registers(chip_num=0, echo=echo)
        if echo:
            rospy.loginfo(
                f'Performing signal calibration ... please wait {duration} s!'
            )
        # Send start streaming command
        self.send_streaming_command('start', echo=True)
        # Read incoming data
        # 1 byte packet type, 3 bytes timestamp, 14 bytes EMG data
        framesize = 18
        # Queue for faster data acquisition
        c1ch1_q, c1ch2_q = deque(maxlen=int(1e6)), deque(maxlen=int(1e6))
        try:
            for _ in range(signal_length):
                data = self.ser.read(size=framesize)
                if data[0] == self.packet_type['DATA_PACKET']:
                    # Convert chip 1, channel 1 and 2 data to integer values
                    c1ch1 = int.from_bytes(data[5:8], byteorder='big', signed=True)
                    c1ch2 = int.from_bytes(data[8:11], byteorder='big', signed=True)
                    # Calibrate emg channels using constant and offset:
                    c1ch1 = c1ch1 * calibration_constant
                    c1ch2 = c1ch2 * calibration_constant
                    # Append results to deque
                    c1ch1_q.append(c1ch1)
                    c1ch2_q.append(c1ch2)
                else:
                    rospy.logerr(f'Did not recieve DATA_PACKET: {data}')
        except rospy.ROSInterruptException:
            # Send stop streaming command
            self.send_streaming_command('stop', echo=echo)
            # Necessary to wait a bit before flushing input
            rospy.sleep(0.2)
            self.ser.reset_input_buffer()
            raise rospy.ROSInterruptException
        # Send stop streaming command
        self.send_streaming_command('stop', echo=echo)
        # Necessary to wait a bit before flushing input
        rospy.sleep(0.2)
        self.ser.reset_input_buffer()
        # Retain only values with amplitude near -1 and 1
        c1ch1 = [i for i in c1ch1_q if abs(abs(i) - 1) < 0.1]
        c1ch2 = [i for i in c1ch2_q if abs(abs(i) - 1) < 0.1]
        # Keep same number of positive and negative values in list
        pos_count = len([i for i in c1ch1 if i >= 0])
        while pos_count != len(c1ch1) - pos_count:
            c1ch1.pop()
            pos_count = len([i for i in c1ch1 if i >= 0])
        pos_count = len([i for i in c1ch2 if i >= 0])
        while pos_count != len(c1ch2) - pos_count:
            c1ch2.pop()
            pos_count = len([i for i in c1ch2 if i >= 0])
        # Calculate ADC offset as signal mean value
        self.adc_offset_ch1 = sum(c1ch1) / len(c1ch1)
        self.adc_offset_ch2 = sum(c1ch2) / len(c1ch2)
        self.signal_calibrated = True
        if echo:
            rospy.loginfo('Calibration done!')
            rospy.loginfo(f'ADC offset for Channel 1: {self.adc_offset_ch1:.6f}')
            rospy.loginfo(f'ADC offset for Channel 2: {self.adc_offset_ch2:.6f}')
        return

    def synchronise_system_time(self, ntimes=3, echo=False):
        """Get real world clock from shimmer"""
        # Firmware clock in 1/32768 sec
        if echo:
            rospy.loginfo('Synchronising Shimmer time with system time!')
        clock_step = 32768
        rate = rospy.Rate(2)  # 2 Hz
        if self.ser.is_open:
            for i in range(ntimes):
                # Capture ROS time to timestamp
                system_ts = rospy.Time.now()
                self.ser.write(struct.pack('B', self.packet_type['GET_RWC_COMMAND']))
                self.wait_for_ack()
                # Read 9 bytes - first response and 8 system time in clock time
                data = self.ser.read(9)
                if data[0] == self.packet_type['RWC_RESPONSE']:
                    # Convert bytes to internal clock_ticks
                    clock_ticks = int.from_bytes(data[1:], 'little')
                    # Convert to timestamp in secs
                    timestamp = clock_ticks / clock_step
                    dt = datetime.fromtimestamp(timestamp)
                    # Convert ROS time to datetime
                    system_dt = datetime.fromtimestamp(system_ts.to_sec())
                    system_timestamp = system_dt.timestamp()
                    if echo:
                        rospy.loginfo(
                            f'Shimmer3 time: {dt},\t timestamp: {timestamp},\t'
                        )
                    if echo:
                        rospy.loginfo(
                            f'System time:   {system_dt},\t timestamp: {system_timestamp}\t'
                        )
                    # Sending set real world clock command
                    self.ser.write(
                        struct.pack(
                            'B' * 9,
                            self.packet_type['SET_RWC_COMMAND'],
                            *(
                                (
                                    round(rospy.Time.now().to_sec() * clock_step)
                                ).to_bytes(8, 'little')
                            ),
                        )
                    )
                    self.wait_for_ack()
                    # Wait till specific rate finished
                    rate.sleep()
                else:
                    rospy.logerr('Did not recieve RWC_RESPONSE!')
        else:
            raise serial.SerialException
