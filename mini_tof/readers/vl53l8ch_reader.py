"""
Class which enables reading and decoding data from a VL53L8CH ToF sensor connected to a
microcontroller via USB serial.
"""

import struct
import time

import serial

from mini_tof_interfaces.msg import AmbientLight, DepthEstimate, ToFFrame, ToFHistogram

class VL53L8CHReader:

    def __init__(self, port, num_zones):
        self.mcu = serial.Serial(
            port,
            921600,
            bytesize=8,
            parity=serial.PARITY_ODD,
            stopbits=1,
            dsrdtr=True,
            rtscts=True,
            xonxoff=False,
        )
        time.sleep(2)

        self.num_zones = num_zones
        # the sensor only supports 16 or 64 zones
        if not self.num_zones in [16, 64]:
            print(f"VL53L8CH sensor only supports 16 or 64 zones ({self.num_zones} provided)")
            return

    def get_measurement(self):
        self.mcu.reset_input_buffer()

        started = False
        second_cycle = False
        concat_frame_data = [None] * self.num_zones
        concat_ambient_light = [None] * self.num_zones
        i = 0
        while i < 3: # 3 frames per shift
            frame_data = {}
            frame_ambient_light = {}
        
            # We expect zone idx 0 to come first - if not, bad frame and throw it out until we see
            # zone idx 0
            expected_zone_idx = 0
            while expected_zone_idx < self.num_zones:
                try:
                    zone_idx, ambient_light, zone_data, _, distance_mm, range_sigma_mm, reading_counter, start_bin = (
                        self.readline_and_decode()
                    )
                except Exception as e:
                    print(f"(1) Notice, reading serial data: {e}")
                    continue
                if reading_counter == 0 and start_bin == 0 and started:
                    second_cycle = True # after the second cycle, data is consistent
                if reading_counter == 0 and start_bin == 0:
                    started = True

                frame_data[zone_idx] = zone_data
                frame_ambient_light[zone_idx] = ambient_light
                if zone_idx == self.num_zones - 1 and expected_zone_idx != self.num_zones - 1:
                    expected_zone_idx = 0
                else:
                    expected_zone_idx += 1
            
            # After collecting a complete frame, add it to concat_frame_data
            if second_cycle and reading_counter == 2:
                i += 1
                for zone_idx, zone_data in frame_data.items():
                    if concat_frame_data[zone_idx] is None:
                        concat_frame_data[zone_idx] = []
                        concat_ambient_light[zone_idx] = []
                    concat_frame_data[zone_idx].extend(zone_data)
                    concat_ambient_light[zone_idx].append(frame_ambient_light[zone_idx]) # 1 per zone per reading (nx3 total)
        return concat_frame_data, concat_ambient_light, distance_mm, range_sigma_mm

    def measurement_to_ros_msg(self, m, mcu_port, sensor_model):
        """
        Convert a measurement as returned by get_measurement() into a
        mini_tof_interfaces.msg.ToFFrame ROS message.

        Args:
            m (tuple): The measurement returned by get_measurement().
            mcu_port (str): The port of the microcontroller.
            sensor_model (str): The model of the sensor.
        
        Returns:
            ToFFrame: A ROS message containing the processed measurement data.
        """
        concat_frame_data, concat_ambient_light, distance_mm, range_sigma_mm = m
        message = ToFFrame()
        message.ambient_lights = [AmbientLight(ambient_lights=al) for al in concat_ambient_light]  # ambient light for each zone
        message.i2c_address = -1  # VL53L8CH does not use I2C address
        message.tick = -1 # VL53L8CH does not provide tick information
        message.num_valid_results = -1 # does not apply to VL53L8CH
        message.temperature = -1 # VL53L8CH does not provide temperature information
        message.measurement_num = -1 # VL53L8CH does not provide measurement number
        message.depth_estimates = [
            DepthEstimate(depth_estimates=distance_mm, confidences=range_sigma_mm),
        ]
        message.serial_port = mcu_port
        message.sensor_model = sensor_model
        message.histograms = [ToFHistogram(histogram=hist) for hist in concat_frame_data]
        message.reference_histogram = ToFHistogram(histogram=[]) # VL53L8CH does not provide a reference histogram

        return message

    def readline_and_decode(self):
        """
        Given a serial port object denoting a port connected to an MCU with a VL3L8CH connected,
        read a line of data from the port and decode it

        Returns:
            int: Zone index
            list: List of floats representing the data from the zone
            int: Length of the binary data read
        """

        eol = b"\xFF\xFF\xFF\xFF"
        distance_mm = None
        range_sigma_mm = None
        reading_counter = None
        start_bin = None

        c = self.mcu.read_until(expected=eol)
        byte_listing = bytearray(c)
        length = len(byte_listing) - len(byte_listing) % 4
        line_as_floats = [
            struct.unpack("<f", bytes(byte_listing[i : i + 4]))[0] for i in range(0, length, 4)
        ]

        # extract data, see MCU code for ordering, line_as_floats[-1] is always thrown out
        try:
            zone_index = int(line_as_floats[0])  # zone index
            ambient_light = line_as_floats[1]  # ambient light
            hist = line_as_floats[2:-1] # histogram data
        
        except Exception as e:
            print(f"Error parsing data: {e}")
            raise ValueError(f"Failed to decode serial data: {e}")

        # grab ending metadata
        if zone_index == self.num_zones - 1:
            c = self.mcu.read_until(expected=eol)
            byte_listing = bytearray(c)
            length2 = len(byte_listing) - len(byte_listing) % 4
            line_as_floats = [
                struct.unpack("<f", bytes(byte_listing[i : i + 4]))[0] for i in range(0, length2, 4)
            ]
            try:
                distance_mm = line_as_floats[:self.num_zones]
                range_sigma_mm = line_as_floats[self.num_zones:-3]
                reading_counter = int(line_as_floats[-3])  # readings_since_swap
                start_bin = int(line_as_floats[-2])        # startBin value
            except Exception as e:
                print(f"Error reading metadata: {e}")
                raise ValueError(f"Failed to decode metadata from serial: {e}")
        
        return zone_index, ambient_light, hist, length, distance_mm, range_sigma_mm, reading_counter, start_bin


class VL53L8CHReaderNoAggregation:

    def __init__(self, port, num_zones):
        self.mcu = serial.Serial(
            port,
            921600,
            bytesize=8,
            parity=serial.PARITY_ODD,
            stopbits=1,
            dsrdtr=True,
            rtscts=True,
            xonxoff=False,
        )
        time.sleep(2)

        self.num_zones = num_zones
        # the sensor only supports 16 or 64 zones
        if not self.num_zones in [16, 64]:
            print(f"VL53L8CH sensor only supports 16 or 64 zones ({self.num_zones} provided)")
            return

    def capture(self):
        self.mcu.reset_input_buffer()

        frame_data = [None] * self.num_zones
        frame_ambient_light = [None] * self.num_zones
    
        # We expect zone idx 0 to come first - if not, bad frame and throw it out until we see
        # zone idx 0
        expected_zone_idx = 0
        while expected_zone_idx < self.num_zones:
            try:
                zone_idx, ambient_light, zone_data, _, distance_mm, range_sigma_mm, _, _ = (
                    self.readline_and_decode()
                )
            except Exception as e:
                print(f"(1) Notice, reading serial data: {e}")
                continue

            try:
                frame_data[zone_idx] = zone_data
                frame_ambient_light[zone_idx] = ambient_light
            except Exception as e:
                pass
            if zone_idx == self.num_zones - 1 and expected_zone_idx != self.num_zones - 1:
                expected_zone_idx = 0
            else:
                expected_zone_idx += 1
        return frame_data, frame_ambient_light, distance_mm, range_sigma_mm
    
    def measurement_to_ros_msg(self, m, mcu_port, sensor_model):
        """
        Convert a measurement as returned by get_measurement() into a
        mini_tof_interfaces.msg.ToFFrame ROS message.

        Args:
            m (tuple): The measurement returned by get_measurement().
            mcu_port (str): The port of the microcontroller.
            sensor_model (str): The model of the sensor.
        
        Returns:
            ToFFrame: A ROS message containing the processed measurement data.
        """
        concat_frame_data, concat_ambient_light, distance_mm, range_sigma_mm = m
        message = ToFFrame()
        message.ambient_lights = [AmbientLight(ambient_lights=al) for al in concat_ambient_light]  # ambient light for each zone
        message.i2c_address = -1  # VL53L8CH does not use I2C address
        message.tick = -1 # VL53L8CH does not provide tick information
        message.num_valid_results = -1 # does not apply to VL53L8CH
        message.temperature = -1 # VL53L8CH does not provide temperature information
        message.measurement_num = -1 # VL53L8CH does not provide measurement number
        message.depth_estimates = [
            DepthEstimate(depth_estimates=distance_mm, confidences=range_sigma_mm),
        ]
        message.serial_port = mcu_port
        message.sensor_model = sensor_model
        message.histograms = [ToFHistogram(histogram=hist) for hist in concat_frame_data]
        message.reference_histogram = ToFHistogram(histogram=[]) # VL53L8CH does not provide a reference histogram

        return message

    def readline_and_decode(self):
        """
        Given a serial port object denoting a port connected to an MCU with a VL3L8CH connected,
        read a line of data from the port and decode it

        Returns:
            int: Zone index
            list: List of floats representing the data from the zone
            int: Length of the binary data read
        """

        eol = b"\xFF\xFF\xFF\xFF"
        distance_mm = None
        range_sigma_mm = None
        reading_counter = None
        start_bin = None

        c = self.mcu.read_until(expected=eol)
        byte_listing = bytearray(c)
        length = len(byte_listing) - len(byte_listing) % 4
        line_as_floats = [
            struct.unpack("<f", bytes(byte_listing[i : i + 4]))[0] for i in range(0, length, 4)
        ]

        # extract data, see MCU code for ordering, line_as_floats[-1] is always thrown out
        try:
            zone_index = int(line_as_floats[0])  # zone index
            ambient_light = line_as_floats[1]  # ambient light
            hist = line_as_floats[2:-1] # histogram data
        
        except Exception as e:
            print(f"Error parsing data: {e}")
            raise ValueError(f"Failed to decode serial data: {e}")

        # grab ending metadata
        if zone_index == self.num_zones - 1:
            c = self.mcu.read_until(expected=eol)
            byte_listing = bytearray(c)
            length2 = len(byte_listing) - len(byte_listing) % 4
            line_as_floats = [
                struct.unpack("<f", bytes(byte_listing[i : i + 4]))[0] for i in range(0, length2, 4)
            ]
            try:
                distance_mm = line_as_floats[:self.num_zones]
                range_sigma_mm = line_as_floats[self.num_zones:-3]
                reading_counter = int(line_as_floats[-3])  # readings_since_swap
                start_bin = int(line_as_floats[-2])        # startBin value
            except Exception as e:
                print(f"Error reading metadata: {e}")
                raise ValueError(f"Failed to decode metadata from serial: {e}")
        
        return zone_index, ambient_light, hist, length, distance_mm, range_sigma_mm, reading_counter, start_bin