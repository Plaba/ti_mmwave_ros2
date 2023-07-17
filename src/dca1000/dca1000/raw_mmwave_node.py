#!/usr/bin/env python
# license removed for brevity
from rclpy.node import Node
import rclpy
import socket
import struct
import numpy as np

from ti_mmwave_msgs.msg import DataFrame, MMWaveConfig
from .ring_buffer import RingBuffer


class DCA1000Sensor(Node):
    # dca1000evm configuration commands; only the ones used are filled in
    DCA_CMDS = {
        'RESET_FPGA_CMD_CODE'               : b"",
        'RESET_AR_DEV_CMD_CODE'             : b"",
        'CONFIG_FPGA_GEN_CMD_CODE'          : b"\x5a\xa5\x03\x00\x06\x00\x01\x01\x01\x02\x03\x1e\xaa\xee",
        'CONFIG_EEPROM_CMD_CODE'            : b"",
        'RECORD_START_CMD_CODE'             : b"\x5a\xa5\x05\x00\x00\x00\xaa\xee",
        'RECORD_STOP_CMD_CODE'              : b"\x5a\xa5\x06\x00\x00\x00\xaa\xee",
        'PLAYBACK_START_CMD_CODE'           : b"",
        'PLAYBACK_STOP_CMD_CODE'            : b"",
        'SYSTEM_CONNECT_CMD_CODE'           : b"\x5a\xa5\x09\x00\x00\x00\xaa\xee",
        'SYSTEM_ERROR_CMD_CODE'             : b"\x5a\xa5\x0a\x00\x01\x00\xaa\xee",
        'CONFIG_PACKET_DATA_CMD_CODE'       : b"\x5a\xa5\x0b\x00\x06\x00\xc0\x05\xc4\x09\x00\x00\xaa\xee",
        'CONFIG_DATA_MODE_AR_DEV_CMD_CODE'  : b"",
        'INIT_FPGA_PLAYBACK_CMD_CODE'       : b"",
        'READ_FPGA_VERSION_CMD_CODE'        : b"\x5a\xa5\x0e\x00\x00\x00\xaa\xee",
    }

    DCA_CMD_PORT = 4096
    DCA_DATA_PORT = 4098

    def __init__(self):
        super().__init__('raw_mmwave_node')

        self.seqn = 0 # this is the last packet index
        self.bytec = 0 # this is a byte counter

        self.dca_ip = self.declare_parameter('dca_ip', '192.168.33.30').value
        self.frame_id = self.declare_parameter('frame_id', 'ti_mmwave').value

        self.config_sub = self.create_subscription(MMWaveConfig, 'mmWaveCFG', self.after_configure, 1)
        self.data_pub = self.create_publisher(DataFrame, 'raw_data', 10)

        self.dca_cmd_addr = (self.dca_ip, self.DCA_CMD_PORT)
        self.dca_data_addr = (self.dca_ip, self.DCA_DATA_PORT)

        self.capture_started = False
    
    def after_configure(self, msg:MMWaveConfig):
        self.radar_cfg = msg
        frame_len = 2 * \
            self.radar_cfg.num_adc_samples * \
            self.radar_cfg.num_lvds_lanes * \
            self.radar_cfg.num_tx
        self.data_array = RingBuffer(int(2*frame_len), int(frame_len))

        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.data_socket.bind(self.dca_data_addr)
        self.data_socket.settimeout(25e-5)
        self.data_socket_open = True

        self.dca_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dca_socket.bind(self.dca_cmd_addr)
        self.dca_socket.settimeout(10)
        self.dca_socket_open = True

        self.configureDCA()

        self.collect_data_timer = self.create_timer(25e-5, self.collect_data)
        self.publish_data_timer = self.create_timer(25e-5, self.publish_data)

        self.config_sub = None

        self.context.on_shutdown(self.close)
        self.start_raw_data_capture()

    def collect_response(self):
        status = 1
        while status:
            try:
                msg, server = self.dca_socket.recvfrom(2048)
                (cmd_code, status) = struct.unpack('<HH', msg[2:6])

                return (cmd_code, status)

            except Exception as e:
                self.get_logger().info(str(e))
                continue

    def configureDCA(self):
        if not self.dca_socket:
            self.get_logger().info("DCA1000EVM not connected")
            return

        # Set up DCA
        self.get_logger().info("Configuring DCA1000EVM")

        self._send_dca_cmd('SYSTEM_CONNECT_CMD_CODE')
        self._send_dca_cmd('READ_FPGA_VERSION_CMD_CODE')
        self._send_dca_cmd('CONFIG_FPGA_GEN_CMD_CODE')
        self._send_dca_cmd('CONFIG_PACKET_DATA_CMD_CODE')

        self.get_logger().info("Done configuring DCA1000EVM")
    
    def _send_dca_cmd(self, cmd_code):
        if not self.dca_socket:
            return
        self.get_logger().debug(f"Sending {cmd_code} to DCA1000EVM")
        self.dca_socket.sendto(self.DCA_CMDS[cmd_code], self.dca_cmd_addr)
        cmd, status = self.collect_response()
        self.get_logger().debug(f"Received (cmd_type, status): (0x{cmd:04x}, 0x{status:04x})")
        if status:
            self.get_logger().error(f"Error in sending {cmd_code} to DCA1000EVM")
        return status

    def start_raw_data_capture(self):
        self._send_dca_cmd('RECORD_START_CMD_CODE')
    
    def stop_raw_data_capture(self):
        self._send_dca_cmd('RECORD_STOP_CMD_CODE')

    def collect_data(self):
        try:
            msg, server = self.data_socket.recvfrom(2048)

        except Exception as e:
            self.get_logger().debug(f"Error in receiving data: {e}")
            return

        seqn, bytec = struct.unpack('<IIxx', msg[:10])

        self.data_array.pad_and_add_msg(self.seqn, seqn, np.frombuffer(msg[10:], dtype=np.int16))

        self.seqn = seqn
        self.bytec = bytec

    def publish_data(self):
        if self.capture_started:
            if self.data_array.queue.qsize() > 0:
                message = DataFrame()
                message.data = self.data_array.queue.get()
                message.header.frame_id = self.frame_id
                message.header.stamp = self.get_clock().now().to_msg()

                self.data_pub.publish(message)

    def close(self):
        self.stop_raw_data_capture()
        self.dca_socket.close()
        self.data_socket.close()


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(DCA1000Sensor())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
