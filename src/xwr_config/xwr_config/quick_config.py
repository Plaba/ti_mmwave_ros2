#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from ti_mmwave_msgs.srv import MMWaveCLI
from ti_mmwave_msgs.msg import MMWaveConfig
from .radar_config import RadarConfig

SERVICE_NAME = "mmWaveCLI"
CFG_TOPIC_NAME = "mmWaveCFG"


def get_config(path:str) -> RadarConfig:
    with open(path, 'r') as f:
        config = f.read()
        return RadarConfig(config)


def send_config_to_xwr(node:Node, config:RadarConfig) -> None:
    client = node.create_client(MMWaveCLI, SERVICE_NAME)
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().debug('service not available, waiting again...')
    
    for cmd in config.commands:
        req = MMWaveCLI.Request()
        req.comm = str(cmd)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        if result is not None:
            node.get_logger().debug(f"Service call success {result.resp}")
            if "Done" not in result.resp:
                node.get_logger().fatal(f"Command '{req}' to XWR failed: {cmd.resp}")
                node.get_logger().fatal("Exiting...")
                rclpy.shutdown()
                return
        else:
            node.get_logger().error("Service call failed %r" % (future.exception(),))


def main(args=None):
    rclpy.init(args=args)

    node = Node('quick_config')
    cfg_pub = node.create_publisher(MMWaveConfig, CFG_TOPIC_NAME, 10)

    override_enable_lvds = node.declare_parameter("override_enable_lvds", False).value
    node.get_logger().info("override_enable_lvds: %s" % override_enable_lvds)
    cfg_path = node.declare_parameter("cfg_path", "").value

    if cfg_path == "":
        node.get_logger().fatal("No config file specified")
        rclpy.shutdown()
        return
    
    config = get_config(cfg_path)

    if override_enable_lvds:
        node.get_logger().info("Enabling LVDS stream because of override_enable_lvds")
        config.enable_lvds_stream()

    send_config_to_xwr(node, config)

    cfg_pub.publish(config.to_msg())
    node.get_logger().info("Published config to topic %s" % CFG_TOPIC_NAME)
    
    node.get_logger().info("Finished XWR configuration. Exiting...")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
