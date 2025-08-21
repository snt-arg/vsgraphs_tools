#!/usr/bin/env python3

"""
* This file is part of Visual S-Graphs (vS-Graphs).
* Copyright (C) 2023-2025 SnT, University of Luxembourg
*
* 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
*
* vS-Graphs is free software: you can redistribute it and/or modify it under the terms
* of the GNU General Public License as published by the Free Software Foundation, either
* version 3 of the License, or (at your option) any later version.
*
* This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details: https://www.gnu.org/licenses/
"""

import json
import time
import rclpy
import socket
import struct
import argparse
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker

HOST = "0.0.0.0"
VOXBLOX_PORT = 12345
POINTCLOUD_PORT = 12346
POINTCLOUD_CALLBACK_FREQ = 3.0  # seconds
POINTCLOUD_TOPIC = "/camera/depth/points"
VOXBLOX_TOPIC = "/voxblox_skeletonizer/sparse_graph"


class JazzyRelay_Client(Node):
    def __init__(self, host=HOST):
        # Create a ROS node
        super().__init__("jazzy_relay_client")
        self.get_logger().info(f"[Voxblox_Client] Starting Jazzy relay ...")
        self.get_logger().info(
            f"[Voxblox_Client] It receives Voxblox MarkerArray data via TCP socket and publishes as ROS2 Topic at {host}:{VOXBLOX_PORT}"
        )
        self.pub = self.create_publisher(MarkerArray, VOXBLOX_TOPIC, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.connect((host, VOXBLOX_PORT))
        self.sock.setblocking(False)
        self.buffer = ""
        self.get_logger().info(
            f"[Voxblox_Client] Connected to Noetic relay at {host}:{VOXBLOX_PORT}"
        )
        self.create_timer(0.1, self.receive_loop)

    def receive_loop(self):
        try:
            chunk = self.sock.recv(4096).decode("utf-8")
            self.buffer += chunk
        except BlockingIOError:
            pass
        # Process complete JSON lines
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if not line.strip():
                continue
            try:
                msg_dict = json.loads(line)
                marker_array = MarkerArray()
                for m in msg_dict.get("markers", []):
                    marker = Marker()
                    # Header
                    marker.header.frame_id = m["header"]["frame_id"]
                    marker.header.stamp.sec = m["header"]["stamp"]["sec"]
                    marker.header.stamp.nanosec = m["header"]["stamp"]["nanosec"]
                    # Basic fields
                    marker.ns = m["ns"]
                    marker.id = m["id"]
                    marker.type = m["type"]
                    marker.action = m["action"]
                    # Pose
                    marker.pose.position.x = m["pose"]["position"]["x"]
                    marker.pose.position.y = m["pose"]["position"]["y"]
                    marker.pose.position.z = m["pose"]["position"]["z"]
                    marker.pose.orientation.x = m["pose"]["orientation"]["x"]
                    marker.pose.orientation.y = m["pose"]["orientation"]["y"]
                    marker.pose.orientation.z = m["pose"]["orientation"]["z"]
                    marker.pose.orientation.w = m["pose"]["orientation"]["w"]
                    # Scale & color
                    marker.scale.x = m["scale"]["x"]
                    marker.scale.y = m["scale"]["y"]
                    marker.scale.z = m["scale"]["z"]
                    marker.color.r = m["color"]["r"]
                    marker.color.g = m["color"]["g"]
                    marker.color.b = m["color"]["b"]
                    marker.color.a = m["color"]["a"]
                    # Lifetime
                    marker.frame_locked = m["frame_locked"]
                    marker.lifetime.sec = m["lifetime"]["sec"]
                    marker.lifetime.nanosec = m["lifetime"]["nanosec"]
                    # Points
                    marker.points = [Point(x=p["x"], y=p["y"], z=p["z"]) for p in m.get("points", [])]
                    # Colors
                    marker.colors = [
                        ColorRGBA(r=c["r"], g=c["g"], b=c["b"], a=c["a"]) for c in m.get("colors", [])
                    ]
                    # Text & mesh
                    marker.text = m.get("text", "")
                    marker.mesh_resource = m.get("mesh_resource", "")
                    marker.mesh_use_embedded_materials = m.get("mesh_use_embedded_materials", False)
                    # Append to MarkerArray
                    marker_array.markers.append(marker)
                # Publish the MarkerArray
                self.pub.publish(marker_array)
                # Log
                self.get_logger().info(
                    f"[Voxblox_Client] Dictionary processed and published as ROS2 topic ..."
                )
            except json.JSONDecodeError:
                self.get_logger().warn("[Voxblox_Client] Failed to decode JSON line")

    def destroy_node(self):
        self.get_logger().info("[Voxblox_Client] Shutting down relay...")
        try:
            if hasattr(self, "sock"):
                self.sock.close()
        except Exception as e:
            self.get_logger().warn(f"[Voxblox_Client] Error closing socket: {e}")
        super().destroy_node()


class JazzyRelay_Server(Node):
    def __init__(self, host=HOST):
        super().__init__("jazzy_relay_server")
        self.get_logger().info(f"[PointCloud_Server] Starting Jazzy relay ...")
        self.get_logger().info(
            f"[PointCloud_Server] It subscribes to ROS2 Topic '{POINTCLOUD_TOPIC}' and publishes its data via TCP socket."
        )
        self.sub = self.create_subscription(
            PointCloud2, POINTCLOUD_TOPIC, self.callback, 10
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, POINTCLOUD_PORT))
        self.sock.listen(1)
        self.get_logger().info(
            f"[PointCloud_Server] Waiting for Noetic relay on {host}:{POINTCLOUD_PORT} ..."
        )
        self.last_sent = 0.0
        self.conn, addr = self.sock.accept()
        self.get_logger().info(
            f"[PointCloud_Server] Connected to Noetic relay at {addr}"
        )

    def callback(self, msg: PointCloud2):
        # Run callback only once per second
        now = time.time()
        if now - self.last_sent < POINTCLOUD_CALLBACK_FREQ:
            return
        self.last_sent = now
        # Prepare and send PointCloud2 data
        try:
            # Log the received PointCloud2 message
            self.get_logger().info(
                f"[PointCloud_Server] Sent PointCloud2 {msg.width}x{msg.height} ({len(msg.data)} bytes)"
            )
            header = struct.pack("III", msg.width, msg.height, len(msg.data))
            self.conn.sendall(header + msg.data)
        # Handle connection issues by reconnecting
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn(
                "[PointCloud_Server] Client disconnected, waiting for reconnection..."
            )
            try:
                self.conn.close()
            except:
                pass
            self.conn, addr = self.sock.accept()
            self.get_logger().info(f"[PointCloud_Server] Reconnected to {addr}")
        # Handle other exceptions
        except Exception as e:
            self.get_logger().error(
                f"[PointCloud_Server] Error sending PointCloud2: {e}"
            )

    def destroy_node(self):
        self.get_logger().info("[PointCloud_Server] Shutting down relay...")
        try:
            if hasattr(self, "conn"):
                self.conn.close()
            if hasattr(self, "sock"):
                self.sock.close()
        except Exception as e:
            self.get_logger().warn(f"[PointCloud_Server] Error closing socket: {e}")
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=["voxblox_client", "pc_server"],
        required=True,
        help="Run as Voxblox skeleton receiver (client) or PointCloud publisher (server)!",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="IP address of Noetic container (for Jazzy mode)!",
    )
    args = parser.parse_args()
    # Initialize ROS 2
    rclpy.init()
    if args.mode == "voxblox_client":
        node = JazzyRelay_Client(host=args.host)
    elif args.mode == "pc_server":
        node = JazzyRelay_Server(host=args.host)
    else:
        raise ValueError("Invalid mode! Use 'voxblox_client' or 'pc_server'.")
    # Spin the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
