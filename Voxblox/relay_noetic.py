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
import rospy
import socket
import struct
import argparse
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2, PointField

HOST = "0.0.0.0"
VOXBLOX_PORT = 12345
POINTCLOUD_PORT = 12346
# Cam.Frame: "/camera/depth/points", World Frame: "/vs_graphs/points_map"
POINTCLOUD_TOPIC = "/vs_graphs/points_map"
VOXBLOX_TOPIC = "/voxblox_skeletonizer/sparse_graph"


class NoeticRelay_Server:
    def __init__(self, host=HOST):
        # Create a ROS node
        rospy.init_node("noetic_relay_server", anonymous=False)
        rospy.loginfo("[Voxblox_Server] Starting Noetic relay ...")
        rospy.loginfo(
            f"[Voxblox_Server] It subscribes to ROS1 Topic '{VOXBLOX_TOPIC}' and publishes its data via TCP socket."
        )
        # Subscribe to the get sparse graph of Voxblox
        self.sub = rospy.Subscriber(
            VOXBLOX_TOPIC, MarkerArray, self.callback, queue_size=10
        )
        # Create a TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, VOXBLOX_PORT))
        self.sock.listen(1)
        rospy.loginfo(
            f"[Voxblox_Server] Waiting for Jazzy relay on {host}:{VOXBLOX_PORT} ..."
        )
        self.conn, addr = self.sock.accept()
        rospy.loginfo(f"[Voxblox_Server] Connected to Jazzy relay at {addr}!")
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg: MarkerArray):
        data = {"markers": []}
        # Process each marker in the MarkerArray to match the expected format
        for m in msg.markers:
            marker_dict = {
                "header": {
                    "stamp": {
                        "sec": m.header.stamp.secs,
                        "nanosec": m.header.stamp.nsecs,
                    },
                    "frame_id": m.header.frame_id,
                },
                "ns": m.ns,
                "id": m.id,
                "type": m.type,
                "action": m.action,
                "pose": {
                    "position": {
                        "x": m.pose.position.x,
                        "y": m.pose.position.y,
                        "z": m.pose.position.z,
                    },
                    "orientation": {
                        "x": m.pose.orientation.x,
                        "y": m.pose.orientation.y,
                        "z": m.pose.orientation.z,
                        "w": m.pose.orientation.w,
                    },
                },
                "scale": {
                    "x": m.scale.x,
                    "y": m.scale.y,
                    "z": m.scale.z,
                },
                "color": {
                    "r": m.color.r,
                    "g": m.color.g,
                    "b": m.color.b,
                    "a": m.color.a,
                },
                "lifetime": {
                    "sec": m.lifetime.secs,
                    "nanosec": m.lifetime.nsecs,
                },
                "frame_locked": m.frame_locked,
                "points": [{"x": p.x, "y": p.y, "z": p.z} for p in m.points],
                "colors": [{"r": c.r, "g": c.g, "b": c.b, "a": c.a} for c in m.colors],
                "text": m.text,
                "texture_resource": "",
                "texture": {
                    "header": {
                        "stamp": {
                            "sec": m.header.stamp.secs,
                            "nanosec": m.header.stamp.nsecs,
                        },
                        "frame_id": m.header.frame_id,
                    },
                    "format": "",
                    "data": [],
                },
                "uv_coordinates": [],
                "mesh_resource": m.mesh_resource,
                "mesh_file": {
                    "filename": "",
                    "data": [],
                },
                "mesh_use_embedded_materials": m.mesh_use_embedded_materials,
            }
            # Append marker to the data dictionary
            data["markers"].append(marker_dict)
        # Send data to the client
        if hasattr(self, "conn") and self.conn:
            try:
                rospy.loginfo(f"[Voxblox_Server] Sending the generated dictionary ...")
                self.conn.sendall((json.dumps(data) + "\n").encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                rospy.logwarn("[Voxblox_Server] Lost connection to client!")

    def shutdown(self):
        rospy.loginfo("[Voxblox_Server] Shutting down Noetic relay ...")
        try:
            if hasattr(self, "conn"):
                self.conn.close()
            if hasattr(self, "sock"):
                self.sock.close()
        except Exception as e:
            rospy.logwarn(f"[Voxblox_Server] Error closing socket: {e}")


class NoeticRelay_Client:
    def __init__(self, host=HOST):
        # Create a ROS node
        rospy.init_node("noetic_relay_client", anonymous=False)
        rospy.loginfo("[PointCloud_Client] Starting Noetic relay ...")
        rospy.loginfo(
            f"[PointCloud_Client] It receives PointCloud data via TCP socket and publishes as ROS1 Topic '{POINTCLOUD_TOPIC}'."
        )
        # Create ROS publisher
        self.pub = rospy.Publisher(POINTCLOUD_TOPIC, PointCloud2, queue_size=10)
        # Create a TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.connect((host, POINTCLOUD_PORT))
        self.sock.setblocking(False)
        self.buffer = ""
        rospy.loginfo(
            f"[PointCloud_Client] Connected to Jazzy relay at {host}:{POINTCLOUD_PORT}"
        )

    def spin(self):
        rate = rospy.Rate(100)
        headerSize = struct.calcsize("IIII??I")
        while not rospy.is_shutdown():
            try:
                # Read header
                header = b""
                while len(header) < headerSize:
                    packet = self.sock.recv(headerSize - len(header))
                    if not packet:
                        rospy.logwarn("[PointCloud_Client] Connection closed by server")
                        return
                    header += packet
                (
                    width,
                    height,
                    point_step,
                    row_step,
                    is_dense,
                    is_bigendian,
                    n_fields,
                ) = struct.unpack("IIII??I", header)
                # Read fields
                fields = []
                field_size = struct.calcsize("32sIII")
                for _ in range(n_fields):
                    field_bytes = b""
                    while len(field_bytes) < field_size:
                        packet = self.sock.recv(field_size - len(field_bytes))
                        if not packet:
                            return
                        field_bytes += packet
                    name_bytes, offset, datatype, count = struct.unpack(
                        "32sIII", field_bytes
                    )
                    name = name_bytes.decode("utf-8", errors="ignore").rstrip("\0")
                    fields.append(
                        PointField(
                            name=name, offset=offset, datatype=datatype, count=count
                        )
                    )
                # Read data blob
                data_size = row_step * height
                data = b""
                while len(data) < data_size:
                    packet = self.sock.recv(data_size - len(data))
                    if not packet:
                        return
                    data += packet
                # Reconstruct PointCloud2
                pc2_msg = PointCloud2()
                pc2_msg.header.stamp = rospy.Time.now()
                pc2_msg.header.frame_id = "map"
                pc2_msg.width = width
                pc2_msg.height = height
                pc2_msg.is_dense = is_dense
                pc2_msg.point_step = point_step
                pc2_msg.is_bigendian = is_bigendian
                pc2_msg.row_step = row_step
                pc2_msg.data = data
                pc2_msg.fields = fields
                # If width or height is zero, skip publishing
                if width == 0 or height == 0:
                    rospy.logwarn(
                        f"[PointCloud_Client] Received empty PointCloud2 ({width}x{height}), skipping publish!"
                    )
                    continue
                # Check for corrupted header values
                if not (1 <= width <= 4096 and 1 <= height <= 4096):
                    rospy.logwarn(
                        f"[PointCloud_Client] Skipping potentially corrupted PointCloud2 ({width}x{height}), skipping publish!"
                    )
                    continue
                # Publish PointCloud2 message
                self.pub.publish(pc2_msg)
                rospy.loginfo(
                    f"[PointCloud_Client] Published PointCloud2 {width}x{height} ({len(data)} bytes)"
                )
            except BlockingIOError:
                # Avoid busy-waiting and just continue if no data is available
                pass
            except Exception as e:
                rospy.logwarn(f"[PointCloud_Client] Failed to parse message: {e}")
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("[PointCloud_Client] Shutting down client...")
        try:
            if hasattr(self, "sock"):
                self.sock.close()
        except Exception as e:
            rospy.logwarn(f"[PointCloud_Client] Error closing socket: {e}")


def main():
    # Argument parser
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=["voxblox_server", "pc_client"],
        required=True,
        help="Run as Voxblox skeleton publisher (server) or PointCloud receiver (client)!",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="IP address of Noetic container (for Jazzy mode)!",
    )
    args = parser.parse_args()
    try:
        # Initialize ROS
        if args.mode == "voxblox_server":
            NoeticRelay_Server(host=args.host)
            rospy.spin()
        elif args.mode == "pc_client":
            client = NoeticRelay_Client(host=args.host)
            client.spin()
        else:
            raise ValueError("Invalid mode! Use 'voxblox_server' or 'pc_client'.")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if client:
                client.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
