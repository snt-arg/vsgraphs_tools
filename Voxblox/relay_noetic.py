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
import argparse
from visualization_msgs.msg import MarkerArray

PORT = 12345
HOST = "0.0.0.0"
VOXBLOX_TOPIC = "/voxblox_skeletonizer/sparse_graph"


class NoeticRelay:
    def __init__(self, host=HOST):
        # Create a ROS node
        rospy.init_node("voxblox_noetic_relay", anonymous=False)
        # Subscribe to the get sparse graph of Voxblox
        self.sub = rospy.Subscriber(
            VOXBLOX_TOPIC, MarkerArray, self.callback, queue_size=10
        )
        # Create a TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, PORT))
        self.sock.listen(1)
        rospy.loginfo(f"[Voxblox_Noetic] Waiting for client on {host}:{PORT} ...")
        self.conn, addr = self.sock.accept()
        rospy.loginfo(f"[Voxblox_Noetic] Connected to client at {addr}!")
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg: MarkerArray):
        data = {"markers": []}
        rospy.loginfo(f"[Voxblox_Noetic] Processing {len(msg.markers)} markers ...")
        for m in msg.markers:
            data["markers"].append(
                {
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
                    "scale": {"x": m.scale.x, "y": m.scale.y, "z": m.scale.z},
                    "color": {
                        "r": m.color.r,
                        "g": m.color.g,
                        "b": m.color.b,
                        "a": m.color.a,
                    },
                }
            )
        # Send data to the client
        if hasattr(self, "conn") and self.conn:
            try:
                self.conn.sendall((json.dumps(data) + "\n").encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                rospy.logwarn("[Voxblox_Noetic] Lost connection to client!")

    def shutdown(self):
        rospy.loginfo("[Voxblox_Noetic] Shutting down relay...")
        try:
            if hasattr(self, "conn"):
                self.conn.close()
            if hasattr(self, "sock"):
                self.sock.close()
        except Exception as e:
            rospy.logwarn(f"[Voxblox_Noetic] Error closing socket: {e}")


if __name__ == "__main__":
    # Argument parser
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=["voxblox_send"],
        required=True,
        help="Run as Voxblox Skeleton Sender (server)!",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="IP address of Noetic container (for Jazzy mode)!",
    )
    args = parser.parse_args()
    # Initialize ROS
    if args.mode == "voxblox_send":
        node = NoeticRelay(host=args.host)
    else:
        raise ValueError("Invalid mode! Use 'voxblox_send'.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass