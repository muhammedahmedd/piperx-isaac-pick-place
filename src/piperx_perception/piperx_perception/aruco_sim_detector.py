#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np
import cv2.aruco as aruco

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class ArucoSimDetector(Node):
    def __init__(self):
        super().__init__("aruco_sim_detector")

        self.bridge = CvBridge()

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # Physical marker side length in meters
        self.marker_size = 0.04

        self.camera_matrix = None
        self.dist_coeffs = None
        self.printed_camera_info = False
    
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/isaac/camera_info",
            self.camera_info_callback,
            10
        )

        self.subscription = self.create_subscription(
            Image,
            "/isaac/rgb_raw",
            self.image_callback,
            10
        )

        self.get_logger().info("Aruco sim detector started.")
        self.get_logger().info("Subscribing to /isaac/rgb_raw and /isaac/camera_info")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

        if not self.printed_camera_info:
            self.get_logger().info(
                f"Received camera intrinsics:\n{self.camera_matrix}"
            )
        self.printed_camera_info = True

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        debug_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imwrite("/workspace/piperx_ws/debug_frame.png", debug_frame)

        corners, ids, rejected = aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None:
            detected_ids = ids.flatten().tolist()
            self.get_logger().info(f"Detected ArUco marker IDs: {detected_ids}")
        else:
            return
        
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            self.get_logger().info(
                f"Marker {marker_id} tvec in camera optical frame: "
                f"x={tvec[0]:.4f} m, y={tvec[1]:.4f} m, z={tvec[2]:.4f} m"
            )

            self.get_logger().info(
                f"Marker {marker_id} rvec: "
                f"rx={rvec[0]:.4f}, ry={rvec[1]:.4f}, rz={rvec[2]:.4f}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = ArucoSimDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()