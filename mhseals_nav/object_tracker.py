#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

class ObjectTracker:
    def __init__(self, initial_position, track_id):
        self.id = track_id
        self.missed_frames = 0
        
        # 6 state variables: x, y, z, vx, vy, vz
        # 3 measurement variables: x, y, z
        self.kf = cv2.KalmannFilter(6, 3)
        
        dt = 1.0
        
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ], dtype=np.float32)
        
        self.kf.measurementMatrix = np.zeros((3, 6), np.float32)
        self.kf.measurementMatrix[0, 0] = 1
        self.kf.measurementMatrix[1, 1] = 1
        self.kf.measurementMatrix[2, 2] = 1