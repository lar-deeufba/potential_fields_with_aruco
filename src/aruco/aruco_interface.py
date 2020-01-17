#!/usr/bin/env python 
import sys
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

class ArucoInterface(object):

    def __init__(self):
        # Size of the ArUco marker in meters
        self.marker_size = 0.05
        rospy.init_node("aruco_pose_publisher", disable_signals=True)
        self.positionPublisher = rospy.Publisher("marker_pose", Point, queue_size=10)

    def checkCamera(self):
        """ Checks if the camera is available """
        cameraFound = False
        print("[INFO]: Searching for camera...")
        try:
            for camera in glob.glob("/dev/video?"):
                if camera == "/dev/video2":
                    cameraIndex = 2
                    cameraFound = True
                    print("[INFO]: Using index 2 for the camera.")
                    return cameraIndex, cameraFound
                elif camera == "/dev/video1":
                    cameraIndex = 1
                    cameraFound = True
                    print("[INFO]: Using index 1 for the camera.")
                    return cameraIndex, cameraFound
                elif camera == "/dev/video0":
                    cameraIndex = 0
                    cameraFound = True
                    print("[INFO]: Using index 0 for the camera")
                    return cameraIndex, cameraFound
                else:
                    print("[ERROR]: No camera found.")
                    cameraFound = False
                    cameraIndex = 0
                    return cameraIndex, cameraFound
        except(TypeError):
            print("[ERROR]: Camera is probably not connected.")

    def extract_calibration(self):
        """ Gets the the camera and distortion matrix from the calibrate_camera method by reading the yaml file. """
        #TODO add function to check if the folder exists because opencv points to other error rather than saying it doesnt exist
        cv_file = cv2.FileStorage("calib_images/calibration.yaml", cv2.FILE_STORAGE_READ)
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()
        print("[INFO]: Extracted camera parameters.")
        cv_file.release()
        return camera_matrix, dist_matrix

    def track_aruco(self):
        """ Tracks the ArUco Marker in real time. """
        marker_size = self.marker_size
        # Getting the parameters from the calibration
        camera_matrix, dist_matrix = self.extract_calibration()
        cameraIndex, foundCamera = self.checkCamera()
        try:
            cap = cv2.VideoCapture(cameraIndex)
            while (True and foundCamera):
                # Getting a frame from video stream
                ret, frame = cap.read()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Using the 6x6 dictionary from ArUco
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
                parameters = aruco.DetectorParameters_create()

                # Lists of ids and the corners belonging to each id
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                #  Just enters this condition if any id is found on the camera frame
                if np.all(ids is not None):
                    # Retrieves the rotation and translation vector, i.e orientation and position from the camera wrt. marker
                    rvec, tvec = aruco.estimatePoseSingleMarkers(corners[0], marker_size, camera_matrix, dist_matrix)
                    print 'Rotation Vector: ', rvec
                    print 'Translation Vector:', tvec
                    msgToPublish = Point()
                    msgToPublish.x = tvec[0][0][0]
                    msgToPublish.z = tvec[0][0][2]
                    self.positionPublisher.publish(msgToPublish)

        except(KeyboardInterrupt):
            print("Interrupt from keyboard, closing caption")
            cap.release()
            sys.exit(0)



ai = ArucoInterface()
ai.track_aruco()