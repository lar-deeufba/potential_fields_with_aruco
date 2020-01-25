import cv2
import numpy as np
import glob

class CameraSetup(object):
    """ This class expects that you've already taken pictures to perform the calibration. 
    To ease the process, in our case the monocular calibration node from ROS was used (it creates a yaml file as well). 
    But the yaml file created from the node isn't compatible with the yaml parser in Python so it has to be redone. """

    def __init__(self, wait_time=10 , chessb_col=8, chessb_row=6):
        # If you want to see the pictures slowly then just change the WAIT_TIME
        # Our chessboard is composed by 9 rows and 7 columns! Which means only 8x6 corners would be detected

        self.wait_time = wait_time
        self.chessb_col = chessb_col
        self.chessb_row = chessb_row
        self.load_image = 'calib_images/*.png'
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def calibrate_camera(self):
        wait_time = self.wait_time
        chessb_col = self.chessb_col
        chessb_row = self.chessb_row
        load_image = self.load_image
        criteria = self.criteria

        # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessb_row*chessb_col,3), np.float32)
        objp[:,:2] = np.mgrid[0:chessb_col,0:chessb_row].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        images = glob.glob(load_image)

        for fname in images:
            print("loading %s" % fname)
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (chessb_col,chessb_row),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (chessb_col,chessb_row), corners2,ret)
                cv2.imshow('img',img)
                cv2.waitKey(wait_time)

        cv2.destroyAllWindows()
        print("Calculating the distortion and coefficients matrix...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        print("Saving the distortion and coefficients to calib_images/calibration.yaml")
        cv_file = cv2.FileStorage("calib_images/calibration.yaml", cv2.FILE_STORAGE_WRITE)
        cv_file.write("camera_matrix", mtx)
        cv_file.write("dist_coeff", dist)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()

cs = CameraSetup()
cs.calibrate_camera()