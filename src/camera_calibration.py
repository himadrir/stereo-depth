#! /usr/bin/env python
import numpy as np
import cv2 as cv
import glob
import rospy
import os
import rospkg


def getCameraCalibrationMatrix():


    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    r = rospkg.RosPack()
    path = r.get_path('lab7')
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob(path+'/images/*.png')
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (9,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            # Uncomment to view project on images
            # cv.drawChessboardCorners(img, (9,6), corners2, ret)
            # cv.imshow('img', img)
            # cv.waitKey(0)
    # cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    rospy.loginfo(mtx)

if __name__ == '__main__':
    rospy.init_node('camera_calibration', anonymous = True)
    getCameraCalibrationMatrix()
    rospy.spin()
