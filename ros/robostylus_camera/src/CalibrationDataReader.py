#!/usr/bin/env python

import logging
import rospy
from std_msgs.msg import *
from robostylus_camera.srv import *

import json

FILENAME_PARAM = '~filename'
OPERATION_PARAM = '~operation'
CALIBRATION_TRANSFER_SERVICE_NAME = 'transfer_data'

def main():

    logging.basicConfig()

    rospy.init_node('calibration_reader')

    # Check for proper configuration
    if not rospy.has_param(FILENAME_PARAM):
        rospy.logerr('Filename parameter not set')
        quit()
    if not rospy.has_param(OPERATION_PARAM):
        rospy.logerr('Operation parameter not set')
        quit()

    # Get parameters
    filename = rospy.get_param(FILENAME_PARAM)
    operation = rospy.get_param(OPERATION_PARAM)

    try:
        # Create service calling helper
        rospy.wait_for_service(CALIBRATION_TRANSFER_SERVICE_NAME)
        transferData = rospy.ServiceProxy(CALIBRATION_TRANSFER_SERVICE_NAME, TransferCalibrationData)

        if operation == 'load':
            # Load dictionary from file
            with open(filename, 'r') as load_file:
                dct = json.load(load_file)

            # Validate data
            if not ('top_screen_x'    in dct and len(dct['top_screen_x'])    == 4 and 
                    'top_screen_y'    in dct and len(dct['top_screen_y'])    == 4 and 
                    'bottom_screen_x' in dct and len(dct['bottom_screen_x']) == 4 and
                    'bottom_screen_y' in dct and len(dct['bottom_screen_y']) == 4):
                rospy.logerr('File %s contains invalid data', filename)
                quit()

            # Call service with the data
            response = transferData(True, dct['top_screen_x'], dct['top_screen_y'], dct['bottom_screen_x'], dct['bottom_screen_y'])

            rospy.loginfo('Calibration settings loaded!')

        elif operation == 'save':
            # Transfer null data and receive current settings
            dummy = [0] * 4
            response = transferData(False, dummy, dummy, dummy, dummy)

            # Format response properly
            dct = {}
            for name in response.__slots__:
                dct[name] = response.__getattribute__(name)

            # Save data to file
            with open(filename, 'w') as save_file:
                json.dump(dct, save_file)

            rospy.loginfo('Calibration settings saved!')

        else:
            rospy.logerr('Invalid operation')
            quit()

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s", e)
    except IOError, e:
        rospy.logerr("IOError: %s", e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
