import sys
import numpy as np
from openteach.constants import *
from openteach.utils.files import *
from openteach.utils.network import ZMQKeypointSubscriber

class OculusThumbBoundCalibrator(object):
    def __init__(self, host, transformed_keypoints_port):
        # Initializing the keypoint subscriber
        self.transformed_keypoint_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = transformed_keypoints_port,
            topic = 'transformed_hand_coords'
        )

        # Storage paths
        make_dir(CALIBRATION_FILES_PATH)

    def _stop(self):
        print('Stopping the calibrator.')
        self.transformed_keypoint_subscriber.stop()

    def _get_thumb_tip_coord(self):
        return self.transformed_keypoint_subscriber.recv_keypoints()[OCULUS_JOINTS['thumb'][-1]]
    
    def _get_index_tip_coord(self):
        return self.transformed_keypoint_subscriber.recv_keypoints()[OCULUS_JOINTS['index'][-1]]
    
    def _get_middle_tip_coord(self):
        return self.transformed_keypoint_subscriber.recv_keypoints()[OCULUS_JOINTS['middle'][-1]]
    
    def _get_ring_tip_coord(self):
        return self.transformed_keypoint_subscriber.recv_keypoints()[OCULUS_JOINTS['ring'][-1]]
    
    
    def _get_index_xy_coords(self):
        return [self._get_index_tip_coord()[0], self._get_index_tip_coord()[1]]
    
    def _get_middle_xy_coords(self):
        return [self._get_middle_tip_coord()[0], self._get_middle_tip_coord()[1]]
    
    def _get_ring_xy_coords(self):
        return [self._get_ring_tip_coord()[0], self._get_ring_tip_coord()[1]]

    def _get_thumb_xy_coords(self):
        return [self._get_thumb_tip_coord()[0], self._get_thumb_tip_coord()[1]]
    
    def _get_index_z_coords(self):
        return self._get_index_tip_coord()[-1]
    
    def _get_middle_z_coords(self):
        return self._get_middle_tip_coord()[-1]
    
    def _get_ring_z_coords(self):
        return self._get_ring_tip_coord()[-1]

    def _get_thumb_z_coords(self):
        return self._get_thumb_tip_coord()[-1]

    def calibrate_finger(self,finger_type):
        coords = ['Right','Left', 'top' , 'bottom']
        result = []
        
        _ = input("Place your {} in the most right.".format(finger_type))
        if finger_type =="index":
            right = self._get_index_xy_coords()[0]
        elif finger_type =="middle":
            right = self._get_middle_xy_coords()[0]
        elif finger_type == "ring":
            right = self._get_ring_xy_coords()[0]

        _ = input("Place your {} in the most left.".format(finger_type))
        if finger_type =="index":
            left = self._get_index_xy_coords()[0]
        elif finger_type =="middle":
            left = self._get_middle_xy_coords()[0]
        elif finger_type == "ring":
            left = self._get_ring_xy_coords()[0]

        _ = input("Place your {} in the most top.".format(finger_type))
        if finger_type =="index":
            top = self._get_index_xy_coords()[1]
        elif finger_type =="middle":
            top = self._get_middle_xy_coords()[1]
        elif finger_type == "ring":
            top = self._get_ring_xy_coords()[1]

        _ = input("Place your {} in the most bottom.".format(finger_type))
        if finger_type =="index":
            bottom = self._get_index_xy_coords()[1]
        elif finger_type =="middle":
            bottom = self._get_middle_xy_coords()[1]
        elif finger_type == "ring":
            bottom = self._get_ring_xy_coords()[1]
          
        _ = input("Place your {} in the lowest z corner.".format(finger_type))
        if finger_type =="index":
            lowest = np.array(self._get_index_z_coords())
        elif finger_type =="middle":
            lowest = np.array(self._get_middle_z_coords())
        elif finger_type == "ring":
            lowest = np.array(self._get_ring_z_coords())

        _ = input("Place your {} in the highest z corner.".format(finger_type))
        if finger_type =="index":
            highest = np.array(self._get_index_z_coords())
        elif finger_type =="middle":
            highest = np.array(self._get_middle_z_coords())
        elif finger_type == "ring":
            highest = np.array(self._get_ring_z_coords())

        result.append(np.array([bottom,top]))
        result.append(np.array([left,right]))
        result.append(np.array([lowest,highest]))
        return np.array(result)
        


    def _calibrate(self):
        _ = input("Place the thumb in the top right corner.")
        top_right_coord = np.array(self._get_thumb_xy_coords())

        _ = input("Place the thumb in the bottom right corner.")
        bottom_right_coord = np.array(self._get_thumb_xy_coords())

        _ = input("Place the thumb in the ring bottom corner.")
        ring_bottom_coord = np.array(self._get_thumb_xy_coords())

        _ = input("Place the thumb in the ring top corner.")
        ring_top_coord = np.array(self._get_thumb_xy_coords())

        '''
        _ = input("Place the thumb in the index bottom corner.")
        index_bottom_coord = self._get_thumb_xy_coords()

        _ = input("Place the thumb in the index top corner.")
        index_top_coord = self._get_thumb_xy_coords()

        _ = input("Stretch the thumb to get highest index bound z value.")
        index_high_z = self._get_thumb_z_coords()

        _ = input("Relax the thumb to get the lowest index bound z value.")
        index_low_z = self._get_thumb_z_coords()

        _ = input("Place the thumb in the middle bottom corner.")
        middle_bottom_coord = self._get_thumb_xy_coords()

        

        _ = input("Stretch the thumb to get highest middle bound z value.")
        middle_high_z = self._get_thumb_z_coords()

        _ = input("Relax the thumb to get the lowest middle bound z value.")
        middle_low_z = self._get_thumb_z_coords()

        
        '''
        _ = input("Straight the thumb to get toppest y bound")
        highest_y_coord = self._get_thumb_xy_coords()


        _ = input("Stretch the thumb to get highest z bound.")
        ring_high_z = self._get_thumb_z_coords()

        _ = input("Relax the thumb to get the lowest z value.")
        ring_low_z = self._get_thumb_z_coords()

        ring_top_coord[1] = highest_y_coord[1]+0.01
        top_right_coord[1]= highest_y_coord[1]+0.01
        ring_z = np.array([ring_low_z,ring_high_z])

        index_bound = self.calibrate_finger('index')
        middle_bound = self.calibrate_finger('middle')
        ring_bound = self.calibrate_finger('ring')

        index_bound = np.array([
            [index_bound[0][0],index_bound[0][1]],
            [index_bound[1][0],index_bound[1][1]],
            [index_bound[2][0],index_bound[2][1]]
        ])

        middle_bound = np.array([
            [middle_bound[0][0],middle_bound[0][1]],
            [middle_bound[1][0],middle_bound[1][1]],
            [middle_bound[2][0],middle_bound[2][1]]
        ])

        ring_bound = np.array([
            [ring_bound[0][0],ring_bound[0][1]],
            [ring_bound[1][0],ring_bound[1][1]],
            [ring_bound[2][0],ring_bound[2][1]]
        ])

        '''
        thumb_index_bounds = np.array([
            top_right_coord,
            bottom_right_coord,
            index_bottom_coord,
            index_top_coord,
            [index_low_z, index_high_z]
        ])

        thumb_middle_bounds = np.array([
            index_top_coord,
            index_bottom_coord,
            middle_bottom_coord,
            middle_top_coord,
            [middle_low_z, middle_high_z]
        ])

        thumb_ring_bounds = np.array([
            middle_top_coord,
            middle_bottom_coord,
            ring_bottom_coord,
            ring_top_coord,
            [ring_low_z, ring_high_z]
        ])
        '''
        thumb_bounds = np.vstack((top_right_coord,bottom_right_coord,ring_bottom_coord,ring_top_coord,ring_z))

        finger_bound = np.vstack((index_bound,middle_bound,ring_bound))
        handpose_coords = np.vstack((top_right_coord,bottom_right_coord,ring_bottom_coord,ring_top_coord))

        np.save(VR_DISPLAY_THUMB_BOUNDS_PATH, handpose_coords)
        
        np.save(VR_THUMB_BOUNDS_PATH, thumb_bounds)

        np.save(VR_FINGER_BOUNDS_PATH,finger_bound)

        return thumb_bounds,finger_bound

    def get_bounds(self):
        sys.stdin = open(0) # To take inputs while spawning multiple processes

        if check_file(VR_THUMB_BOUNDS_PATH):
            use_calibration_file = input("\nCalibration file already exists. Do you want to create a new one? Press y for Yes else press Enter")

            if use_calibration_file == "y":
                thumb_bounds,finger_bound = self._calibrate()
            else:
                calibrated_bounds = np.load(VR_THUMB_BOUNDS_PATH)
                thumb_bounds= calibrated_bounds
                finger_bound = np.load(VR_FINGER_BOUNDS_PATH)
        else:
            print("\nNo calibration file found. Need to calibrate hand poses.\n")
            thumb_bounds,finger_bound = self._calibrate()
        self._stop()
        #return thumb_index_bounds, thumb_middle_bounds, thumb_ring_bounds #NOTE: We're changing this to only return the ring top/botton and right top/bottom

        # Return one bound with the max of the zs as the z bound - NOTE: Here, we're getting the largest limits
        high_z = thumb_bounds[-1][1]
        low_z = thumb_bounds[-1][0]

        # Get the four bounds from index and the ring bounds
        thumb_bounds = [
            thumb_bounds[0],
            thumb_bounds[1],
            thumb_bounds[2], 
            thumb_bounds[3],
            [low_z, high_z]
        ]

        index_bounds = [
            finger_bound[0],
            finger_bound[1],
            finger_bound[2],
        ]
        middle_bounds = [
            finger_bound[3],
            finger_bound[4],
            finger_bound[5],
        ]

        ring_bounds = [
            finger_bound[6],
            finger_bound[7],
            finger_bound[8]
        ]

        finger_bounds = {
            'index': index_bounds,
            'middle': middle_bounds,
            'ring': ring_bounds
        }

        #print(thumb_bounds)
        return thumb_bounds,finger_bounds