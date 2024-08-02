import numpy as np
from abc import ABC
from copy import deepcopy as copy
from .leap_kdl import LeapKDL
from openteach.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from openteach.utils.files import *
from openteach.utils.vectorops import *
import math
from openteach.utils.vectorops import coord_in_bound

MIDDLE_ROTATE_DEGREE = 10

class LeapKinematicControl(ABC):
    def __init__(self, bounded_angles = True):
        np.set_printoptions(suppress = True)

        # Loading the Allegro Hand configs
        self.hand_configs = get_yaml_data(get_path_in_package("robot/Leap/configs/leap_info.yaml"))
        self.finger_configs = get_yaml_data(get_path_in_package("robot/Leap/configs/leap_link_info.yaml"))
        self.bound_info = get_yaml_data(get_path_in_package("robot/Leap/configs/leap_bounds.yaml"))
        
        self.fingertip_threshold = 0.005
        self.time_steps = self.bound_info['time_steps']
        self.finger_tips_queue = {
            'thumb':[],
            'index':[],
            'ring':[],
            'middle':[]
        }

        self.bounded_angles = bounded_angles
        self.bounds = {}
        for finger in self.hand_configs['fingers'].keys():
            self.bounds[finger] = np.array(self.bound_info['jointwise_angle_bounds'][
                self.finger_configs['links_info'][finger]['offset'] : self.finger_configs['links_info'][finger]['offset'] + 4
            ])

    def _get_curr_finger_angles(self, curr_angles, finger_type):
        return np.array(curr_angles[
            self.finger_configs['links_info'][finger_type]['offset'] : self.finger_configs['links_info'][finger_type]['offset'] + 4
        ])


class LeapJointControl(LeapKinematicControl):
    def __init__(self, bounded_angles = True):
        super().__init__(bounded_angles)
        np.set_printoptions(suppress = True)

        self.linear_scaling_factors = self.bound_info['linear_scaling_factors']
        self.rotatory_scaling_factors = self.bound_info['rotatory_scaling_factors']

    def _get_filtered_angles(self, finger_type, calc_finger_angles, curr_angles, moving_avg_arr):
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)       
        desired_angles = np.array(copy(curr_angles))

        # Applying angular bounds
        if self.bounded_angles is True:
            #print(avg_finger_angles)
            #print(curr_finger_angles)
            del_finger_angles = avg_finger_angles - curr_finger_angles
            clipped_del_finger_angles = np.clip(del_finger_angles, - self.bounds[finger_type], self.bounds[finger_type])

            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] += clipped_del_finger_angles[idx]
        else:
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx]

        return desired_angles 

    def calculate_finger_angles(self, finger_type, finger_joint_coords, curr_angles, moving_avg_arr):
        translatory_angles = []
        for idx in range(self.hand_configs['joints_per_finger'] - 1): # Ignoring the rotatory joint
            angle = calculate_angle_yz_plane(
                finger_joint_coords[idx],
                finger_joint_coords[idx + 1],
                finger_joint_coords[idx + 2]
            )
            translatory_angles.append(angle * self.linear_scaling_factors[idx]+math.pi)

        rotatory_angle = [self.calculate_finger_rotation(finger_joint_coords,finger_type) * self.rotatory_scaling_factors[finger_type]+math.pi] 
        calc_finger_angles = rotatory_angle + translatory_angles
        filtered_angles = self._get_filtered_angles(finger_type, calc_finger_angles, curr_angles, moving_avg_arr)
        return filtered_angles

    def calculate_finger_rotation(self, finger_joint_coords,finger_type):
        straight_line = np.array([0,1])
        vector_tip_to_knucle = finger_joint_coords[-1] - finger_joint_coords[1]
        vector_tip_to_knucle = vector_tip_to_knucle[:2]
        angle = calculate_angle_between_vectors(straight_line,vector_tip_to_knucle)
        
        # Checking if the finger tip is on the left side or the right side of the knuckle
        #knuckle_vector = finger_joint_coords[1] - finger_joint_coords[0]
        #tip_vector = finger_joint_coords[-1] - finger_joint_coords[0]

        
        #knuckle_vector_slope = knuckle_vector[1] / knuckle_vector[0]
        #tip_vector_slope = tip_vector[1] / tip_vector[0]
        #print(finger_type,vector_tip_to_knucle[0])
        if vector_tip_to_knucle[0]>0:
            angle =  -1 * angle
        if finger_type == 'ring':
            return angle if angle>0 else 0
        elif finger_type == 'index':
            return angle if angle<0 else 0
        elif finger_type == 'middle':
            return max(-MIDDLE_ROTATE_DEGREE*math.pi/180, min(angle, MIDDLE_ROTATE_DEGREE*math.pi/180))
        else: 
            return angle


class LeapKDLControl(LeapKinematicControl):
    def __init__(self,  bounded_angles = True):
        super().__init__(bounded_angles)
        self.solver = LeapKDL()
        self.finger_joint_solver = LeapJointControl()
    def calculate_desired_angles(
        self, 
        finger_type, 
        transformed_coords, 
        moving_avg_arr, 
        curr_angles,
        hand_joint_angles
    ):
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)        
        avg_finger_coords = moving_average(transformed_coords, moving_avg_arr, self.time_steps)    
        calc_finger_angles = self.solver.finger_inverse_kinematics(finger_type, avg_finger_coords, curr_finger_angles,hand_joint_angles)
        
        desired_angles = np.array(copy(curr_angles))

        # Applying angular bounds
        if self.bounded_angles is True:
            del_finger_angles = calc_finger_angles - curr_finger_angles
            clipped_del_finger_angles = np.clip(del_finger_angles, - self.bounds[finger_type], self.bounds[finger_type])
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] += clipped_del_finger_angles[idx]
        else:
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = calc_finger_angles[idx]


        return desired_angles 

    def finger_1D_motion(
        self, 
        finger_type, 
        hand_y_val, 
        robot_x_val, 
        robot_y_val, 
        y_hand_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 1D control along the Z direction - used in index and middle fingers at a fixed depth and fixed y
        '''
        x_robot_coord = robot_x_val
        y_robot_coord = robot_y_val
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def finger_2D_motion(
        self, 
        finger_type, 
        hand_x_val, 
        hand_y_val, 
        robot_x_val, 
        x_hand_bound, 
        y_hand_bound, 
        y_robot_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in Y and Z directions - used in ring finger at a fixed depth
        '''
        x_robot_coord = robot_x_val
        y_robot_coord = linear_transform(hand_x_val, x_hand_bound, y_robot_bound)
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def finger_2D_depth_motion(
        self, 
        finger_type, 
        hand_y_val, 
        robot_y_val, 
        hand_z_val, 
        y_hand_bound, 
        z_hand_bound, 
        x_robot_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in X and Z directions - used in index and middle fingers at a varied depth
        '''
        x_robot_coord = linear_transform(hand_z_val, z_hand_bound, x_robot_bound)
        y_robot_coord = robot_y_val
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles


    def thumb_motion_2D(
        self, 
        hand_coordinates, 
        xy_hand_bounds, 
        yz_robot_bounds, 
        robot_x_val, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in Y and Z directions - human bounds are mapped to robot bounds
        '''
        y_robot_coord, z_robot_coord = persperctive_transform(
            (hand_coordinates[0], hand_coordinates[1]), 
            xy_hand_bounds, 
            yz_robot_bounds
        )

        x_robot_coord = robot_x_val        
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]
        
        desired_angles = self.calculate_desired_angles('thumb', transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def thumb_motion_3D(
        self, 
        hand_coordinates, 
        xy_hand_bounds, 
        yz_robot_bounds, 
        z_hand_bound, 
        x_robot_bound, 
        moving_avg_arr, 
        curr_angles,
        hand_joint_angle
    ):
        
        new_xy_hand_bounds = copy(xy_hand_bounds)
        new_yz_robot_bounds = copy(yz_robot_bounds)
        #print(xy_hand_bounds)
        top = xy_hand_bounds[0][1]
        bottom = xy_hand_bounds[1][1]
        middle = 0.023001949460838483
        robot_middle = 0.022658
        robot_top = yz_robot_bounds[0][1]
        robot_bottom = yz_robot_bounds[1][1]
        #print(yz_robot_bounds)
        #print(hand_coordinates[0])
        
        if hand_coordinates[0]<middle:
            new_xy_hand_bounds[0]=np.array([middle,top])
            new_xy_hand_bounds[1]=np.array([middle,bottom])
            new_yz_robot_bounds[0]=np.array([robot_middle,robot_top])
            new_yz_robot_bounds[1]=np.array([robot_middle,robot_bottom])
            y_robot_coord, x_robot_coord = persperctive_transform(
            (hand_coordinates[0], hand_coordinates[1]), 
            new_xy_hand_bounds, 
            new_yz_robot_bounds)
        else:  
            new_xy_hand_bounds[-1]=np.array([middle,top])
            new_xy_hand_bounds[2]=np.array([middle,bottom])
            new_yz_robot_bounds[-1]=np.array([robot_middle,robot_top])
            new_yz_robot_bounds[2]=np.array([robot_middle,robot_bottom])
            y_robot_coord, x_robot_coord = persperctive_transform(
            (hand_coordinates[0], hand_coordinates[1]), 
            new_xy_hand_bounds, 
            new_yz_robot_bounds
        )
        '''
        y_robot_coord, x_robot_coord = persperctive_transform(
            (hand_coordinates[0], hand_coordinates[1]), 
            xy_hand_bounds, 
            yz_robot_bounds
        )
        '''
        '''
        For 3D control in all directions - human bounds are mapped to robot bounds with varied depth
        '''
        
        z_robot_coord = linear_transform(hand_coordinates[2], z_hand_bound, x_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]
        transformed_coords = is_fingertip_stationary(self.finger_tips_queue['thumb'],transformed_coords,1,self.fingertip_threshold)
        desired_angles = self.calculate_desired_angles('thumb', transformed_coords, moving_avg_arr, curr_angles,hand_joint_angles=hand_joint_angle)
        return desired_angles
    

    def finger_3D_motion(
        self, 
        finger_type, 
        finger_keypoints,
        hand_bound, 
        robot_bound, 
        moving_avg_arr, 
        curr_angles,
    ):
        translatory_angles = []
        for idx in range(self.hand_configs['joints_per_finger'] - 1): # Ignoring the rotatory joint
            angle = calculate_angle_yz_plane(
                finger_keypoints[idx],
                finger_keypoints[idx + 1],
                finger_keypoints[idx + 2]
            )
            translatory_angles.append(angle * self.finger_joint_solver.linear_scaling_factors[idx])

        rotatory_angle = [self.finger_joint_solver.calculate_finger_rotation(finger_keypoints,finger_type) * self.finger_joint_solver.rotatory_scaling_factors[finger_type]] 
        calc_finger_angles = rotatory_angle + translatory_angles

        '''
        For 3D control in all directions - used in ring finger at a varied depth
        '''
        #print("finger_key point", finger_keypoints)
        #print("hand_bound",hand_bound)
        #print("robot_bound ",robot_bound)
        x_robot_coord = linear_transform(finger_keypoints[-1][0], hand_bound[1], robot_bound[0]['x_bounds'])
        y_robot_coord = linear_transform(finger_keypoints[-1][1], hand_bound[0], robot_bound[0]['y_bounds'])
        z_robot_coord = linear_transform(finger_keypoints[-1][2], hand_bound[2], robot_bound[0]['z_bounds'])
        transformed_coords = [y_robot_coord, x_robot_coord, z_robot_coord] 
        transformed_coords = is_fingertip_stationary(self.finger_tips_queue[finger_type],transformed_coords,1,self.fingertip_threshold)
        #print(finger_type," transformed_coord ",transformed_coords)
        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles,calc_finger_angles)
        #print("angle, ",desired_angles[0:4])
        return desired_angles