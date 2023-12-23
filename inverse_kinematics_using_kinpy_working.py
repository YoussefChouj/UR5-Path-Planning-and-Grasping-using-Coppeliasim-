# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 18:14:54 2023

@author: youssef
"""

import numpy as np
import kinpy as kp
from path_planner_module_validation_default_scene_collision_detection import RobotManager
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from scipy.spatial.transform import Rotation as R


client = RemoteAPIClient()
sim = client.require('sim')

import numpy as np
import math

# Function to create a quaternion from an axis and angle
def create_quaternion(axis, angle_degrees):
    angle_radians = math.radians(angle_degrees) / 2
    return np.array([
        math.cos(angle_radians),
        math.sin(angle_radians) * axis[0],
        math.sin(angle_radians) * axis[1],
        math.sin(angle_radians) * axis[2]
    ])

# Function to multiply two quaternions
def multiply_quaternions(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

'''# Original quaternion
desired_quaternion_1 = np.array([1.0, 0.0, 0.0, 7.549790126404332e-08])

# Rotation quaternions for 60 degrees around X and Y axes
rotation_x = create_quaternion([1, 0, 0], 60)
rotation_y = create_quaternion([0, 1, 0], 60)

# Calculating new quaternions
new_quaternion_x = multiply_quaternions(rotation_x, desired_quaternion_1)
new_quaternion_y = multiply_quaternions(rotation_y, desired_quaternion_1)'''



def get_desired_target_gripper_quat_v2(object_euler_rot, des_picking_orientation):
    
    #print("object_euler_rot :",object_euler_rot)
    
    # Step 1: Define the desired orientation based on des_picking_orientation and object_quat
    if des_picking_orientation == "up":
        desired_quaternion_1 = np.array([1.0, 0.0, 0.0, 7.549790126404332e-08])  # (180, 0, 0) working 
        rotation_z = create_quaternion([0, 0, 1], 270)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)

       
      


    elif des_picking_orientation =="up_x_60":
        desired_quaternion_1 = np.array([1.0, 0.0, 0.0, 7.549790126404332e-08])  # (180, 0, 0) working 
        rotation_z = create_quaternion([0, 0, 1], 270)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)
        rotation_x = create_quaternion([1, 0, 0], 60)
        desired_quaternion_1 = multiply_quaternions(rotation_x, desired_quaternion_1)

    elif des_picking_orientation == "up_x_-60":
        desired_quaternion_1 = np.array([1.0, 0.0, 0.0, 7.549790126404332e-08])  # (180, 0, 0) working 
        rotation_z = create_quaternion([0, 0, 1], 270)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)
        rotation_x = create_quaternion([1, 0, 0], -60)
        desired_quaternion_1 = multiply_quaternions(rotation_x, desired_quaternion_1)

    elif des_picking_orientation == "up_y_60":
        desired_quaternion_1 = np.array([1.0, 0.0, 0.0, 7.549790126404332e-08])  # (180, 0, 0) working 
        rotation_z = create_quaternion([0, 0, 1], 270)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)
        rotation_z = create_quaternion([0, 1, 0], 60)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)

    elif des_picking_orientation == "up_y_-60":
        desired_quaternion_1 = np.array([1.0, 0.0, 0.0, 7.549790126404332e-08])  # (180, 0, 0) working 
        rotation_z = create_quaternion([0, 0, 1], 270)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)
        rotation_y = create_quaternion([0, 1, 0], -60)
        desired_quaternion_1 = multiply_quaternions(rotation_y, desired_quaternion_1)
      
    elif des_picking_orientation == "side_North":
        desired_quaternion_1 = np.array([0.7071067690849304,
         0.7071068286895752,
         2.980232594040899e-08,
         5.960465188081798e-08]) #working
 
    elif des_picking_orientation == "side_South":
        desired_quaternion_1 = np.array([0.7071067690849304,
         -0.7071068286895752,
         -2.980232594040899e-08,
         -5.960465188081798e-08])  
        
        
    elif des_picking_orientation == "side_West":
        desired_quaternion_1 = np.array([0.0, -0.7071068286895752, 0.0, 0.7071068286895752])  #east best
        rotation_x = create_quaternion([1, 0, 0], -270)
        desired_quaternion_1 = multiply_quaternions(rotation_x, desired_quaternion_1)
        rotation_z = create_quaternion([0, 0, 1], -90)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)
              
    elif des_picking_orientation == "side_East":
        desired_quaternion_1 = np.array([0.0, -0.7071068286895752, 0.0, 0.7071068286895752])  #east best
        rotation_x = create_quaternion([1, 0, 0], -270)
        desired_quaternion_1 = multiply_quaternions(rotation_x, desired_quaternion_1)
        rotation_z = create_quaternion([0, 0, 1], -270)
        desired_quaternion_1 = multiply_quaternions(rotation_z, desired_quaternion_1)


    alpha, beta, gamma = object_euler_rot
    rearranged_euler_rot = [-gamma, beta, -alpha]  # Re-arranged and negated 
    
    #print("rearranged_euler_rot:", rearranged_euler_rot)
    
    desired_quaternion_1 = R.from_quat(desired_quaternion_1)
   
    # Convert these Euler angles to a quaternion
    rearranged_quaternion = R.from_euler('xyz', rearranged_euler_rot).as_quat()
    
    # Step 3: Combine the two quaternions
    actual_desired_orient =  desired_quaternion_1 * R.from_quat(rearranged_quaternion)
    

    return actual_desired_orient.as_quat()


def Get_target_dummy_quaterion_and_position_From_Coppelliasim(des_picking_orrientation , new_target_dummy_handle):
    """
    Main function to get the transformation matrices of interest.
    """

    print("moving target to dummy with handle : ",new_target_dummy_handle)


    object_euler_rot = sim.getObjectOrientation(new_target_dummy_handle, sim.handle_world)

    object_pos = sim.getObjectPosition(new_target_dummy_handle, sim.handle_world)

    desired_quat_orient = get_desired_target_gripper_quat_v2(object_euler_rot, des_picking_orrientation)

    

    return object_pos, desired_quat_orient

# Function to wrap angles to 2pi
def wrap_to_2pi(angles):
    return np.mod(angles, 2*np.pi)

def transform_quaternion_2(quat):
    """
    Transform a quaternion from (w, x, y, z) to (x, y, z, w).

    :param quat: A quaternion in the format (w, x, y, z).
    :return: A quaternion in the format (x, y, z, w).
    """
    w, x, y, z = quat
    return x, y, z, w

robot_manager = RobotManager(sim)  

def compute_ik_configs_of_multip_pos_orientations(
    path_to_urdf_file,
    mapping_dict_forward_new,
    dict_of_all_pos_and_corr_robotconfigs_new,
    aut_def_condition, 
    all_orientation_keys ):
    
    # Build the kinematic chain from the URDF file
    chain = kp.build_serial_chain_from_urdf(
        open(path_to_urdf_file).read(),
        root_link_name="world",
        end_link_name="rg2_gripper",
    )
    print("Hii")


    # Function to update configs and compute IK
    
    def compute_and_store_ik(
        chain,
        orientation_key,
        dummy_name_key,
        config_list
    ):
        dummy_handle = sim.getObject(dummy_name_key)
        orien_keys = all_orientation_keys.copy()
        print("I  am in ,lets see waht we got ")
        print("dummy_handle",dummy_handle)
        print("aut_def_condition",aut_def_condition)
        if dummy_handle is not None and aut_def_condition:
            print("Save meee")
            for key in orien_keys :
        
                target_pos, target_quat = Get_target_dummy_quaterion_and_position_From_Coppelliasim(
                    key, dummy_handle
                )
                targetpose = np.concatenate([target_pos, target_quat]).tolist()
                #print(f"targetpose : {targetpose}")
                ret = kp.transform.Transform(rot=target_quat, pos=target_pos)
                ik_sol = chain.inverse_kinematics(ret, np.zeros(6))
                ik_sol = wrap_to_2pi(ik_sol)
                print("Hi")
                if robot_manager.configuration_validation_callback(ik_sol):
                    print("Hello")
                    config_list[0] = ik_sol
                    config_list[1] = key
                    config_list[3] = targetpose
                    break
        elif dummy_handle is not None and not aut_def_condition:
            target_pos, target_quat = Get_target_dummy_quaterion_and_position_From_Coppelliasim(
                orientation_key, dummy_handle
            )
            targetpose = np.concatenate([target_pos, target_quat]).tolist()
            ret = kp.transform.Transform(rot=target_quat, pos=target_pos)
            ik_sol = chain.inverse_kinematics(ret, np.zeros(6))
            ik_sol = wrap_to_2pi(ik_sol)
            if robot_manager.configuration_validation_callback(ik_sol):
                config_list[0] = ik_sol
                config_list[3] = targetpose
            else:
                for key in orien_keys :
            
                    target_pos, target_quat = Get_target_dummy_quaterion_and_position_From_Coppelliasim(
                        key, dummy_handle
                    )
                    ret = kp.transform.Transform(rot=target_quat, pos=target_pos)
                    ik_sol = chain.inverse_kinematics(ret, np.zeros(6))
                    ik_sol = wrap_to_2pi(ik_sol)
                    if robot_manager.configuration_validation_callback(ik_sol):
                        config_list[0] = ik_sol
                        config_list[1] = key
                        config_list[3] = targetpose
                        break


    # Loop through the forward mapping
    print("mapping_dict_forward_new:",mapping_dict_forward_new)
    for init_key, fin_key in mapping_dict_forward_new.items():
        init_list = dict_of_all_pos_and_corr_robotconfigs_new.get(init_key, None)
        print("init_list:",init_list)
        fin_list = dict_of_all_pos_and_corr_robotconfigs_new.get(fin_key, None)
        print("fin_list:",fin_list)
        if init_list is not None and fin_list is not None:
            print("Hiii")
            compute_and_store_ik(
                chain,
                init_list[1],
                init_list[2],
                init_list
            )
            compute_and_store_ik(
                chain,
                fin_list[1],
                fin_list[2],
                fin_list
            )


    return dict_of_all_pos_and_corr_robotconfigs_new

def transform_quaternion(quat):
    """
    Transform a quaternion from (x, y, z, w) to (w, x, y, z).

    :param quat: A quaternion in the format (x, y, z, w).
    :return: A quaternion in the format (w, x, y, z).
    """
    x, y, z, w = quat
    return w, x, y, z



def IK_Kinpy(path_to_urdf_file, pose):
    chain = kp.build_serial_chain_from_urdf(
    open(path_to_urdf_file).read(),
    root_link_name="world",
    end_link_name="rg2_gripper",
    )
    object_pos = pose[:3]
    object_quat = pose[3:]

    ret = kp.transform.Transform(rot=object_quat, pos=object_pos)
    ik_sol = chain.inverse_kinematics(ret, np.zeros(6))
    ik_sol = wrap_to_2pi(ik_sol)
    return ik_sol

