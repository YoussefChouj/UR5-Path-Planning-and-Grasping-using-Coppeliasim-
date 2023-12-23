
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from  inverse_kinematics_using_kinpy_working import compute_ik_configs_of_multip_pos_orientations, IK_Kinpy
from path_planner_module_validation_default_scene_collision_detection import PathPlanner, RobotManager , AngleBoundsCalculator

client = RemoteAPIClient()
sim = client.require('sim')
simOMPL = client.require('simOMPL')






# Calculate initial and final joint positions using inverse kinematics using kinpy library 
# of all the objects from the scenes 


# List of object positions names in CoppeliaSim
object_names = [
    '/DummyTb1Pos1', '/DummyTb1Pos2', '/DummyTb1Pos3',
    '/DummyTb1Pos4', '/DummyTb1Pos5', '/DummyTb1Pos6',
    '/DummyTb2Pos1', '/DummyTb2Pos2', '/DummyTb2Pos3',
    '/DummyTb2Pos4', '/DummyTb2Pos5', '/DummyTb2Pos6',
]

# Initialize an empty dictionary to hold object names and their handles
objects_dict = {}

# Loop through each object name to get its handle
for obj_name in object_names:
   #print(obj_name)
   obj_handle = sim.getObject(obj_name)
   objects_dict[obj_name] = obj_handle



# Now objects_dict contains the object names as keys and their handles as values
#print("Object Handles Dictionary:", objects_dict)



class CustomMapping:
    def __init__(self, default_forward, default_values, object_names_aliases):
        self.default_forward = default_forward
        self.values = default_values
        self.modified_values = None
        self.aut_def_cond = None
        self.dict_Dummies_to_object_name_mapping = {}
        self.default_mapping_chosen = False
        self.custom_mapping_chosen = False
        self.object_names_aliases = object_names_aliases

    def setup(self):
        use_default = input("Do you want to use default mapping? (yes/no): ")
        if use_default.lower() == 'yes':
            self.setup_default()         
        else:
            self.setup_custom()
        self.setup_orientation()
        self.modified_values = self.values
        

    def setup_default(self):
        self.mapping_dict_forward = {}
        self.mapping_dict_reverse = {}
        for k, v in self.default_forward.items():
            init_key = next(key for key, val in self.values.items() if val[2] == k)
            fin_key = next(key for key, val in self.values.items() if val[2] == v)
            self.mapping_dict_forward[init_key] = fin_key
            self.mapping_dict_reverse[fin_key] = init_key
        self.default_mapping_chosen = True
        self.update_object_name_mapping()

    def setup_custom(self):
        self.mapping_dict_forward = {}
        for k, v in self.values.items():
            if 'init' in k:
                corresponding_key = input(f"For {k}, specify the corresponding 'fin' key: ")
                self.mapping_dict_forward[k] = corresponding_key
        # Automatically generate the reverse mapping
        self.mapping_dict_reverse = {v: k for k, v in self.mapping_dict_forward.items()}
        self.custom_mapping_chosen = True
        self.update_object_name_mapping(self.object_names_aliases)

    def setup_orientation(self):
        change_orientation = input("Do you want to change the default picking orientations or use automatic feasible ones it is recomended to use automated found ones  ? (yes/no): ")
        if change_orientation.lower() == 'yes':
            for k, v in self.values.items():
                new_orientation = input(f"New orientation for {k} please choose from ('up','up_x_60','up_x_-60','up_y_60','up_y_-60', 'side_North','side_South', 'side_West', 'side_East'): ")
                self.values[k][1] = new_orientation
            self.aut_def_cond = False
        else:
           self.aut_def_cond = True 

    def setup_cond_values(self):
        for k in self.values.keys():
            if self.dict_Dummies_to_object_name_mapping[self.values[k][2]] == None:
                self.values[k][-1] = False
            else:
                self.values[k][-1] = True 

    def update_object_name_mapping(self):
        if self.default_mapping_chosen:
            for i, obj_name in enumerate(self.object_names_aliases):
                self.dict_Dummies_to_object_name_mapping[f'/DummyTb2Pos{i+1}'] = obj_name
                self.dict_Dummies_to_object_name_mapping[f'/DummyTb1Pos{i+1}'] = None
        elif self.custom_mapping_chosen:
            # Update based on custom mapping
            for init_key, fin_key in self.mapping_dict_forward.items():
                init_pos = self.values[init_key][2]
                fin_pos = self.values[fin_key][2]
                obj_name = self.dict_Dummies_to_object_name_mapping[fin_pos]
                self.dict_Dummies_to_object_name_mapping[init_pos] = obj_name
                self.dict_Dummies_to_object_name_mapping[fin_pos] = None
        self.setup_cond_values()


# Initialize config arrays
configs = [np.zeros(6) for _ in range(12)]
# Initialize and setup
mapping_dict_forward = {
    '/DummyTb2Pos1': '/DummyTb1Pos1',
    '/DummyTb2Pos2': '/DummyTb1Pos2',
    '/DummyTb2Pos3': '/DummyTb1Pos3',
    '/DummyTb2Pos4': '/DummyTb1Pos4',
    '/DummyTb2Pos5': '/DummyTb1Pos5',
    '/DummyTb2Pos6': '/DummyTb1Pos6',
}


dict_of_all_pos_and_corr_robotconfigs = {
    f'init{i+1}': [configs[i], "up", f'/DummyTb2Pos{i+1}',  None, None] for i in range(6)
} | {
    f'fin{i+1}': [configs[i+6], "up", f'/DummyTb1Pos{i+1}', None, None] for i in range(6)
}

object_names_alliases = [
    '/Cuboid_1', '/Cuboid_2', '/Cuboid_3', 
    '/Cylinder_4', '/Sphere_5', '/Sphere_6'
]




cm = CustomMapping(mapping_dict_forward, dict_of_all_pos_and_corr_robotconfigs,object_names_alliases)
cm.setup()



mapping_dict_forward_new = cm.mapping_dict_forward 

dict_of_all_pos_and_corr_robotconfigs_new = cm.modified_values 
aut_def_condition =cm.aut_def_cond

dict_Dummies_to_object_name_mapping = cm.dict_Dummies_to_object_name_mapping
  

print ("dict_of_all_pos_and_corr_robotconfigs_new", dict_of_all_pos_and_corr_robotconfigs_new)

print ("mapping_dict_forward_new:",mapping_dict_forward_new)


all_orientation_keys = ["up","up_x_60","up_x_-60","up_y_60","up_y_-60", "side_North", "side_South", "side_West", "side_East"]

#---------------------------

path_to_ur5_urdf_file = "C:\\Users\\youssef\\UR5_robot_machine_learning\\UR5_RG2\\ur5_robot_without_limits.urdf.xml"

dict_of_all_ik_computed = compute_ik_configs_of_multip_pos_orientations(path_to_ur5_urdf_file,\
                            mapping_dict_forward_new, \
                               dict_of_all_pos_and_corr_robotconfigs_new, aut_def_condition,
                               all_orientation_keys)

print ("dict_of_all_ik_computed:", dict_of_all_ik_computed)

print("dict_Dummies_to_object_name_mapping:",dict_Dummies_to_object_name_mapping)


#------------------------------- IKpath generation 


# Straight path generation
Robot_manager = RobotManager(sim)
angle_bounding = AngleBoundsCalculator()

def generate_straight_path(object_name, object_name_next,dummy_name,pose, pathPointCount, Robot_manager,prev_path_conf):
    # Ensure pathPointCount is a positive integer
    print(f"pose[2]: {pose[2]}")
    if pathPointCount <= 0:
        raise ValueError("pathPointCount must be a positive integer")
    
    obj_hand = sim.getObject(object_name)
    dum_hand = sim.getObject(dummy_name)
    if object_name_next is  None:

        dum_hand = sim.getObject(dummy_name)
        obj_pos = sim.getObjectPosition(obj_hand, sim.handle_world)
        dum_pos = sim.getObjectPosition(dum_hand, sim.handle_world)
        size = sim.getShapeBB(obj_hand)

        # Calculate the desired z position of the gripper fingers.
        desired_z_position = obj_pos[2] + size[2]/2  - 0.030

        # Calculate the final z position of the gripper TCP.
        rel_dis = abs(desired_z_position - dum_pos[2])


    else:
        z = 0.41
        size = sim.getShapeBB(obj_hand) 
        rel_dis = pose[2] - (z+size[-1] - 0.030)
    rel_dis = rel_dis - 0.02
    print(f"pose[2]: {pose[2]}")


    # Calculate the step size for each point in the path
    step = rel_dis / pathPointCount
    print(f"step: {step}")

    # Initialize the straight path
    straightPath = []
    current_pose = pose.copy()  # Use a copy to avoid modifying the original pose

    # Generate the path
    print("\n IK path planning is starting ")
    print(f"prev_path_conf: {prev_path_conf}")
    prev_path_conf = np.around(prev_path_conf, decimals=8).tolist()

    planner_Ik = PathPlanner(sim, simOMPL, simOMPL.Algorithm.EST, Robot_manager,False,True,True)
    
    for k in range(pathPointCount+1):

        # Solve inverse kinematics for the current pose
        ik_sol = IK_Kinpy(path_to_ur5_urdf_file, current_pose).tolist()

        print(f"planning from :{prev_path_conf} to {ik_sol}")
        if Robot_manager.configuration_validation_callback(ik_sol):
            
            path_dis = planner_Ik.plan_path(prev_path_conf,ik_sol,12,60,Robot_manager,False,6)
            prev_path_conf = ik_sol
            for node in path_dis:
                straightPath.append(node)

        else:
            print("Path is unfeasible at step", k + 1)
            break
        # Update the current pose
        print("current_pose",current_pose)
        current_pose[2] -= step

        

    return straightPath


static_object_names = [
 '/Cuboid_1', '/Cuboid_2', '/Cuboid_3', 
    '/Cylinder_4', '/Sphere_5', '/Sphere_6'
]


def generate_straight_path_custom(object_name, object_name_next,dummy_name,pose, pathPointCount, Robot_manager,prev_path_conf, orient):
    # Ensure pathPointCount is a positive integer
    print(f"pose[2]: {pose[2]}")
    if pathPointCount <= 0:
        raise ValueError("pathPointCount must be a positive integer")
    
    obj_hand = sim.getObject(object_name)
    dum_hand = sim.getObject(dummy_name)
    size = sim.getShapeBB(obj_hand)
    z = 0.41
    if object_name_next is  None:

        dum_hand = sim.getObject(dummy_name)
        obj_pos = sim.getObjectPosition(obj_hand, sim.handle_world)
        dum_pos = sim.getObjectPosition(dum_hand, sim.handle_world)
        if object_name.startswith("/Cuboid_") or object_name.startswith("/Cylinder_"):
            # Calculate the desired z position of the gripper fingers.
            desired_z_position = obj_pos[2] + size[2]/2  - 0.030

            # Calculate the final z position of the gripper TCP.
            rel_dis = abs(desired_z_position - dum_pos[2])
        else:
            desired_z_position = obj_pos[2]
            # Calculate the final z position of the gripper TCP.
            rel_dis = abs(desired_z_position - dum_pos[2])

    else:
        if object_name.startswith("/Cuboid_") or object_name.startswith("/Cylinder_"): 
            rel_dis = pose[2] - (z+size[-1] - 0.030)
        else:
            rel_dis = pose[2] - (z+size[-1]/2)

    print(f"pose[2]: {pose[2]}")


    # Calculate the step size for each point in the path
    step = rel_dis / pathPointCount
    print(f"step: {step}")

    # Initialize the straight path
    straightPath = []
    current_pose = pose.copy()  # Use a copy to avoid modifying the original pose

    # Generate the path
    print("\n IK path planning is starting ")
    print(f"prev_path_conf: {prev_path_conf}")
    prev_path_conf = np.around(prev_path_conf, decimals=8).tolist()
    planner_Ik = PathPlanner(sim, simOMPL, simOMPL.Algorithm.EST, Robot_manager,False,True,True)
    if orient != 'up' :
        for obj_name in static_object_names :
            if obj_name != object_name:
                Robot_manager.manage_object(obj_name,'add_to_robot')    
    
    prev_node = prev_path_conf.copy()
    for k in range(pathPointCount+2):

        # Solve inverse kinematics for the current pose
        ik_sol = IK_Kinpy(path_to_ur5_urdf_file, current_pose).tolist()

        print(f"planning from :{prev_path_conf} to {ik_sol}")
        if Robot_manager.configuration_validation_callback(ik_sol):
            
            path_dis = planner_Ik.plan_path(prev_path_conf,ik_sol,8,20,Robot_manager,False,6)
            prev_path_conf = ik_sol
            if path_dis == []:
                current_pose[2] -= step
                continue
            if k > 0:               
                for node in path_dis[:1]:
                    if node == prev_node:
                        continue
                    prev_node = node.copy() 
                    straightPath.append(node)
              

        else:
            print("Path is unfeasible at step", k + 1)
            break
        # Update the current pose
        print("current_pose",current_pose)
        current_pose[2] -= step

    for obj_name in static_object_names :
        if obj_name != object_name:
            Robot_manager.manage_object(obj_name,'return_to_environment') 
        

    return straightPath

def generate_straight_path_custom_IK(object_name, object_name_next,dummy_name,pose, pathPointCount, Robot_manager,prev_path_conf, orient):
    # Ensure pathPointCount is a positive integer
    print(f"pose[2]: {pose[2]}")
    if pathPointCount <= 0:
        raise ValueError("pathPointCount must be a positive integer")
    
    obj_hand = sim.getObject(object_name)
    dum_hand = sim.getObject(dummy_name)
    size = sim.getShapeBB(obj_hand)
    z = 0.41
    if object_name_next is  None:

        dum_hand = sim.getObject(dummy_name)
        obj_pos = sim.getObjectPosition(obj_hand, sim.handle_world)
        dum_pos = sim.getObjectPosition(dum_hand, sim.handle_world)
        if object_name.startswith("/Cuboid_") or object_name.startswith("/Cylinder_"):
            # Calculate the desired z position of the gripper fingers.
            desired_z_position = obj_pos[2] + size[2]/2  - 0.030

            # Calculate the final z position of the gripper TCP.
            rel_dis = abs(desired_z_position - dum_pos[2])
        else:
            desired_z_position = obj_pos[2]
            # Calculate the final z position of the gripper TCP.
            rel_dis = abs(desired_z_position - dum_pos[2])

    else:
        if object_name.startswith("/Cuboid_") or object_name.startswith("/Cylinder_"): 
            rel_dis = pose[2] - (z+size[-1] - 0.030)
        else:
            rel_dis = pose[2] - (z+size[-1]/2)

    print(f"pose[2]: {pose[2]}")



    # Initialize the straight path
    straightPath = []
    current_pose = pose.copy()  # Use a copy to avoid modifying the original pose

    final_pose = current_pose
    final_pose[2]= current_pose[2] - rel_dis

    # Generate the path
    print("\n IK path planning is starting ")
    print(f"prev_path_conf: {prev_path_conf}")
    prev_path_conf = np.around(prev_path_conf, decimals=8).tolist()
    planner_Ik = PathPlanner(sim, simOMPL, simOMPL.Algorithm.EST, Robot_manager,False,True,True)
    if orient != 'up' :
        for obj_name in static_object_names :
            if obj_name != object_name:
                Robot_manager.manage_object(obj_name,'add_to_robot')    
    
    ik_sol_final = IK_Kinpy(path_to_ur5_urdf_file, final_pose).tolist()
    ik_sol_final[-3:] = prev_path_conf[-3:] 


    straightPath = planner_Ik.plan_path(prev_path_conf,ik_sol_final,12,200,Robot_manager,False,6)
    for obj_name in static_object_names :
        if obj_name != object_name:
            Robot_manager.manage_object(obj_name,'return_to_environment') 
        

    return straightPath[:1]


import json

def convert_element(element):
    if isinstance(element, np.ndarray):
        return element.tolist()
    return element

def save_paths(open_paths, close_paths):

    # Save the open and close paths to JSON files
    with open("open_paths.json", "w") as f:
        json.dump(open_paths, f)
    with open("close_paths.json", "w") as f:
        json.dump(close_paths, f)



planning_time, nb_max_nodes = 12 , 400
initial_config = [0]*6

Robot_manager = RobotManager(sim)

planner = PathPlanner(sim, simOMPL, simOMPL.Algorithm.EST, Robot_manager,False,True,True)


def generate_paths(mapping_dict_forward_new):
    paths = {}
    open_paths = []
    dis_open_paths = []
    close_paths = []
    dis_close_paths = []

    j = 0
    for idx, (init_key, fin_key) in enumerate(mapping_dict_forward_new.items()):
        print(f"init_key: {init_key}")
        config1 = dict_of_all_ik_computed[init_key][0].tolist()
        config2 = dict_of_all_ik_computed[fin_key][0].tolist()
        object_name = dict_Dummies_to_object_name_mapping[dict_of_all_ik_computed[init_key][2]]
        print("object_name:", object_name)
        pose1 = dict_of_all_ik_computed[init_key][3]
        pose2 = dict_of_all_ik_computed[fin_key][3]
        print(f"pose1[2]: {pose1[2]}")
        dummy1 = dict_of_all_ik_computed[init_key][2]
        dummy2 = dict_of_all_ik_computed[fin_key][2]
        picking_orient = dict_of_all_ik_computed[init_key][1]
        placing_orient = dict_of_all_ik_computed[fin_key][1]

        # General path from initial_config to config1
        if idx == 0:
            print("Hi i am the first")
            Robot_manager.manage_object(object_name, None)
            print(f"initial_config:{initial_config}\n config1: {config1}")

            paths[j] = [planner.plan_path(initial_config, config1, planning_time, nb_max_nodes, Robot_manager, False, 6), config1, pose1, 'general_path', 'Open']
            dis_open_paths = paths[j][0] 

        # Down movement for picking
        paths[j + 1] = [generate_straight_path_custom(object_name, None, dummy1, pose1, 2, Robot_manager, paths[j][0][-1], picking_orient), pose1, 'IK_path_forward', 'Open']
        dis_open_paths.extend(paths[j + 1][0])
        open_paths.append(dis_open_paths)
        dis_open_paths = [] 

        # Up movement after picking (reverse path)
        paths[j + 2] = [paths[j + 1][0][::-1], pose1, 'IK_path_reverse', 'Close']
        dis_close_paths = paths[j + 2][0]

        # General path from config1 to config2
        print(f"initial_config:{config1}\n config1: {config2}")
        Robot_manager.manage_object(object_name, 'add_to_robot')
        next_object_name = dict_Dummies_to_object_name_mapping[dict_of_all_ik_computed[fin_key][2]]
        paths[j + 3] = [planner.plan_path(config1, config2, planning_time, nb_max_nodes, Robot_manager, False, 6), config2, pose2, 'general_path', 'Close']
        dis_close_paths .extend(paths[j + 3][0]) 


        # Down movement for placing
        paths[j + 4] = [generate_straight_path_custom(object_name, next_object_name, dummy2, pose2, 2, Robot_manager, paths[j + 3][0][-1], placing_orient), pose2, 'IK_path_forward', 'Close']
        dis_close_paths.extend( paths[j + 4] [0]) # need to be concatinated

        close_paths.append(dis_close_paths)
        dis_close_paths = []
        # Up movement after placing (reverse path)
        paths[j + 5] = [paths[j + 4][0][::-1], pose2, 'IK_path_reverse', 'Open']

        dis_open_paths = paths[j + 5][0]
        # For the last iteration, add path back to initial configuration
        if idx == len(mapping_dict_forward_new) - 1:
            paths[j + 6] = [planner.plan_path(config2, initial_config, planning_time, nb_max_nodes, Robot_manager, False, 6), config2, [0.21989656490672088, -0.11042034384492425, 1.4112301243591316, -0.7071067811864373, -3.7747582837255314e-15, 8.410318035262982e-15, 0.7071067811866578], 'general_path', 'Open']
            dis_open_paths.extend(paths[j + 6][0])

            # Append the final path segment to open_paths
            open_paths.append(dis_open_paths)

            save_paths(open_paths, close_paths)
            break

        # Prepare for the next loop iteration
        Robot_manager.manage_object(object_name, 'return_to_environment')
        next_idx = idx + 1
        next_init_key = list(mapping_dict_forward_new.keys())[next_idx]
        print(f"next_init_key: {next_init_key}")
        config3 = dict_of_all_ik_computed[next_init_key][0].tolist()
        pose3 = dict_of_all_ik_computed[next_init_key][3]
        object_name_nn = dict_Dummies_to_object_name_mapping[dict_of_all_ik_computed[next_init_key][2]]
        paths[j + 7] = [planner.plan_path(config2, config3, planning_time, nb_max_nodes, Robot_manager, False, 6), config3, pose3, 'general_path', 'Open']
        dis_open_paths.extend(paths[j + 7][0])

        j += 7

    return paths


def set_config(simJointHandles, config, dynModel):
    for i in range(len(simJointHandles)):
        if dynModel:
            sim.setJointTargetPosition(simJointHandles[i], config[i])
        else:
            sim.setJointPosition(simJointHandles[i], config[i])

class GripperController:
    def __init__(self, sim):
        self.sim = sim

    def open_gripper(self):
        # Set signal to open the gripper
        self.sim.setInt32Signal('RG2_open', 1)

        # Wait for the gripper to finish opening
        while True:
            if self.sim.getInt32Signal('Gripper_Open_close_done') is not None:
                break

        # Clear the signals
        self.sim.clearInt32Signal('RG2_open')
        self.sim.clearInt32Signal('Gripper_Open_close_done')

    def close_gripper(self):
        # Clear signal to close the gripper (by default it closes)
        self.sim.setInt32Signal('RG2_open', 0)

        # Wait for the gripper to finish closing
        while True:
            if self.sim.getInt32Signal('Gripper_Open_close_done') is not None:
                break

        # Clear the done signal
        self.sim.clearInt32Signal('Gripper_Open_close_done')

gripper = GripperController(sim)




def extract_nodes(path, total_nodes):
    """
    Extracts nodes from a path with denser spacing at the edges and sparser in the middle.
    
    :param path: A list of nodes (e.g., waypoints) in the path.
    :param total_nodes: Total number of nodes to extract.
    :return: A list of extracted nodes.
    """
    if total_nodes >= len(path):
        return path

    # Use a sine function to determine the indices of nodes to extract
    indices = (np.sin(np.linspace(-np.pi / 2, np.pi / 2, total_nodes)) + 1) / 2 * (len(path) - 1)
    indices = np.round(indices).astype(int)

    # Extract nodes based on calculated indices
    extracted_nodes = [path[idx] for idx in indices]

    return extracted_nodes







def load_data(path):
    with open(path, 'r') as f:
        data = json.load(f)
    return data



paths = generate_paths(mapping_dict_forward_new)



                




                
























