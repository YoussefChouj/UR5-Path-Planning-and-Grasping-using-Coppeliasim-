
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from  inverse_kinematics_using_kinpy_working import compute_ik_configs_of_multip_pos_orientations, IK_Kinpy
from path_planner_module_validation_default_scene_collision_detection import PathPlanner, RobotManager , AngleBoundsCalculator
import json

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
    # Initializer for the CustomMapping class
    def __init__(self, default_forward, values, mapping_from_dummy_names_to_object_names):
        # Store the default forward mapping
        self.default_forward = default_forward
        # Store the default values for positions and configurations
        self.values = values
        # Placeholder for modified values after setup
        self.modified_values = None
        # Placeholder for automatic default condition status
        self.aut_def_cond = None
        # Dictionary mapping dummy names to object names
        self.dict_Dummies_to_object_name_mapping = {}
        # Flags to track if default or custom mapping is chosen
        self.default_mapping_chosen = False
        self.custom_mapping_chosen = False
        # Store the mapping from dummy names to object names
        self.mapping_from_dummy_names_to_object_names = mapping_from_dummy_names_to_object_names



    # Main setup method to configure the mapping
    def setup(self):
        # Ask user to choose between default or custom mapping
        use_default = input("Do you want to use default mapping? (yes/no): ")
        if use_default.lower() == 'yes':
            self.setup_default()         
        else:
            self.setup_custom()
        # Setup orientation based on user input
        self.setup_orientation()
        # Update modified values after setup
        self.modified_values = self.values

    # Method to setup default mapping
    def setup_default(self):
        # Initialize forward and reverse mapping dictionaries
        self.mapping_dict_forward = {}
        self.mapping_dict_reverse = {}
        # Create forward and reverse mappings based on default_forward
        for k, v in self.default_forward.items():
            init_key = next(key for key, val in self.values.items() if val[2] == k)
            fin_key = next(key for key, val in self.values.items() if val[2] == v)
            self.mapping_dict_forward[init_key] = fin_key
            self.mapping_dict_reverse[fin_key] = init_key
        # Mark that default mapping is chosen
        self.default_mapping_chosen = True


    # Method to setup custom mapping defined by the user
    def setup_custom(self):
        # Initialize forward mapping dictionary
        self.mapping_dict_forward = {}
        # Create lists of available keys and values for mapping
        available_keys = list(self.values.keys())
        available_values = list(self.values.keys())

        # Iterate until all keys and values are mapped or 'done' is input
        while available_keys and available_values:
            # Display available keys and get user input
            print("Available keys: ", available_keys)
            key = input("Select a key (or type 'done' to finish): ")
            if key.lower() == 'done':
                break
            if key not in available_keys:
                print("Invalid key or key already used. Please select a valid key.")
                continue

            # Display available values and get user input
            print("Available values: ", available_values)
            value = input("Select a value for the chosen key: ")
            if value not in available_values:
                print("Invalid value or value already used. Please select a valid value.")
                continue

            # Add the key-value pair to the mapping and update available lists
            self.mapping_dict_forward[key] = value
            available_keys.remove(key)
            available_values.remove(value)

        # Display final forward mapping
        print(f"mapping_dict_forward: {self.mapping_dict_forward}")
        # Generate and store the reverse mapping automatically
        self.mapping_dict_reverse = {v: k for k, v in self.mapping_dict_forward.items()}
        # Mark that custom mapping is chosen
        self.custom_mapping_chosen = True


    # Method to setup orientation based on user input
    def setup_orientation(self):
        # Ask user if they want to change the default picking orientations
        change_orientation = input("Do you want to change the default picking orientations or use automatic feasible ones (it is recommended to use automated found ones)? (yes/no): ")
        if change_orientation.lower() == 'yes':

            # Allow user to set new orientation for each key in the chosen mapping
            for (k,v) in self.mapping_dict_forward.items():
                new_orientation_init = input(f"New orientation for {k} please choose from ('up','up_x_60','up_x_-60','up_y_60','up_y_-60', 'side_North','side_South', 'side_West', 'side_East'): ")
                new_orientation_fin = input(f"New orientation for {k} please choose from ('up','up_x_60','up_x_-60','up_y_60','up_y_-60', 'side_North','side_South', 'side_West', 'side_East'): ")
                
                self.values[k][1] = new_orientation_init
                self.values[v][1] = new_orientation_fin

            self.aut_def_cond = False
        else:
            # Use automated orientations
            self.aut_def_cond = True


    # Method to setup conditional values based on mapping
    def setup_cond_values(self):
        for k in self.mapping_dict_forward.keys():
            if self.dict_Dummies_to_object_name_mapping[self.values[k][2]] == None:
                self.values[k][-1] = False
            else:
                self.values[k][-1] = True 

    # Method to update object name mapping based on chosen mapping
    def update_object_name_mapping(self,init_keyy):
        obj_name = None
        vals = list(self.mapping_dict_forward.values())  # Create a copy of vals
        init_key = init_keyy
        while True:  # Loop until object name is found
            print(f"Debug: Checking init_key={init_key}, vals={vals}")
            if init_key in vals:
                # Get the key corresponding to value init_key
                key_dumm = self.values[init_key][2]
                obj_name = self.mapping_from_dummy_names_to_object_names[key_dumm]
                if obj_name is None:
                    print(f"Debug: Object name is None. Updating init_key={init_key}")
                    vals.remove(init_key)
                    init_key = self.mapping_dict_reverse[init_key]
                    continue
                else:
                    print(f"Debug: Object name found: {obj_name}")
                    self.mapping_from_dummy_names_to_object_names[self.values[init_keyy][2]] = obj_name
                    self.values[init_keyy][-1] = True
                    break
            else:
                key_dumm = self.values[init_key][2]
                obj_name = self.mapping_from_dummy_names_to_object_names[key_dumm]
                print(f"Debug: init_key={init_key} not in vals. Namme found.")
                break

        return obj_name






# Code to initialize and setup the CustomMapping instance
# Initialize config arrays for each position
configs = [np.zeros(6) for _ in range(12)]

# Define the default forward mapping
mapping_dict_forward = {
    '/DummyTb2Pos1': '/DummyTb1Pos1',
    '/DummyTb2Pos2': '/DummyTb1Pos2',
    '/DummyTb2Pos3': '/DummyTb1Pos3',
    '/DummyTb2Pos4': '/DummyTb1Pos4',
    '/DummyTb2Pos5': '/DummyTb1Pos5',
    '/DummyTb2Pos6': '/DummyTb1Pos6',
}

# Define the dictionary of all positions and corresponding robot configurations
dict_of_all_pos_and_corr_robotconfigs = {
    f'init{i+1}': [configs[i], "up", f'/DummyTb2Pos{i+1}',  None, False] for i in range(6)
} | {
    f'fin{i+1}': [configs[i+6], "up", f'/DummyTb1Pos{i+1}', None, False] for i in range(6)
}

# Define object names aliases
object_names_alliases = [
    '/Cuboid_1', '/Cuboid_2', '/Cuboid_3', 
    '/Cylinder_4', '/Sphere_5', '/Sphere_6'
]

# Define mapping from dummy names to object names
mapping_from_dummy_names_to_object_names = {
    '/DummyTb2Pos1': '/Cuboid_1',
    '/DummyTb2Pos2': '/Cuboid_2',
    '/DummyTb2Pos3': '/Cuboid_3',
    '/DummyTb2Pos4': '/Cylinder_4',
    '/DummyTb2Pos5': '/Sphere_5',
    '/DummyTb2Pos6': '/Sphere_6',
} | {
    f'/DummyTb1Pos{i+1}': None for i in range(6)
}

# Create an instance of CustomMapping and set it up
cm = CustomMapping(mapping_dict_forward, dict_of_all_pos_and_corr_robotconfigs, mapping_from_dummy_names_to_object_names)
cm.setup()

# Extract updated mappings and values after setup
mapping_dict_forward_new = cm.mapping_dict_forward 
dict_of_all_pos_and_corr_robotconfigs_new = cm.modified_values 
aut_def_condition = cm.aut_def_cond
dict_Dummies_to_object_name_mapping = cm.dict_Dummies_to_object_name_mapping


print ("dict_of_all_pos_and_corr_robotconfigs_new", dict_of_all_pos_and_corr_robotconfigs_new)

print ("mapping_dict_forward_new:",mapping_dict_forward_new)

print ("dict_Dummies_to_object_name_mapping:",dict_Dummies_to_object_name_mapping)


all_orientation_keys = ["up","up_x_60","up_x_-60","up_y_60","up_y_-60", "side_North", "side_South", "side_West", "side_East"]

#---------------------------

path_to_ur5_urdf_file = "ur5_robot_without_limits.urdf.xml"


dict_of_all_ik_computed = compute_ik_configs_of_multip_pos_orientations(path_to_ur5_urdf_file,\
                            mapping_dict_forward_new, \
                               dict_of_all_pos_and_corr_robotconfigs_new, aut_def_condition,
                               all_orientation_keys)

print ("dict_of_all_ik_computed:", dict_of_all_ik_computed)




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


def generate_straight_path_custom(object_name, oper , dummy_name, pose, pathPointCount, Robot_manager, prev_path_conf, orient):
    print("Ikkkk")
    # Ensure pathPointCount is a positive integer
    print(f"pose[2]: {pose[2]}")
    if pathPointCount <= 0:
        raise ValueError("pathPointCount must be a positive integer")
    
    obj_hand = sim.getObject(object_name)
    dum_hand = sim.getObject(dummy_name)
    size = sim.getShapeBB(obj_hand)
    z = 0.41
    if oper == 'Place':

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

    elif oper == 'Pick':
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
    print(f"mapping_dict_forward_new.items(): {mapping_dict_forward_new.items()}")
    j = 0
    for idx, (init_key, fin_key) in enumerate(mapping_dict_forward_new.items()):
        print(f"init_key: {init_key}")
        config1 = dict_of_all_ik_computed[init_key][0].tolist()
        config2 = dict_of_all_ik_computed[fin_key][0].tolist()
        object_name = cm.update_object_name_mapping(init_key)
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

        paths[j + 1] = [generate_straight_path_custom(object_name, 'Pick', dummy1, pose1, 2, Robot_manager, config1, picking_orient), config1, pose1, 'IK_path_forward', 'Open']
        dis_open_paths.extend(paths[j + 1][0])
        open_paths.append(dis_open_paths)
        dis_open_paths = [] 

        # Up movement after picking (reverse path)
        paths[j + 2] = [paths[j + 1][0][::-1], pose1, 'IK_path_reverse', 'Close']
        dis_close_paths = paths[j + 2][0]

        # General path from config1 to config2
        print(f"initial_config:{config1}\n config1: {config2}")
        Robot_manager.manage_object(object_name, 'add_to_robot')
        paths[j + 3] = [planner.plan_path(config1, config2, planning_time, nb_max_nodes, Robot_manager, False, 6), config2, pose2, 'general_path', 'Close']
        dis_close_paths .extend(paths[j + 3][0]) 


        # Down movement for placing
        paths[j + 4] = [generate_straight_path_custom(object_name, 'Place', dummy2, pose2, 2, Robot_manager, config2, placing_orient), config2, pose2, 'IK_path_forward', 'Close']
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
        if next_init_key == fin_key:
            continue
        else:
            print(f"next_init_key: {next_init_key}")
            config3 = dict_of_all_ik_computed[next_init_key][0].tolist()
            pose3 = dict_of_all_ik_computed[next_init_key][3]
            paths[j + 7] = [planner.plan_path(config2, config3, planning_time, nb_max_nodes, Robot_manager, False, 6), config3, pose3, 'general_path', 'Open']
            dis_open_paths.extend(paths[j + 7][0])

            j += 7

    return paths


paths = generate_paths(mapping_dict_forward_new)

sim.startSimulation()