# path_planner_module.py

# Import necessary libraries
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np




#sim.startSimulation()

class RobotManager:
    def __init__(self,sim):
        self.sim = sim
        self.simJointHandles = [sim.getObjectHandle(f'/joint{i}') for i in range(1, 7)]
        self.robot_parts = [
            '/link2_visible',
            '/link3_visible',
            '/link4_visible',
            '/UR5/link5_visible',
            '/UR5/link6_visible',
            '/UR5/RG2_leftLink1_visible0',
            '/UR5/RG2_leftLink0_visible0',
            '/UR5/RG2_rightLink1_visible0',
            '/UR5/RG2_rightLink0_visible0',
            '/UR5/RG2/baseVisible0'
        ]
        self.object_names = [
            '/Table1', '/Table2','/Table3', '/Cuboid_1', '/Cuboid_2', '/Cuboid_3', 
            '/Cylinder_4', '/Sphere_5', '/Sphere_6'
        ]
        self._create_collections()

    def _create_collections(self):
        self.roboCollection = self.sim.createCollection(0)
        for part in self.robot_parts:
            #print(part)
            obj_hand = self.sim.getObject(part)
            self.sim.addItemToCollection(self.roboCollection, self.sim.handle_single, obj_hand, 0)

        self.collisionCollection = self.sim.createCollection()
        for object_name in self.object_names:
            obj_hand = self.sim.getObject(object_name)
            self.sim.addItemToCollection(self.collisionCollection, self.sim.handle_single, obj_hand, 0)

    def configuration_validation_callback(self, config):
        tmp = self.get_config()
        self.set_config(config, 0)
        # res = sim.checkCollision(roboCollection, sim.handle_all)
        res = self.sim.checkCollision(self.roboCollection,self.roboCollection,self.roboCollection, self.collisionCollection)
        self.set_config(tmp, 0)
        return res[0] == 0 

    def manage_object(self, object_name, action):     
        if action == 'add_to_robot':
            self.robot_parts.append(object_name)
            self.object_names.remove(object_name)
        if action == 'return_to_environment':
            if object_name in self.robot_parts:
                self.robot_parts.remove(object_name)
            if object_name not in self.object_names:
                self.object_names.append(object_name)
        self._create_collections()  # Update collections

    def get_config(self):
        ret_val = []
        for i in range(len(self.simJointHandles)):
            ret_val.append(self.sim.getJointPosition(self.simJointHandles[i]))
        return ret_val

    def set_config(self, config, dynModel):
        for i in range(len(self.simJointHandles)):
            if dynModel:
                self.sim.setJointTargetPosition(self.simJointHandles[i], config[i])
            else:
                self.sim.setJointPosition(self.simJointHandles[i], config[i])

# Example usage:
# sim = ... # Initialize your simulator object here
# robot_manager = RobotManager(sim)
# robot_manager.manage_object('/Cuboid_1', 'remove_from_environment')
# robot_manager.manage_object('/Cuboid_1', 'add_to_robot')
# robot_manager.manage_object('/Cuboid_1', 'return_to_environment')





class AngleBoundsCalculator:
    def __init__(self):
        pass

    def map_angle_to_0_360_range(self, angle):
        angle_deg = np.degrees(angle) % 360
        return np.radians(angle_deg)

    def map_angle_to_neg_360_0_range(self, angle):
        angle_deg = np.degrees(angle) % 360
        angle_deg -= 360  # Shift to [-360, 0] range
        return np.radians(angle_deg)

    def is_clockwise_close(self, start_angle, goal_angle):
        start_deg = np.degrees(start_angle) % 360
        goal_deg = np.degrees(goal_angle) % 360
        clockwise_distance = (goal_deg - start_deg) % 360
        #print(clockwise_distance)
        counterclockwise_distance = (start_deg - goal_deg) % 360
        #print(counterclockwise_distance)
        return clockwise_distance < counterclockwise_distance 




    def are_angles_in_1st_4th_quadrants(self, start_angle, goal_angle):
        start_deg = np.degrees(start_angle) % 360
        goal_deg = np.degrees(goal_angle) % 360
        # Check if both angles are in the 1st quadrant or both are in the 8th quadrant
        both_in_1st_quadrant = (0 < start_deg < 90) and (0 < goal_deg < 90)
        both_in_4th_quadrant = (270 < start_deg < 360) and (270 < goal_deg < 360)

        are_in_different_quadrants = (start_deg > 270 or start_deg < 90) and (goal_deg > 270 or goal_deg < 90)
        return are_in_different_quadrants and (not both_in_1st_quadrant and not both_in_4th_quadrant)
    

    def find_quadrants(self, angle, quadrants):
        for q in quadrants:
            if q[0] <= angle <= q[1]:
                return q


    def get_constrained_bounds(self, start_angle, goal_angle):
        #print(f"start_angle: {start_angle}, goal_angle: {goal_angle}")
        if start_angle == goal_angle :
            quadrants = [(-360, -315), (-315, -270), (-270, -225), (-225, -180),
                        (-180, -135), (-135, -90), (-90, -45), (-45, 0),
                        (0, 45), (45, 90), (90, 135), (135, 180), (180, 225), 
                        (225, 270), (270, 315), (315, 360)]

        elif self.are_angles_in_1st_4th_quadrants(start_angle, goal_angle):
            print("Ho")
            if start_angle >= 0 and  goal_angle >= 0:
                quadrants = [(0, 45),(45, 90),(315,360)]
            elif start_angle <= 0 and  goal_angle <= 0:
                quadrants = [(-360,-315),(-90,-45),(-45, 0)]
            elif (start_angle <= 0 and  goal_angle >= 0) or (start_angle >= 0 and  goal_angle <= 0):
                quadrants = [(-360,-315),(-90,-45),(-45, 0), (0, 45),(45, 90),(315,360)]
            # ... [Mapping logic for angles in 1st or 4th quadrants]
        elif self.is_clockwise_close(start_angle, goal_angle) and not self.are_angles_in_1st_4th_quadrants(start_angle, goal_angle) :
            print("Hi")
            quadrants = [(0, 45), (45, 90), (90, 135), (135, 180), (180, 225), (225, 270), (270, 315), (315, 360)]
            # ... [Mapping logic for clockwise proximity]
        else:
            #print("HOO")
            quadrants = [(-360, -315), (-315, -270), (-270, -225), (-225, -180),
                         (-180, -135), (-135, -90), (-90, -45), (-45, 0)]
            # ... [Mapping logic for counter-clockwise proximity]

        start_quadrant = self.find_quadrants(np.degrees(start_angle), quadrants)
        print(f"start_quadrant: {start_quadrant}")
        goal_quadrant = self.find_quadrants(np.degrees(goal_angle), quadrants)
        print(f"goal_quadrant: {goal_quadrant}")


        bounds_deg = [min(start_quadrant[0], goal_quadrant[0]), max(start_quadrant[1], goal_quadrant[1])]
        return np.deg2rad(bounds_deg)
    
    def map_config_to_ranges(self, start_config, goal_config):
        mapped_start_config = []
        mapped_goal_config = []
        print(f"start_config_m = {start_config} , goal_config_m = {goal_config}")

        for i, (start_angle, goal_angle) in enumerate(zip(start_config, goal_config)):

            if self.are_angles_in_1st_4th_quadrants(start_angle, goal_angle):
                print(f"i:{i} A")
                if np.deg2rad(270) < start_angle < np.deg2rad(360):
                    st_angl = self.map_angle_to_neg_360_0_range(start_angle)
                else:
                    st_angl = self.map_angle_to_0_360_range(start_angle)
                mapped_start_config.append(st_angl)

                if np.deg2rad(270) < goal_angle < np.deg2rad(360):
                    en_angl = self.map_angle_to_neg_360_0_range(goal_angle)
                else:
                    en_angl = self.map_angle_to_0_360_range(goal_angle)
                mapped_goal_config.append(en_angl)

            elif self.is_clockwise_close(start_angle, goal_angle) and not self.are_angles_in_1st_4th_quadrants(start_angle, goal_angle):
                #print(f"i:{i} B")
                mapped_start_config.append(self.map_angle_to_0_360_range(start_angle))
                mapped_goal_config.append(self.map_angle_to_0_360_range(goal_angle))
            else:
                #print(f"i:{i} C")
                mapped_start_config.append(self.map_angle_to_neg_360_0_range(start_angle))
                mapped_goal_config.append(self.map_angle_to_neg_360_0_range(goal_angle))

        print(f"mapped_start_config = {mapped_start_config} , mapped_goal_config = {mapped_goal_config}")

        return mapped_start_config, mapped_goal_config


class PathPlanner:
    def __init__(self, sim, simOMPL, algorithm, Robot_manager, debug=False, dynamic_constraints = True, itermediat_configs = False):
        # Initialize the planner with necessary components and settings
        self.Robot_manager = Robot_manager
        self.simOMPL = simOMPL
        self.simJointHandles = [sim.getObjectHandle(f'/joint{i}') for i in range(1, 7)]
        self.algorithm = algorithm
        self.current_config = [0] * len(self.simJointHandles)
        self.debug = debug
        self.active_joints = [0] * len(self.simJointHandles)
        self.non_active_joint_config = [0] * len(self.simJointHandles)
        self.dynamic_constraints = dynamic_constraints
        self.calculator = AngleBoundsCalculator()
        self.itermediat_configs = itermediat_configs


    def log(self, message):
        # Utility method for debugging messages
        if self.debug:
            print(message)

        # Auxiliary functions

    def get_joint_bounds_mapping(self, start_angle, goal_angle):
        """Determine which bounds case to use based on start and goal angles."""
        default_non_cons_bonds = np.deg2rad([-360, 360])
        if  self.dynamic_constraints:
            return self.calculator.get_constrained_bounds(start_angle, goal_angle)
        else :
            return default_non_cons_bonds
    
    def generate_intermediate_configs_pairs_general(self, start_config, final_config, dof_per_step):
        num_joints = len(start_config)
        num_steps = num_joints // dof_per_step

        intermediate_configs_init = []
        intermediate_configs_fin = []

        for step in range(num_steps):
            # Generate intermediate final config
            config_fin = final_config[:dof_per_step * (step+1)] + start_config[dof_per_step * (step+1):]

            # Intermediate initial config for the first step is the start_config itself
            if step == 0:
                config_init = start_config
            else:
                # For subsequent steps, the initial config is the final config of the previous step
                config_init = intermediate_configs_fin[-1]
            #config_init, config_fin = self.calculator.map_config_to_ranges(config_init, config_fin)
            intermediate_configs_init.append(config_init)
            intermediate_configs_fin.append(config_fin)

        return intermediate_configs_init, intermediate_configs_fin

    def generate_additional_intermediates(self, start_angle, end_angle):

        distance = abs(start_angle - end_angle)
        self.log(f"distance: {distance}")

        step_count = int(distance / 0.79)
        self.log(f"step_count: {step_count}")

        if step_count > 0:
            step_size = distance / (step_count+1)
            self.log(f"step_size: {step_size}")
            return  [(start_angle + i * step_size * np.sign(end_angle - start_angle)) for i in range(1, step_count+1)]
        return []
    
    def generate_intermediate_configs_pairs(self, start_config, final_config, dof_per_step, gen_add_int_configs = True):
        #print("HI")
        start_config, final_config = self.calculator.map_config_to_ranges(start_config, final_config)
        self.log(f"start_config: {start_config}, final_config:{final_config}")

        if self.itermediat_configs:

            intermediate_configs_init , intermediate_configs_fin = self.generate_intermediate_configs_pairs_general(start_config, final_config, dof_per_step)

            self.log(f"intermediate_configs_init: {intermediate_configs_init}")
            self.log(f"intermediate_configs_fin: {intermediate_configs_fin}")

            intermediate_configs_init_copy = []
            intermediate_configs_fin_copy = []

            for i, (config_initial, config_final) in enumerate(zip(intermediate_configs_init, intermediate_configs_fin)):
                new_config = config_final.copy()
                need_insertion = False

                for j, (start_angle, final_angle) in enumerate(zip(config_initial, config_final)):
                    if self.calculator.are_angles_in_1st_4th_quadrants(start_angle, final_angle):
                        new_config[j] = 0
                        need_insertion = True

                if need_insertion:
                    #config_initial, new_config = self.calculator.map_config_to_ranges(config_initial, new_config)
                    intermediate_configs_init_copy.append(config_initial)
                    intermediate_configs_fin_copy.append(new_config)
                    intermediate_configs_init_copy.append(new_config)
                    intermediate_configs_fin_copy.append(config_final)
                else:
                    intermediate_configs_init_copy.append(config_initial)
                    intermediate_configs_fin_copy.append(config_final)
            self.log(f"\n intermediate_configs_init_copy: {intermediate_configs_init_copy}")
            self.log(f"\n intermediate_configs_fin_copy: {intermediate_configs_fin_copy}")

            original_pairs_bounds = []
            for conf_init, conf_end in zip(intermediate_configs_init_copy, intermediate_configs_fin_copy):
                bounds = []
                for start, end in zip(conf_init, conf_end):
                    bounds.append(self.get_joint_bounds_mapping(start, end))
                original_pairs_bounds.append(bounds)

            if gen_add_int_configs == True:
                # Generate additional intermediate configurations
                extended_intermediate_configs_init = []
                extended_intermediate_configs_fin = []
                bounds_added = []

                for l ,(config_initial, config_final) in enumerate(zip(intermediate_configs_init_copy, intermediate_configs_fin_copy)):

                
    
                    for j, (start_angle, final_angle) in enumerate(zip(config_initial, config_final)):
                        if abs(start_angle - final_angle) > 0.79:
                            #print ("Hi")
                            additional_intermediates = self.generate_additional_intermediates(start_angle, final_angle)

                            self.log(f"additional_intermediates: {additional_intermediates}")

                            # Add the intermediate configurations
                            for intermediate in additional_intermediates:
                                new_config = config_initial.copy()
                                new_config[j] = intermediate
                                extended_intermediate_configs_init.append(config_initial)
                                self.log(f"\nconfig_initial_inter : {config_initial}")
                                extended_intermediate_configs_fin.append(new_config)
                                self.log(f"\nnew_config_inter : {new_config}")
                                config_initial = new_config  # Update initial configuration for next step
                                bounds_added.append(original_pairs_bounds[l])

                    # Append the final configuration of this segment
                    extended_intermediate_configs_init.append(config_initial)
                    extended_intermediate_configs_fin.append(config_final)
                    bounds_added.append(original_pairs_bounds[l])

                return extended_intermediate_configs_init, extended_intermediate_configs_fin, bounds_added
            
            else :
                return intermediate_configs_init_copy, intermediate_configs_fin_copy, original_pairs_bounds
        else:
            bounds = []
            for start, end in zip(start_config, final_config):
                bounds.append(self.get_joint_bounds_mapping(start, end))

            return [start_config], [final_config], [bounds]

    def concatenate_paths(self,paths):
        complete_path = []
        for i, path in enumerate(paths):
            if i > 0:
                path = path[1:]
            complete_path += path
        return complete_path

    def make_valid_callback(self, non_active_joint_config, active_joints):
        # Create a custom validation callback for collision checking
        def valid_callback(state):
            full_state = list(non_active_joint_config)
            active_index = 0
            for i, active in enumerate(active_joints):
                if active:
                    full_state[i] = state[active_index]
                    active_index += 1

            #self.log("Full state for validation: " + str(full_state))
            return self.Robot_manager.configuration_validation_callback(full_state)
        return valid_callback
    

        


    def setup_state_space(self, task, active_joints,start_config, goal_config, bounds_info):
        # Set up the state space for the current segment based on active joints
        self.log("Setting up state space with active joints for current segment...")
        self.log(f"start_config:{start_config} \n goal_config: {goal_config}, \n active_joints: {active_joints}")
        state_spaces = []
        for i, joint_handle in enumerate(self.simJointHandles):
            if active_joints[i]:
                bounds = bounds_info[i]
                print(f"bounds for joint{i}: :{bounds}")
                name = f'joint_{i + 1}_space'
                state_space = self.simOMPL.createStateSpace(name, self.simOMPL.StateSpaceType.joint_position, joint_handle, [bounds[0]], [bounds[1]], 1)
                state_spaces.append(state_space)
        self.simOMPL.setStateSpace(task, state_spaces)
        self.simOMPL.setAlgorithm(task, self.algorithm)


    def compute_path(self,task_name, start_config, goal_config, active_joints, bounds_info,  valid_callback, planning_time ,nb_max_nodes,Robot_manager):
        self.log(f"start_config: {start_config}, goal_config: {goal_config}")
        # Compute a path for the given segment and return the full path including non-active joints
        task = self.simOMPL.createTask(task_name)
        #start_config, goal_config = self.calculator.map_config_to_ranges(start_config, goal_config)
        self.log(f"start_config: {start_config}, goal_config: {goal_config}")
        self.setup_state_space(task, active_joints,start_config, goal_config, bounds_info)
        filtered_start_config = [cfg for cfg, active in zip(start_config, active_joints) if active]  
        filtered_goal_config = [cfg for cfg, active in zip(goal_config, active_joints) if active]
        self.log(f"filtered_start_config: {filtered_start_config}, filtered_goal_config: {filtered_goal_config}")
        self.simOMPL.setStartState(task, filtered_start_config)
        self.simOMPL.setGoalState(task, filtered_goal_config)
        self.simOMPL.setStateValidationCallback(task, valid_callback)
        self.simOMPL.setCollisionPairs(task, [Robot_manager.roboCollection, Robot_manager.roboCollection, Robot_manager.roboCollection, Robot_manager.collisionCollection])
        self.simOMPL.setStateValidityCheckingResolution(task, 0.005)
        self.simOMPL.setup(task)
         
        success_2,_ = self.simOMPL.compute(task, planning_time, -1, nb_max_nodes)

        success_1 = self.simOMPL.hasExactSolution(task)
        success = False
        if success_1:
            active_joint_path =self.simOMPL.getPath(task)
            print("\nExact path has been found")
            success = success_1
        elif  success_2 :            
            print("\nrunning the algorithm again ")
            success_2,active_joint_path = self.simOMPL.compute(task, planning_time, -1, nb_max_nodes)
            print("success_2:",success_2)
            print("\nFaited to not be solved exactely  ")

            self.log("\n Approximate path has been found")
            success = success_2            
        path = []

        if success:

            #self.log(f"active_joint_path:{active_joint_path}")

            # Number of active joints in each state
            num_active_joints = sum(active_joints)

            # Reshape flat list into a list of states
            for i in range(0, len(active_joint_path), num_active_joints):
                active_state = active_joint_path[i:i + num_active_joints]
                full_state = list(self.non_active_joint_config)
                active_index = 0
                for j, active in enumerate(active_joints):
                    if active:
                        full_state[j] = active_state[active_index]
                        active_index += 1
                path.append(full_state)
        else:
            print("\nPath has not been found")
        self.simOMPL.destroyTask(task)
        self.log(f"\nTask {task_name} completed with success: {success}")
        return success, path


    def plan_path(self, start_config, final_config, planning_time , nb_max_nodes ,Robot_manager ,gen_add_intr_config_pairs =False, dof_per_step =6):
        # Plan the path based on the final configuration and degrees of freedom per step
        self.log("Starting path planning...")
        paths = []
        intermediate_init_configs, intermediate_fin_configs, bounds_info = self.generate_intermediate_configs_pairs(start_config, final_config, dof_per_step, gen_add_intr_config_pairs )
        self.log(f"intermediate_fin_configs: {intermediate_fin_configs}")
        self.log(f"intermediate_init_configs: {intermediate_init_configs}")
        planning_time = planning_time/len(intermediate_init_configs)
        
        # Iterate over pairs of configurations
        for idx, (init_config, next_config) in enumerate(zip(intermediate_init_configs, intermediate_fin_configs)):
            #print("Hi")
            self.log(f"next_config: {next_config}")
            self.log(f"init_config: {init_config}")  

            task_name = f'task_{idx}'
            self.log(f"Processing segment {idx+1}/{len(intermediate_fin_configs)}...")

            # Determine which joints are active (1 for active, 0 for inactive)
            self.active_joints = [int(init_val != next_val) for init_val, next_val in zip(init_config, next_config)]
            self.log(f"active_joints: {self.active_joints}")

            # For non-active joints, keep their original values; for active joints, use None
            self.non_active_joint_config = [init_val if not active else None for init_val, active in zip(init_config, self.active_joints)]
            self.log(f"non_active_joint_config: {self.non_active_joint_config}")
         
            custom_valid_callback = self.make_valid_callback(self.non_active_joint_config, self.active_joints)
            if  np.allclose(init_config ,next_config,atol=1e-18):
                print("Hi")
                print(init_config)
                print(next_config)
                continue
            success, path_segment = self.compute_path(task_name, init_config, next_config, self.active_joints, bounds_info[idx],  custom_valid_callback, planning_time, nb_max_nodes, Robot_manager)
            
            if success:
                self.current_config = next_config
                paths.append(path_segment)

            else:
                print("Failed to compute path to next intermediate configuration")
                break
        
        complete_path = self.concatenate_paths(paths)
        self.log("Path planning completed.")
        return complete_path

'''# Example Usage of the planner

print("Initializing Remote API Client...")
client = RemoteAPIClient()
sim = client.require('sim')
simOMPL = client.require('simOMPL')

print("Starting simulation...")
#sim.startSimulation()
#simBase = sim.getObject('/UR5_not_dyn_for_collision')

init_config = [0.15083478152557336, 6.188853900596852, 1.6199028201794858, 0.045225389885161775, 4.712389370706476, 4.863224593218436]

final_config = [6.2207542108295915, 5.859173659829112, 5.273132891710764, 6.146453028978356, 1.5707959899173278, 
1.5083652961946663]
dof_per_step = 6
planning_time = 8
nb_max_nodes = 300



Robot_manager = RobotManager(sim)
planner = PathPlanner(sim, simOMPL, simOMPL.Algorithm.PRM , Robot_manager,True,True ,True)
complete_path = planner.plan_path(init_config,final_config, planning_time, nb_max_nodes,Robot_manager, gen_add_intr_config_pairs=False)
print("Complete Path:", complete_path)
'''
'''

def set_config(simJointHandles, config, dynModel):
    for i in range(len(simJointHandles)):
        if dynModel:
            self.sim.setJointTargetPosition(simJointHandles[i], config[i])
        else:
            self.sim.setJointPosition(simJointHandles[i], config[i])

sw = sim.setStepping(True)
for conf in complete_path:
    set_config(simJointHandles, conf, 1)
    sim.step()
sim.setStepping(sw)'''



'''# Generating intermediate configurations pairs
intermediate_init_configs, intermediate_fin_configs = planner.generate_intermediate_configs_pairs(init_config, final_config, dof_per_step)

# Printing the generated configurations for verification
print("Intermediate Initial Configurations:")
for config in intermediate_init_configs:
    print(config)

print("\nIntermediate Final Configurations:")
for config in intermediate_fin_configs:
    print(config)'''
