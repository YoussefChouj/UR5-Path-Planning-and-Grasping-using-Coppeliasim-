# Automatic Picking and Placing of objects from one table to another in Coppeliasim Using a UR5 robot and a RG2 Gripper:

## Fully automatic and Customizable picking and placing of objects from one table to another using kinpy for inverse kinematics and Coppeliasim  OMPL plugin for motion planning accessed throught the zmq remote Api access for pregrasp planning  and on Coppeliasi scene side Used the  online trajectory generation and exection it  using lua scripting 
     
This project is an implementation of pregrasp planning where the collision free paths are computed beforehand and stored in two files the open and close paths differentiated by the state of the gripper if its open or close  to be executed automatically later on when we start the simulation in the scene.  The provided code does the following :
Now lets Outlined how the Pre-Grasp Planning has been implemented :

First we will need the the start and goal configurations of the robot along with other accompanying information, like the desired pose the object name , the name of of the dummy where the inverse kinematics need to happen ,the desired pose of the griper TCP and other optional data like the picking or placing orientations for example by default if the path planning code is run we get the following dictionary :
    
dict_of_all_ik_computed: {'init1': [array([5.6712645 , 0.04032066, 1.48919789, 0.04127727, 4.71238892,
       4.10046861]), 'up', '/DummyTb2Pos1', [0.6999998807907105, -0.649999713897705, 0.6850000381465475, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], True], 'init2': [array([5.79058293, 0.48350375, 0.92195442, 0.16533849, 4.71238894,
       4.21978697]), 'up', '/DummyTb2Pos2', [0.8749997854232787, -0.6749997615814208, 0.6850000083446505, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], True], 'init3': [array([6.22603416, 0.25933827, 
1.23476852, 0.07668923, 4.71238919,
       4.65523809]), 'up', '/DummyTb2Pos3', [0.8249998688697817, -0.37499990463256827, 0.6850000083446505, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], True], 'init4': [array([6.05543282, 0.52353896, 0.860663  , 0.18659437, 4.71238904,
       4.48463605]), 'up', '/DummyTb2Pos4', [0.9243300907366094, -0.5000430314487929, 0.6850000083446505, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], True], 'init5': [array([0.08955805, 6.22447459, 
1.58724196, 0.04226591, 4.71238935,
       4.80194676]), 'up', '/DummyTb2Pos5', [0.674999749660492, -0.2999998837709427, 0.6850000083446505, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], True], 'init6': [array([0.14830145, 0.32779192, 1.18058077, 0.06242357, 4.71238897,
       4.86069053]), 'up', '/DummyTb2Pos6', [0.8250000476837047, -0.2499998837707999, 0.6710000097751619, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], True], 'fin1': [array([0.61820395, 5.93957171, 5.15988881, 6.17929913, 1.57079585,
       2.18899967]), 'up', '/DummyTb1Pos1', [-0.35000084638595585, -0.725002098083496, 0.6849907696247103, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], False], 'fin2': [array([0.58547245, 5.71265062, 5.49651393, 6.06959556, 1.5707964 ,
       2.15626866]), 'up', '/DummyTb1Pos2', [-0.42500095367431623, -0.750002098083496, 0.6849907696247103, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], False], 'fin3': [array([0.47681352, 5.88911291, 5.23018491, 6.15946163, 1.57079573,
       2.04760935]), 'up', '/DummyTb1Pos3', [-0.40000084042549144, -0.6500022649765016, 0.6849907696247103, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], False], 'fin4': [array([0.31939373, 5.88522065, 5.23571647, 6.15782249, 1.5707964 ,
       1.89018983]), 'up', '/DummyTb1Pos4', [-0.42500081062316875, -0.5500022411346436, 0.6849907696247103, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], False], 'fin5': [array([6.25570562, 6.06456012, 4.99721506, 6.21698437, 1.57079637,
       1.54331642]), 'up', '/DummyTb1Pos5', [-0.3500008404254914, -0.32500403225421914, 0.6849907696247103, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], False], 'fin6': [array([0.08803472, 5.83670952, 5.2699744 , 6.17207519, 1.57079642,
       1.6588312 ]), 'up', '/DummyTb1Pos6', [-0.4500008344650267, -0.4000040322542192, 0.6709907710552218, -0.7071068345716235, 0.0, 0.0, 0.7071067278014678], False]} 
Where for each key stored is a list where the 1st columns is where the computed ik solution is , the 2nd column is where the picking orientation is set, the 3rd column is where the name of the dummy in the coppeliasim scene, the 4th column is where the desired Gripper TCP is stored and the 5th column is where a Boolean storing the state whether the object exist bellow the dummy or not .

Then as I coded the path planning start following a default sequence if the user picks it or in a custom faction where he is asked to choose between the two :
‘’Do you want to use default mapping? (yes/no): yes’’
if yes then the default picking and placing positions are assigned
If no then the user is asked to input from to where to pick and place an object 
By imputing for each init[idx] key the corresponding init[idx] or fin[idx] key corresponding to distinct positions in the scene on the tables (init keys correspond to table 1 and fin keys correspond to table 2 in the scene) for all the object in the scene(6 in total)

Then next the user is asked :

’’Do you want to change the default picking orientations or use automatic feasible ones it is recomended to use automated 
found ones  ? (yes/no): no’’

I recommend no answer for default upward picking orientation 
But if the answer is yes then user is asked to fill one by one the following one by one :
New orientation for init1 please choose from ('up','up_x_60','up_x_-60','up_y_60','up_y_-60', 'side_North','side_South', 'side_West', 'side_East'): up
New orientation for init2 please choose from ('up','up_x_60','up_x_-60','up_y_60','up_y_-60', 'side_North','side_South', 'side_West', 'side_East'):
.
.
.
New orientation for fin6 please choose from ('up','up_x_60','up_x_-60','up_y_60','up_y_-60', 'side_North','side_South', 'side_West', 'side_East'):

Directly after come the sequenced path planning that loop on to  the key to value mappings in the following sequence


![image](https://github.com/YoussefChouj/UR5-Path-Planning-and-Grasping-using-Coppeliasim-/assets/153049901/f1a1a5d8-68aa-47be-a334-ebb512c62a07)





To account for grasping i had to do that in the path planning code that was done by classifying the open and close and paths in and save them separately so that when the json file is loaded in the coppeliasim scene I can  loop through both the corresponding open and close path in each iteration one by one the first path execution is the open path where the gripper is open and after its execution using coppeliasim ruckig online trajectory generation a signal is sent to the gripper and its closed and the closed path is executed and so on using the same process for each object to be moved


  
To replicate the results of this project, you need to set up a virtual environment and install the necessary dependencies.

Prerequisites
Anaconda 
CoppeliaSim 
Creating and Activating a Virtual Environment
Clone the Repository:

Clone this repository to your local machine using 
     git clone https://github.com/YoussefChouj/UR5-Path-Planning-and-Grasping-using-Coppeliasim-.git.

Create a Virtual Environment:

Open Anaconda Prompt or your terminal.
Navigate to the cloned repository's directory.
Create a new virtual environment by running:

    conda create --name youssef_env python=3.10.10

Activate the Virtual Environment:

Activate the created environment by running:

    conda activate youssef_env

Installing Dependencies
With the virtual environment activated, install the required packages using the requirements.txt file:

    pip install -r requirements.txt

Running the Code
After setting up the environment and installing dependencies, navigate to the script's directory and before running the code make sure 
the Copeeliasim scene is opened then run :

    python Path_planning_for_a_UR5_using_RG2_gripper_kinpy_OMPL_2.py

After the paths have been saved to the json files "open_paths.json" and "close_paths.json" in your home directory copy their path(make sure \\ instead of \ seperation is there in the path) and past it into the script attached to the UR5 robot in the coppeliasim scene and specifically down in the script in  :
function sysCall_thread()
.
.
    -- Put the paths here 
    local openPaths = loadJsonFile('C:\\Users\\youssef\\UR5_robot_machine_learning\\open_paths.json')
    local closePaths = loadJsonFile('C:\\Users\\youssef\\UR5_robot_machine_learning\\close_paths.json')
.
.
    -- specify the path where the joints data will be recorded 
    writeToFile('C:\\Users\\youssef\\UR5_robot_machine_learning\\UR5_RG2\\openPathData.json', openDataJson)
    writeToFile('C:\\Users\\youssef\\UR5_robot_machine_learning\\UR5_RG2\\closePathData.json', closeDataJson)

    print("Data saved to openPathData.json and closePathData.json")
    sim.stopSimulation()
end
after specifying the paths where the joints data have been saved and to plot them copy the same paths to  the Plot_Joints_Data.py file instructions to plot specific datas  

# Load data
open_path_file = 'C:\\Users\\youssef\\UR5_robot_machine_learning\\UR5_RG2\\openPathData.json'
close_path_file = 'C:\\Users\\youssef\\UR5_robot_machine_learning\\UR5_RG2\\closePathData.json'

open_data = load_data(open_path_file)
close_data = load_data(close_path_file)

#Watch video at :
https://www.youtube.com/watch?v=ojhHVL0t34I

# Example usage
path_index_to_plot = 2  # Change this to the desired path index

plot_all_joint_data(close_data, path_index_to_plot)
# To plot data for close paths, use: plot_all_joint_data(close_data, path_index_to_plot)
