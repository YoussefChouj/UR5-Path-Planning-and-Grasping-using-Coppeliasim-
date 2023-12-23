Setting Up the Virtual Environment and Installing Dependencies :
To replicate the results of this project, you need to set up a virtual environment and install the necessary dependencies.

Prerequisites
Anaconda 
CoppeliaSim 
Creating and Activating a Virtual Environment
Clone the Repository:

Clone this repository to your local machine using git clone https://github.com/YoussefChouj/UR5-Path-Planning-and-Grasping-using-Coppeliasim-.git.
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

