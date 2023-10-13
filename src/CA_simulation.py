# -*- coding: utf-8 -*-

# Simulation with several parameter values for different CA algorithms and strategies

import os
import subprocess
import time
import yaml

global PF_CRIT_DIST_values
global PF_AZIMUTH_ERROR_values
global W1_values
global W2_values
global time_step_values
global time_horizon_values
global RVO_CRIT_DIST_values
global RVO_AZIMUTH_ERROR_values
global W1_pointer
global W2_pointer
global time_step_pointer
global time_horizon_pointer
global CRIT_DIST_pointer
global AZIMUTH_ERROR_pointer
global STOP_AND_WAIT
global ALGOR
global END_SIMULATION
global first
global iterations
global i

PF_CRIT_DIST_values = [6.0, 7.0, 10.0, 20.0]
PF_AZIMUTH_ERROR_values = [0.8, 0.6, 0.4 ,0.2]
RVO_CRIT_DIST_values = [6.0, 7.0, 10.0, 20.0]
RVO_AZIMUTH_ERROR_values = [0.8, 0.6, 0.4 ,0.2]
DES_VEL_values = [0.5]
W1_values = [1.0]
W2_values = [0.5, 2.5, 500.0]
time_step_values = [1.0, 0.5, 0.1]
time_horizon_values = [20.0, 10.0, 5.0]
CRIT_DIST_pointer = 0
DES_VEL_pointer = 0
AZIMUTH_ERROR_pointer = 0
W1_pointer = 0
W2_pointer = 0
time_step_pointer = 0
time_horizon_pointer = 0
STOP_AND_WAIT = False
ALGOR = 1
END_SIMULATION = False
first = True
iterations = 4
i = 0

def main():
    launch_file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/launch/collision_avoidance.launch'

    while True:
        # Modify parameters.yaml
        # If all parameter values have been simulated then stop simulating
        if modify_parameters():
            break

        # Start simulation in a separate process
        launch_process = subprocess.Popen(['roslaunch', launch_file_path])

        while True:
            if check_ros_processes():
                # Record results topic
                os.chdir('/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density')
                bag_process = subprocess.Popen(['rosbag', 'record', '-a', '-O', 'results.bag'])
                break
            time.sleep(5)

        # Wait for the launch process to finish before continuing
        launch_process.wait()

        # Wait for the bag file to be created before running rostopic
        bag_file_path = '/home/arm792/MRS/src/MRS_stack/collision_avoidance/simulation_results/high_density/results.bag'
        while not os.path.exists(bag_file_path):
            time.sleep(1)

        # Run rostopic to extract data
        rostopic_process = subprocess.Popen(['rostopic', 'echo', '-b', bag_file_path, '-p', '/results'], stdout=subprocess.PIPE)
        csv_data, _ = rostopic_process.communicate()

        # Append the extracted data to the CSV file
        with open('testing_results.csv', 'a') as csv_file:
            csv_file.write(csv_data.decode())  # Write the data to the CSV file

        # Remove the bag file if it still exists
        if os.path.exists(bag_file_path):
            os.remove(bag_file_path)

def check_ros_processes():
    process = subprocess.Popen('rosnode list', shell=True, stdout=subprocess.PIPE)
    output = process.stdout.read().decode()
    return output

def modify_parameters():
    global PF_CRIT_DIST_values
    global PF_AZIMUTH_ERROR_values
    global W1_values
    global W2_values
    global time_step_values
    global time_horizon_values
    global RVO_CRIT_DIST_values
    global RVO_AZIMUTH_ERROR_values
    global W1_pointer
    global W2_pointer
    global time_step_pointer
    global time_horizon_pointer
    global CRIT_DIST_pointer
    global AZIMUTH_ERROR_pointer
    global STOP_AND_WAIT
    global ALGOR
    global END_SIMULATION
    global first
    global iterations
    global i

    if not first:
        if ALGOR == 1 and not STOP_AND_WAIT:
            W1_pointer += 1

            if W1_pointer >= len(W1_values):
                W1_pointer = 0
                W2_pointer += 1

            if W2_pointer >= len(W2_values):
                W2_pointer = 0
                AZIMUTH_ERROR_pointer += 1    

            if AZIMUTH_ERROR_pointer >= len(PF_AZIMUTH_ERROR_values):
                AZIMUTH_ERROR_pointer = 0
                CRIT_DIST_pointer += 1  

            if CRIT_DIST_pointer >= len(PF_CRIT_DIST_values):
                CRIT_DIST_pointer = 0
                #STOP_AND_WAIT = True
                ALGOR = 2
        
        elif ALGOR == 1 and STOP_AND_WAIT:
            STOP_AND_WAIT = False
            ALGOR = 2
        
        elif ALGOR == 2 and not STOP_AND_WAIT:
            time_step_pointer += 1

            if time_step_pointer >= len(time_step_values):
                time_step_pointer = 0
                time_horizon_pointer += 1    

            if time_horizon_pointer >= len(time_horizon_values):
                time_horizon_pointer = 0
                AZIMUTH_ERROR_pointer += 1  

            if AZIMUTH_ERROR_pointer >= len(RVO_AZIMUTH_ERROR_values):
                AZIMUTH_ERROR_pointer = 0
                CRIT_DIST_pointer += 1

            if CRIT_DIST_pointer >= len(RVO_CRIT_DIST_values):  
                CRIT_DIST_pointer = 0    
                #STOP_AND_WAIT = True 
                ALGOR = 1
                
                i += 1

                if i == iterations:
                    END_SIMULATION = True 
        
        elif ALGOR == 2 and STOP_AND_WAIT:
            ALGOR = 1
            STOP_AND_WAIT = False         
    
    first = False

    yaml_file_path = "/home/arm792/MRS/src/MRS_stack/collision_avoidance/config/parameter_values/parameters.yaml"

    if ALGOR == 1:
        AZIMUTH_ERROR_values = PF_AZIMUTH_ERROR_values
        CRIT_DIST_values = PF_CRIT_DIST_values
    
    else:
        AZIMUTH_ERROR_values = RVO_AZIMUTH_ERROR_values
        CRIT_DIST_values = RVO_CRIT_DIST_values

    with open(yaml_file_path, 'r') as yaml_file:
        data = yaml.load(yaml_file, Loader=yaml.Loader)

        if 'parameter_values' in data and isinstance(data['parameter_values'], list):
            for param in data['parameter_values']:
                if param['name'] == 'STOP_AND_WAIT':
                    param['value'] = STOP_AND_WAIT
                
                elif param['name'] == 'ALGOR':
                    param['value'] = ALGOR

                elif param['name'] == 'AZIMUTH_ERROR':
                    param['value'] = AZIMUTH_ERROR_values[AZIMUTH_ERROR_pointer]
                
                elif param['name'] == 'CRIT_DIST':
                    param['value'] = CRIT_DIST_values[CRIT_DIST_pointer]
                
                elif param['name'] == 'W1':
                    param['value'] = W1_values[W1_pointer]
                
                elif param['name'] == 'W2':
                    param['value'] = W2_values[W2_pointer]
                    
                elif param['name'] == 'time_step':
                    param['value'] = time_step_values[time_step_pointer]
                
                elif param['name'] == 'time_horizon':
                    param['value'] = time_horizon_values[time_horizon_pointer]

                elif param['name'] == 'DES_VEL':
                    param['value'] = DES_VEL_values[DES_VEL_pointer]

    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)
    
    if END_SIMULATION:
        return True
    
    else:
        return False

if __name__ == "__main__":
    main()
