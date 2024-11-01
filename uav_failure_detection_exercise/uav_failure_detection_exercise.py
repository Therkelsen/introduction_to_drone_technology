#!/usr/bin/env python3

'''
Copyright 2024 Kjeld Jensen <kjen@sdu.dk>
SDU UAS Center
University of Southern Denmark

Course: Introduction to Drone Technology
Module: UAV attitude failure detection
This code has been tested to work under Ubuntu 22.04 and 24.04

2024-10-31 KJ First version
'''


'''
You need to install pyulog, however in Ubuntu 24.04 you should do
this in a virtual environment. Run the commands below to create
a virtual environment and install pyulog into this virtual environment

sudo apt install python3-pip python3-venv

python3 -m venv ~/python3-ulog-venv

~/python3-ulog-venv/bin/pip install pyulog

You will also need these for plotting:

~/python3-ulog-venv/bin/pip install matplotlib 
~/python3-ulog-venv/bin/pip install pyqt5 

Now when you wish to execute this script, you need to run this command instead of the usual python3. Please note that your active directory has to be containing the ulog file:

~/python3-ulog-venv/bin/python uav_failure_detection_exercise.py
'''

from pyulog import ULog
import matplotlib.pyplot as plt

def plot_result(start_s, end_s, pressure_time, pressure_log, kill_sw_time, kill_sw_log):
    fig, ax = plt.subplots(1, 2)
    
    # acceleration plot:
    ax[0].plot(pressure_time, pressure_log, linewidth=0.5, color='green')
    ax[1].plot(kill_sw_time, kill_sw_log, linewidth=0.5, color='blue')

    ax[0].set(xlabel='Time [s]', ylabel='Pressure [hPa]', xlim=[start_s, end_s])
    ax[1].set(xlabel='Time (s)', ylabel='Kill switch', xlim=[start_s, end_s])

    plt.tight_layout()
    plt.show()

#####################################################################
'''
This is the section where you have to modify the code to create your
UAV failure detection algorithm. For now the algorithm only prints
the values and some are plotted at the end.

At these links you will find videos of the flights corresponding to
the log files:

Test 5: https://youtu.be/raK2fnk5ULk
Test 8: https://youtu.be/gJK06kHUvz8
Test 9: https://youtu.be/opDSC2ox-c4
'''

log_file_path = 'TEST5_30-01-19.ulg'
#log_file_path = 'TEST8_30-01-19.ulg'
#log_file_path = 'TEST9_08-02-19.ulg'

start_seconds = 100
end_seconds = 900
pressure_time = []
pressure_log = []
kill_sw_time = []
kill_sw_log = []

def algorithm_init():
    print ('Algorithm init')

def algorithm_update(new_data):
    # if the data is within our time interval of interest
    if new_data['timestamp'] > start_seconds and new_data['timestamp'] < end_seconds:

        # if new sensor data is available
        if new_data['field'] == 'sensor_combined':
            print (new_data['timestamp'], 'imu', new_data['gyro_rad_x'], new_data['gyro_rad_y'], new_data['gyro_rad_z'], new_data['acc_x'], new_data['acc_y'], new_data['acc_z'])
        
        # if new barometer data is available
        if new_data['field'] == 'vehicle_air_data':
            print (new_data['timestamp'], 'pressure', new_data['baro_pressure_pa'])
            pressure_time.append(new_data['timestamp'])
            pressure_log.append(new_data['baro_pressure_pa'])

        # if the kill switch status has been changed
        if new_data['field'] == 'manual_control_setpoint':
            print (new_data['timestamp'], 'kill switch', new_data['aux1'])
            kill_sw_time.append(new_data['timestamp'])
            kill_sw_log.append(new_data['aux1'])

def algorithm_done():
    print ('Algorithm done')
    plot_result(start_seconds, end_seconds, pressure_time, pressure_log, kill_sw_time, kill_sw_log)

#####################################################################

def load_data(log_file_path):
    # Load the ULog file
    ulog = ULog(log_file_path)
    d = []
    
    # loop through all data types in the ulog file
    for data in ulog.data_list:
        #print (data.name)

        if data.name == 'sensor_combined':
            data_keys = [f.field_name for f in data.field_data]
            print ('Keys', data_keys)
            timestamps = data.data['timestamp']
            gyro_rad_x = data.data['gyro_rad[0]']
            gyro_rad_y = data.data['gyro_rad[1]']
            gyro_rad_z = data.data['gyro_rad[2]']
            accelerometer_x = data.data['accelerometer_m_s2[0]']
            accelerometer_y = data.data['accelerometer_m_s2[1]']
            accelerometer_z = data.data['accelerometer_m_s2[2]']
            for ts, g_x, g_y, g_z, acc_x, acc_y, acc_z in zip(timestamps, gyro_rad_x, gyro_rad_y, gyro_rad_z, accelerometer_x, accelerometer_y, accelerometer_z):
                d.append({"timestamp": int(ts)/1000000., "field": data.name, "gyro_rad_x": float(g_x), "gyro_rad_y": float(g_y), "gyro_rad_z": float(g_z), "acc_x": float(acc_x), "acc_y": float(acc_y), "acc_z": float(acc_z)})

        if data.name == 'vehicle_air_data':
            data_keys = [f.field_name for f in data.field_data]
            print ('Keys', data_keys)        
            timestamps = data.data['timestamp']
            pressures = data.data['baro_pressure_pa']
            for ts, pres in zip(timestamps, pressures):
                d.append({"timestamp": int(ts)/1000000., "field": data.name, "baro_pressure_pa": float(pres)})

        if data.name == 'manual_control_setpoint':
            data_keys = [f.field_name for f in data.field_data]
            print ('Keys', data_keys)        
            timestamps = data.data['timestamp']
            aux1 = data.data['aux1']
            for ts, a1 in zip(timestamps, aux1):
                d.append({"timestamp": int(ts)/1000000., "field": data.name, "aux1": float(a1)})

    # sort all data using the timestamp (first entry) as key
    sorted_data = sorted(d, key=lambda x: list(x.values())[0])
    return sorted_data    

def run_simulation(data):
    algorithm_init()
    for d in data:
        algorithm_update(d)
    algorithm_done()

if __name__ == "__main__":
    print ('Loading data from the ulog file')
    data = load_data(log_file_path)

    print ('Run simulation')
    run_simulation(data)

    print ('Simulation completed')

