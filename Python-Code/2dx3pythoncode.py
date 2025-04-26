import serial
import numpy as np
import math

# CONFIGURATION
PORT = 'COM3'      # port number of my laptop is com3
BAUDRATE = 115200 
POINTS_PER_SCAN = 32
DEGREE_STEP = 360 / POINTS_PER_SCAN
Z_INCREMENT = 10         # How much to raise Z after each scan for exageration

xyz_filename = "tof_radar.xyz"
scan_number = 0 #tracks the z-layer height 
all_points = [] #accumulates all xyz points from each scan

# SERIAL SETUP
try: #tries to open the serial port 
    ser = serial.Serial(PORT, BAUDRATE, timeout=1) #opens the serial port for reading from tmc, timeout = 1 makes sure readline wont hang forever 
    print(f"Connected to {PORT} @ {BAUDRATE} baud") #connection message 
except Exception as e:
    print(f"Failed to connect to serial port: {e}") #error message if it cannot connnect 
    exit()

def polar_to_cartesian(angle_deg, distance_mm, z_height):
    angle_rad = math.radians(angle_deg) #converting to x y cooridnates 
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return [x, y, z_height]

#z height is passed in to tag the etnire scan layer
def capture_scan(z_height):
    points = [] #local list to store jsut this scan's 32 points 
    count = 0 #tracks how many valid distance readings have been recieved 
    print("Receiving points...")
    while count < POINTS_PER_SCAN: #loops until 32 points have been recieved
        try:
            line = ser.readline().decode('utf-8').strip() #reads a full line of uart, .decode covnerts the raw bytes to a string, .strip removes newline characters such as spaces,etc
            if not line.isdigit(): #skips over the line if it is not purely digital 
                continue
            distance = int(line) #converts line to an integer
            angle = count * DEGREE_STEP #calculating angle of the scan point
            point = polar_to_cartesian(angle, distance, z_height) #converitng polar coordinates to xyz format 
            points.append(point) #adding pint to the local scan list 
            count += 1#increasing poiont count
            print(f"Point {count:02}: {distance} mm -> {point}") #visual of point count and distance 
        except Exception as e:
            print(f"Serial error during scan: {e}") #error message for if any errors occur 
    return points #returns all points 

#MAIN LOOP
while True:
    #wait for enter press on python terminal to start showing points 
    input("Press Enter to capture one 360-degree scan...")
    z = scan_number * Z_INCREMENT #calcualting z height based on scan number 
    scan_points = capture_scan(z) #calls capture scan to collect the 32 new points at this height 
    all_points.extend(scan_points) #appending all new poitns to list 



    # Saveing to .xyz file 
    try:
        np.savetxt(xyz_filename, np.array(all_points), fmt='%.2f')
        print(f"Saved {len(all_points)} points to {xyz_filename}\n")
    except Exception as e:
        print(f"Error saving file: {e}")

    scan_number += 1
