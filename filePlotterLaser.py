import matplotlib.pyplot as plt
import pandas as pd
import argparse
import math
import numpy as np

'''
# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
import math

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)
        for i, dist in enumerate(val):
            rad = i * 2 * math.pi / 720
            x = 0
            y = 0
            if not math.isnan(dist) and math.isinf(dist):
                
                
            

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.grid()
    plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)

'''

def plot_errors(csv_file):
    df = pd.read_csv(csv_file, header=None, usecols=list(range(720)))  # Read the second row of the first 720 columns

    data = df.iloc[64].values

    x_values = []
    y_values = []

    for i, value in enumerate(data):
        value = float(value)
        if not np.isnan(value) and not np.isinf(value):
            angle = i * 0.5  # Angle in degrees
            angle_rad = math.radians(angle)  # Convert angle to radians

            x = value * math.cos(angle_rad)
            y = value * math.sin(angle_rad)

            x_values.append(x)
            y_values.append(y)
        else:
            x_values.append(0)
            y_values.append(0)

    plt.scatter(x_values, y_values, label='Data Points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Laser Scan Data Plot')
    plt.grid()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', required=True, help='CSV file to process')

    args = parser.parse_args()
    print("Plotting the CSV file:", args.files)

    if args.files is None:
        print("No CSV file provided")
        exit(1)


    plot_errors(args.files)
