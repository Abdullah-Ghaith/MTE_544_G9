import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
import os

def get_sub_plot_title(filename, folder):
    file = str(os.path.basename(filename))
    file = file.replace("robotPose_", "")
    file = file.replace("_", ".")
    file = file.replace("Q", "Q=")
    file = file.replace(".R.", " R=")
    file = file.replace(".csv", "")
    file = file.replace(".robotPose", "")
    
    return f"{folder} {file}"

def plot_errors(filename, folder):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    fig, axes = plt.subplots(2,1, figsize=(14,6))

    fig.suptitle(get_sub_plot_title(filename, folder))
    
    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values])
    axes[0].set_title("state space")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    axes[1].set_xlabel("Time (ns)")
    axes[1].set_ylabel("Acceleration (m/s^2)")
    for i in range(4):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()
    
    plt.subplots_adjust(hspace=0.3)

    plt.show()

def get_csv_files(folder):
    csv_files = []
    folder_path = os.path.join(os.getcwd(), folder)
    for filename in os.listdir(folder_path):
        if filename.endswith(".csv"):
            csv_files.append(os.path.join(folder_path, filename))
    return csv_files

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--folder', required=True, help='Folder of files to process')
    
    args = parser.parse_args()
    
    print("plotting the filesn in folder", args.folder)

    folder=args.folder
    csv_files_in_folder = get_csv_files(folder)
    
    for filename in csv_files_in_folder:
        plot_errors(filename, folder)


