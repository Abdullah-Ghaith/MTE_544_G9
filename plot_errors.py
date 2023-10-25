import matplotlib.pyplot as plt
from utilities import FileReader

AXIS0_LABELS = [["e (rad)", "e_dot (rad)"], ["e (m)", "e_dot (m)"], ["x (m)", "y (m)"]]

AXIS1_YLABELS = ["Error (rad)", "Error (m)", "Position/Orientation"]
LEGEND_BACKING = [" angular", " linear", " linear"]

TITLES = [["Angular Error State Space", "Angular Error vs Time"], ["Linear Error State Space", "Linear Error vs Time"], ["Robot Movement State Space", "Robot Pose vs Time"]]

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))

    labels = AXIS0_LABELS.pop()
    x_label = labels[0]
    y_label = labels[1]
    titles = TITLES.pop()
    axis_0_title = titles[0]
    axis_1_title = titles[1]
    
    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[0].set_title(axis_0_title)
    axes[0].grid()
    axes[0].set_ylabel(y_label)
    axes[0].set_xlabel(x_label)

    
    ylabel = AXIS1_YLABELS.pop()
    backing = LEGEND_BACKING.pop()
    axes[1].set_xlabel('time (ns)')
    axes[1].set_ylabel(ylabel)
    axes[1].set_title(axis_1_title)
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ backing)

    axes[1].legend()
    axes[1].grid()

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



