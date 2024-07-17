import os
import argparse
import matplotlib.pyplot as plt
import numpy as np

# Aliases
_MAP_NAMES = {
    "rmse_nn_metric": "RMSE Pointclouds (nearest)",
    "eval_r": "Frobenius Norm For Rotation",
    "eval_t": "RMSE Translation",
}

if __name__ == "__main__":
    path = "C:/TUM/vis/average"
    folderlist = os.listdir(path)
    for foldername in folderlist:
        outputdir = f'C:/TUM/vis/figs/{foldername}'
        if (os.path.exists(outputdir) == False):
            os.mkdir(outputdir)
        
        nframes = os.listdir(f"{path}/{foldername}")
        tmp = foldername.split('_')
        if(len(foldername.split('_')) == 3):
            icp, linear, correspondence = foldername.split('_')
            title = f"ICP: {icp} / {linear} / {correspondence}"
        elif(len(foldername.split('_')) == 4):
            icp, linear, correspondence, color = foldername.split('_')
            title = f"ICP: {icp} / {linear} / {correspondence} / {color}"
       
        for metric_type in _MAP_NAMES:
            file_path = f"{path}/{foldername}/{metric_type}.txt"
            iters, mean_values = [], []

            with open(file_path) as f:
                lines = f.readlines()
                for line in lines:
                    iter, value = line.split(",")
                    iters.append(int(iter))
                    mean_values.append(float(value))
            
            mean_values = np.array(mean_values)
            std_values = mean_values.std()

            plt.title(f'{_MAP_NAMES[metric_type]} averaged across all meshes')
            plt.xlabel("Iteration")
            plt.ylabel(_MAP_NAMES[metric_type])
            plt.plot(iters, mean_values)
            plt.fill_between(iters, mean_values - std_values, mean_values + std_values, alpha=0.2)
            plt.xticks(iters)
            plt.savefig(f'{outputdir}/{metric_type}.png') 
            #plt.show()
            plt.close()