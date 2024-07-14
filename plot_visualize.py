import argparse
import matplotlib.pyplot as plt

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file_path", type=str, default="./metric.txt")
    parser.add_argument("--type", type=int, default=2, help="0: time, 1: Point-to-Point RMSE, 2: Point-to-Plane RMSE")
    args = parser.parse_args()

    liter, ltime, lrmse1, lrmse2 = [], [], [], []
    file_path = args.file_path
    with open(file_path) as f:
        lines = f.readlines()
        for line in lines:
            iter, time, rmse1, rmse2 = line.split(",")
            liter.append(int(iter))
            ltime.append(float(time))
            lrmse1.append(float(rmse1))
            lrmse2.append(float(rmse2))
    
    if(args.type == 0):
        data = ltime
        ylabel = "Time"
    elif(args.type == 1):
        data = lrmse1
        ylabel = "Point To Point RMSE"
    elif(args.type == 2):
        data = lrmse2
        ylabel = "Point To Plane RMSE"

    xlabel = "Linear ICP"
    plt.title(xlabel)
    plt.xlabel('Number of Iterations')
    plt.ylabel(ylabel)
    plt.plot(liter, data, c='orange')
    plt.xticks(liter)
    plt.show()