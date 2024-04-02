import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.pyplot import cm
import argparse
from io import StringIO
import numpy as np

parser = argparse.ArgumentParser(description="Plot NEMA motor position controlled data")
parser.add_argument("--file", type=str, default="./data/oneeigth/data2000000.csv", help="path to csv file")
args = parser.parse_args()

with open(args.file, "r") as f:
    data = f.read()

split_data = data.split("\n\n")[:-1]
# print(split_data[0])

color = cm.rainbow(np.linspace(0, 1, len(split_data)))

targets = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 200, 300, 400, 500, 1000] 
fig, ax = plt.subplots(nrows = 3, ncols=len(targets)//3, num=args.file)
for i, split in enumerate(split_data):
    df = pd.read_csv(StringIO(split), sep=",", header=None)
    times = df[0].to_numpy()
    locations = df[1].to_numpy()
    speeds = [y - x for x, y in zip(locations, locations[1:])]
    accelerations = [y - x for x, y in zip(speeds, speeds[1:])]
    ax[i % 3][i // 3].scatter(times, locations, c=[color[i]], marker='.')
    # ax[i % 3][i // 3].scatter(times[:-1], speeds, c=[color[i]], marker='.')
    ax[i % 3][i // 3].set_title(targets[i])
plt.show()