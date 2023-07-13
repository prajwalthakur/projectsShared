import csv
import os
import matplotlib.pyplot as plt
import numpy as np
import pdb
with open("weight_data.csv", newline='') as f:
    reader = csv.reader(f)
    my_list = [row[-1] for row in reader]
fig = plt.figure(1, figsize=(12, 8), dpi=100)
plt.title("reward Vs iteration")
v = len(my_list[0:600])
ax = fig.add_subplot(111)
ax.set_xlim([0,v])
ax.set_ylim([0.0,0.40])
trajct = np.asarray(my_list[0:600]).astype(np.float)
ax.plot(trajct)
plt.savefig("rewardVsIteration2.jpg")
