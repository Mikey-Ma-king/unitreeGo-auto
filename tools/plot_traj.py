import matplotlib.pyplot as plt
import sys
import os

# cmd_vx,cmd_vy,cmd_raw_rate,yaw_rate,x,y,z,w,i,j,k,vx,vy,vz
import csv

Xs = []
Ys = []
with open(os.path.join(sys.argv[1], 'v_cmd.csv')) as f:
    reader = csv.DictReader(f)
    for row in reader:
        print(row)
        Xs.append(float(row['x']))
        Ys.append(float(row['y']))

plt.plot(Xs, Ys)

# dest path
# with open(os.path.join(sys.argv[1], 'p_target.csv')) as f:
#     x, y, z = map(float, f.read()[1:-2].split())
#     plt.scatter([x], [y])

plt.axis('equal')
plt.show()
