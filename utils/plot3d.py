import yaml
import sys
import cv2
import numpy as np
import pprint


def tupleToXYArray(points):
    arr = [[],[]]
    for p in points:
        arr[0].append(p[0])
        arr[1].append(p[1])

    return arr


def getCorrespondingPoints(loader,dataset):
    #skip first line
    p1_all = []
    p2_all = []
    for index,row in enumerate(dataset[0][1:]):
        points1 = loader.parseRow(row)

        #+1 because we ignore firt line
        points2 = loader.parseRow(dataset[1][index+1])

        p1_all += points1
        p2_all += points2

    p1_all = tupleToXYArray(p1_all)
    p2_all = tupleToXYArray(p2_all)

    return (np.array(p1_all), np.array(p2_all))




def loadPoints(file1,file2):
    from plot import loadTopicData

    loader = loadTopicData()

    files = [file1, file2]

    dataset = loader.loadFiles(files)

    points = getCorrespondingPoints(loader,dataset)

    return points




def loadProjectionPatrix(filename):
    f = open(filename)
    dataMap = yaml.safe_load(f)
    f.close()

    data = dataMap['projection_matrix']['data']
    #create numpy array
    rows = dataMap['projection_matrix']['rows']
    cols = dataMap['projection_matrix']['cols']
    p = np.array(data).reshape((rows,cols))

    return p


if len(sys.argv) != 5:
    print("Error: not enough input arguments!\n")
    print("usage: plot3d.py left-yaml-file right-yaml-file points-left points-right")
    print("\nexample:\n\tplot3d.py ../ttyUSB0.yaml ../ttyUSB1.yaml points-left.csv points-right.csv")
    exit()

points1,points2 = loadPoints(sys.argv[3],sys.argv[4])

#projection matrix of camera 1
P1 = loadProjectionPatrix(sys.argv[1])
P2 = loadProjectionPatrix(sys.argv[2])

#P1[0][3] = -0.12
print("P1",P1)
print("P2",P2)


"""
#input points 1
rows = 2
#number of points
cols = 2
points1 = np.array([117.627388,105.089256287, 128-77.5111198425, 128-77.06656646729999]).reshape((rows,cols))
#input points 2
points2 = np.array([77.6499099731,63.3031005859, 128-77.766292572, 128-77.45426940920001]).reshape((rows,cols))
"""

print("2d points",points1,points2)
points3d = cv2.triangulatePoints(P1,P2,points1,points2)

print("3d points",points3d)

#homogenous coordinates: x,y,z,w
#make it homogenous
points3d /= points3d[3]

#normalized
print("3d points",points3d)



#plot in 3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#set bounding box and equal scaling on all axes
ax.set_aspect('equal')

dst = 0.4
ax.set_xlim(points3d[0].mean()-dst,points3d[0].mean()+dst)
ax.set_ylim(points3d[1].mean()-dst,points3d[1].mean()+dst)
ax.set_zlim(points3d[2].mean()-dst,points3d[2].mean()+dst)



import matplotlib.cm as cm
colors = cm.rainbow(np.linspace(0, 1, 20))

#for every 12 points (we always get 12 points at once, because this is our led pattern)
led_points = 12
for a in range(0,len(points3d[0])/led_points):
    ax.scatter(points3d[0][a*12:(a+1)*12],points3d[1][a*12:(a+1)*12],points3d[2][a*12:(a+1)*12],color=colors[(2*a)%20], alpha=0.5)

#first camera
baseline = P1[0][3]/(-P1[0][0])
print("baseline is ",baseline)
ax.scatter(baseline,0,0,color='red')

#second camera
baseline = P2[0][3]/(-P2[0][0])
print("baseline is ",baseline)
ax.scatter(baseline,0,0,color='red')

plt.show()

