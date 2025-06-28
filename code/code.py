from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt
import random

# Load and process the image
img = Image.open('/Users/babar/Downloads/map2.png')
img = ImageOps.grayscale(img)
np_img = np.array(img)
np_img = ~np_img  # invert B&W
np_img[np_img > 0] = 1
plt.set_cmap('binary')
plt.imshow(np_img)

# Save the processed image
np.save('cspace.npy', np_img)

# Read the saved image
grid = np.load('cspace.npy')

# Define the starting point and goal
start = np.array([100.0, 100.0])
goal = np.array([773.0, 375.0])

# Number of iterations and step size for RRT
numIterations = 200
stepsize = 50

# Define the goal region as a circle
goalRegion = plt.Circle((goal[0], goal[1]), stepsize, color='b', fill=False)
fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

# Class definition for treeNode
class treeNode():
    def _init_(self, locationx, locationy):
        self.locationx = locationx
        self.locationy = locationy
        self.children = []
        self.parent = None

# Class definition for RRTAlgorithm
class RRTAlgorithm():
    def _init_(self, start, goal, numIterations, grid, stepsize):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(numIterations, 200)
        self.grid = grid
        self.rho = stepsize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []

    def addchild(self, locationx, locationy):
        if (locationx == self.goal.locationx):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationx, locationy)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode

    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point

    def steerToPoint(self, locationstart, locationEnd):
        offset = self.rho * self.unitVector([locationstart.locationx, locationstart.locationy], locationEnd)
        point = np.array([locationstart.locationx + offset[0], locationstart.locationy + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0] - 1
        return point

    def isInobstacle(self, locationstart, locationEnd):
        u_hat = self.unitVector([locationstart.locationx, locationstart.locationy], locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = locationstart.locationx + i * u_hat[0]
            testPoint[1] = locationstart.locationy + i * u_hat[1]
            if self.grid[round(testPoint[1]), round(testPoint[0])] == 1:
                return True
        return False


    def unitVector(self, locationstart, locationEnd):
        v = np.array([locationEnd[0] - locationstart[0], locationEnd[1] - locationstart[1]])
        u_hat = v / np.linalg.norm(v)
        return u_hat

    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
            pass

    def distance(self, node1, point):
        dist = np.sqrt((node1.locationx - point[0]) ** 2 + (node1.locationy - point[1]) ** 2)
        return dist

    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        pass

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def retraceRRTPath(self, goal):
        if goal.locationx == self.randomTree.locationx:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.locationx, goal.locationy])
        self.Waypoints.insert(0, currentPoint)
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)

rrt = RRTAlgorithm(start, goal, numIterations, grid, stepsize)
for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("Iteration: ", i)
    point = rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInobstacle(rrt.nearestNode, new)
    if bool == False:
        rrt.addchild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationx, new[0]], [rrt.nearestNode.locationy, new[1]], 'go', linestyle="--")
        if rrt.goalFound(new):
            rrt.addchild(goal[0], goal[1])
            print("Goal found!")
            break

rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0, start)
print("number of way points: ", rrt.numWaypoints)
print("path distance (m): ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints)

for i in range(len(rrt.Waypoints) - 1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i + 1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i + 1][1]], 'ro', linestyle="--")
    plt.pause(0.10)

plt.show()
