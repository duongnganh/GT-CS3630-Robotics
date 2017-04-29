# Wang, Yongxin
# Nguyen, Anh Duong

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import asyncio
import time

# A helper data structure for backtracking purposes. 
# Made this class for the AI to find the path after performing search
class Node:
    def __init__(self, cell, parent):
        self.cell = cell
        self.parent = parent
    def __repr__(self):
        return "[" + str(self.state) + ", " + str(self.parent) + "]"

    def __cmp__(self, other):
        return 0
    def __lt__ (self, other):
        return self.cell < other.cell

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    print (grid.getStart())
    frontier = PriorityQueue()
    frontierCpy = {}

    goal = grid.getGoals()[0]
    # ((x, y), (parentX, parentY), cost)
    startX = grid.getStart()[0]
    startY = grid.getStart()[1]
    startNode = Node(((startX, startY), 0), None)

    init_heu = heuristic(startNode.cell[0], goal)
    frontierCpy[startNode.cell[0]] = init_heu
    frontier.put((init_heu, 0, startNode))

    while frontier.qsize() != 0:
        tup = frontier.get()

        currNode = tup[2]
        currG = tup[1] * -1
        grid.addVisited(currNode.cell[0])
        frontierCpy.pop(currNode.cell[0], None)

        if currNode.cell[0] == goal:
            path = []
            while currNode != None:
                path.insert(0, currNode.cell[0])
                currNode = currNode.parent
            grid.setPath(path)
            return path


        neighbors = grid.getNeighbors(currNode.cell[0])
        # [((x, y), distance), ]
        for n in neighbors:
            if n[0] not in grid.getVisited():
                newNode = Node(n, currNode)

                h = heuristic(n[0], goal)

                oneStepCost = n[1]
                g = oneStepCost + currG
                if n[0] not in frontierCpy or frontierCpy[n[0]] > h + g:
                    frontier.put((h+g, -1*g, newNode))
                    frontierCpy[n[0]] = h+g


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    # return 1
    i = current[0] - goal[0]
    j = current[1] - goal[1]
    return math.sqrt(math.pow(i,2) + math.pow(j,2)) # Your code here
    # return math.fabs(current[0] - goal[0]) + math.fabs(current[1] - goal[1])

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment document for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent

    foundGoal = False

    setPath = False
    
    while not stopevent.is_set():
        if grid.getGoals() == []:
            # find goal
            look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            cube = None

            try:
                cube = robot.world.wait_for_observed_light_cube(timeout=30)
                print("Found cube: %s" % cube)
                foundGoal = True
                pos = cube.pose.position
                x, y, z = pos.x, pos.y, pos.z
                cube_angle = cube.pose.rotation.angle_z.degrees
                a, b = 0, 0
                if cube_angle > -22.5 and cube_angle < 22.5:
                    a, b = -1, 0
                if cube_angle >= 22.5 and cube_angle < 67.5:
                    a, b = -1, 1
                if cube_angle >= 67.5 and cube_angle < 112.5:
                    a, b = 0, 1
                if cube_angle >= 112.5 and cube_angle < 157.5:
                    a, b = 1, 1
                if cube_angle >= 157.5 or cube_angle <= -157.5:
                    a, b = 1, 0
                if cube_angle > -67.5 and cube_angle <= -22.5:
                    a, b = -1, -1
                if cube_angle > -112.5 and cube_angle <= -67.5:
                    a, b = 0, -1
                if cube_angle > -157.5 and cube_angle <= -112.5:
                    a, b = 1, -1

                obs1 = int(x/grid.scale + grid.getStart()[0] + 0.5), int(y/grid.scale + grid.getStart()[1] + 0.5)
                add_obs = []
                for i in range(-1, 2, 1):
                    for j in range(-1, 2, 1):
                        ob = obs1[0] + i, obs1[1] + j
                        add_obs.append(ob)
                grid.addObstacles(add_obs);
                goal = obs1[0] + a * 2, obs1[1] + b * 2
                print(goal)
                grid.addGoal(goal)


            except asyncio.TimeoutError:
                print("Didn't find a cube")
            finally:
                # whether we find it or not, we want to stop the behavior
                look_around.stop()
        else:

        # astar
        # astar(grid)
        # get path
        # path = grid.getPath() #a list of cells
            
            robot_pose = robot.pose
            print(robot_pose)
            rx, ry = robot_pose.position.x, robot_pose.position.y
            cx = int(rx/grid.scale + grid.getStart()[0] + 0.5)
            cy = int(ry/grid.scale + grid.getStart()[1] + 0.5)
            # update start
            grid.clearStart()
            grid.setStart((cx, cy))

            astar(grid, heuristic)
            path = grid.getPath()
            print(path)
            prev = path[0]
            for p in path[1:]:
                diff = p[0] - prev[0], p[1] - prev[1]
                angle = getAngle(diff)

                posX, posY = grid.scale * (p[0] - 1), grid.scale * (p[1] - 1)
                # posX, posY = 25 * p[0] - rx, 25 * p[1] - ry
                # pose = cozmo.util.Pose(posX, posY, 0.0, q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0)

                # Everytime we move, check to see if there's an obstacle in our vision
                pose = cozmo.util.Pose(posX, posY, 0.0, angle_z = cozmo.util.degrees(angle))
                robot.go_to_pose(pose, in_parallel = True).wait_for_completed()
                # robot.go_to_pose(pose)
                # time.sleep(0.1)

                print(p)
                print(robot.pose.position.x, robot.pose.position.y)
                print(posX, posY)

                prev = p
            ca = -1.0*cube_angle
            pose = cozmo.util.Pose(robot.pose.position.x, robot.pose.position.y, robot.pose.position.z, angle_z = cozmo.util.degrees(ca))
            robot.go_to_pose(pose).wait_for_completed()

            break # Your code here
def getAngle(cord):
    cord_list = [(1,0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (-1, -1)]
    angles = [0, 45, 90, 135, 180, -45, -90, -135]
    for i in range(len(cord_list)):
        if cord == cord_list[i]:
            return angles[i]
    return 0

######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()