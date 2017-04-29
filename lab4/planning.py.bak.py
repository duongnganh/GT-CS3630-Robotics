
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo

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
    frontierCpy = []

    goal = grid.getGoals()[0]
    # ((x, y), (parentX, parentY), cost)
    startX = grid.getStart()[0]
    startY = grid.getStart()[1]
    startNode = Node(((startX, startY), 0), None)

    init_heu = heuristic(startNode.cell[0], goal)
    frontierCpy.append((startNode.cell[0], init_heu))
    frontier.put((init_heu, 0, startNode))

    while frontier.qsize() != 0:
        tup = frontier.get()

        currNode = tup[2]
        currG = tup[1] * -1
        grid.addVisited(currNode.cell[0])
        frontierCpy.remove((currNode.cell[0], tup[0]))

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
            # if n[0] not in grid.getVisited():
                # if (n[0], dist) not in frontierCpy:
                    newNode = Node(n, currNode)

                    h = heuristic(n[0], goal)

                    oneStepCost = n[1]
                    # cpy = currNode
                    # while cpy.parent != None:
                    #     g += cpy.cell[1]
                    g = oneStepCost + currG
                    if (n[0], h+g) not in frontierCpy:
                        frontier.put((h+g, -1*g, newNode))
                        frontierCpy.append((n[0], h+g))
                    # a list of tuple
        

    # pass # Your code here


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
    
    while not stopevent.is_set():
        pass # Your code here


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

