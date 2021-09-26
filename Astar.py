# A* algorithm along with helper methods

import numpy as np
import heapq as hq
import helper as hp
import visualize as vis
from maze import Maze

trajectoryUnknown = 0
cellsProcessed = 0

"""
mDistance - calculates manhattan distance from goal
:param x: point whose priority will be evaluated
:param G: goal
:param distances: array storing distances spaces are from the start
:return: priority number based on speed
"""


def mDistance(x, G, distances):
    xCoord = x[0]
    yCoord = x[1]
    xdist = G[0] - xCoord
    ydist = G[1] - yCoord
    # h = abs(xdist) + abs(ydist)  # calculates heuristic (comment out for #9)
    h = (abs(xdist) + abs(ydist)) * 1.6  # calculates weighted heuristic
    return distances[yCoord][xCoord][0] + h


"""
cDistance - calculates Chebyshev distance from goal
:param x: point whose priority will be evaluated
:param G: goal
:param distances: array storing distances spaces are from the start
:return: priority number based on speed
"""


def cDistance(x, G, distances):
    xCoord = x[0]
    yCoord = x[1]
    xdist = G[0] - xCoord
    ydist = G[1] - yCoord
    h = max(abs(xdist), abs(ydist))  # calculates heuristic
    return distances[yCoord][xCoord][0] + h


"""
eDistance - calculates euclidean distance from goal
:param x: point whose priority will be evaluated
:param G: goal
:param distances: array storing distances spaces are from the start
:return: priority number based on speed
"""


def eDistance(x, G, distances):
    xCoord = x[0]
    yCoord = x[1]
    xdist = G[0] - xCoord
    ydist = G[1] - yCoord
    h = np.sqrt((pow(xdist, 2)) + pow(ydist, 2))  # calculates heuristic
    return distances[yCoord][xCoord][0] + h


"""
AStarKnown - Performs A* search on maze with given parameters
*Assumes that all of the maze is known
:param maze: Grid to solve with blocked/unblocked cells
:param S: Start Point
:param G: Goal Point
:param heuristic: Heuristic calculation method
:param longReturn: Flag to determine if entire path is returned or only T/F solvable
:return: List of coords with path to goal if possible OR T/F if path exists
"""


def AStarKnown(maze, S, G, heuristic, longReturn):
    start = (0, S[0], S[1])  # store each coordinate as (f(n),x,y)
    fringe = []
    hq.heappush(fringe, start)  # create priority queue with the start as the only element in the fringe

    # Initialize array to keep track of f(n) per cell in grid and parents
    # Matrix allows for constant time access to all necessary information
    gn = [[(np.inf, None) for i in range(maze.dim)] for j in range(maze.dim)]
    gn[S[1]][S[0]] = (0, None)

    # Iteratively find best path
    helperAStar(maze, G, heuristic, fringe, gn)

    # Construct actual string path from result of Astar search if requested
    if longReturn:
        path = createPath(maze, S, G, gn)
        #print(path)
        return path

    # Return T/F is goal is reachable
    if gn[G[1]][G[0]][0] == np.inf:
        return False
    return True


"""
helperAStarKnown - A* Child Generation and Path Checking
:param maze: Grid to solve with blocked/unblocked cells
:param G: Goal Point
:param heuristic: Heuristic calculation method
:param fringe: Priority Queue of different cells
:param gn: Matrix containing distances of all cells and parents
"""


def helperAStar(maze, G, heuristic, fringe, gn):
    if not fringe:  # check for empty fringe meaning no remaining paths to check
        return

    prev = None
    while fringe:
        global cellsProcessed
        cellsProcessed += 1
        x = hq.heappop(fringe)  # pop first element and check if goal otherwise create children and call again
        if x[1] == G[0] and x[2] == G[1]:
            return
        createChildren(x, maze, G, heuristic, fringe, gn)
        prev = x
    return


"""
createChildrenKnown - Generates children of given point and stores in (gn,x,y) form in fringe when needed 
:param x: Point to generate children
:param maze: Grid to solve with blocked/unblocked cells
:param G: Goal Point
:param heuristic: Heuristic calculation method
:param fringe: Priority Queue of different cells
:param gn: Matrix containing distances of all cells and parents
"""


def createChildren(x, maze, G, heuristic, fringe, gn):
    xc = x[1]
    yc = x[2]
    newgn = gn[yc][xc][0] + 1  # note that python matrix storage is not normal x,y graph coordinates

    # generate legal children for all 4 possible moves based on origin point
    for direction in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
        newxc = direction[0] + xc
        newyc = direction[1] + yc

        if newxc >= maze.dim or newxc < 0 or newyc >= maze.dim or newyc < 0:  # Legal within bounds children only!
            continue

        oldgn = gn[newyc][newxc][0]
        if newgn < oldgn and maze.maze[newyc][newxc] == 0:  # add only if improved distance and open
            gn[newyc][newxc] = (newgn, (xc, yc))  # store new gn and parent coords
            item = (heuristic((newxc, newyc), G, gn), newxc, newyc)  # store value based on heuristic and x,y coords
            hq.heappush(fringe, item)
    return


"""
AStarUnknown - Performs A* search on maze with given parameters
*Assumes that all of the maze is not known
:param maze: Grid to solve with blocked/unblocked cells
:param S: Start Point
:param G: Goal Point
:param heuristic: Heuristic calculation method
:param longReturn: Flag to determine if entire path is returned or only T/F solvable
:return: List of coords with path to goal if possible OR T/F if path exists
"""


def AStarUnknown(maze, S, G, heuristic, longReturn):
    # Initialize open array to represent information known at the time
    knownMaze = Maze(maze.dim, 0)

    # Run A* once with open Grid
    start = (0, S[0], S[1])
    prevStop = (0, 0)
    fringe = []
    hq.heappush(fringe, start)
    gn = [[(np.inf, None) for i in range(maze.dim)] for j in range(maze.dim)]
    gn[S[1]][S[0]] = (0, None)
    helperAStar(knownMaze, G, heuristic, fringe, gn)
    #vis.printArray(gn)
    #print("\n")
    path = createPath(knownMaze, S, G, gn)

    while True:
        if not path:
            break

        stopPoint = checkPath(path, knownMaze, maze, gn)
        if stopPoint == G:
            break
        else:
            # Rerun A* with updated information and new start point
            start = (0, stopPoint[0], stopPoint[1])
            fringe = []
            hq.heappush(fringe, start)
            gn = [[(np.inf, None) for i in range(maze.dim)] for j in range(maze.dim)]
            gn[stopPoint[1]][stopPoint[0]] = (0, (gn[stopPoint[1]][stopPoint[0]][1]))
            prevStop = stopPoint
            helperAStar(knownMaze, G, heuristic, fringe, gn)
            #vis.printArray(gn)
            #print("\n")
            path = createPath(knownMaze, stopPoint, G, gn)

    # Construct actual string path from result of Astar search if requested
    if longReturn:
        global trajectoryUnknown
        #print("Trajectory is {}".format(trajectoryUnknown))

        global cellsProcessed
        #print("Cells Processed Number is {}".format(cellsProcessed))

        start = (0, S[0], S[1])
        fringe = []
        hq.heappush(fringe, start)
        gn = [[(np.inf, None) for i in range(maze.dim)] for j in range(maze.dim)]
        gn[S[1]][S[0]] = (0, None)
        helperAStar(knownMaze, G, heuristic, fringe, gn)
        path = createPath(knownMaze, S, G, gn)
        #print(path)
        return path, trajectoryUnknown, cellsProcessed

    # Return T/F is goal is reachable
    if gn[G[1]][G[0]][0] == np.inf:
        return False
    return True


def checkPath(path, knownMaze, maze, gn):
    if not path:
        return False

    # iterate through path adding information when necessary
    for i in range(len(path)):
        global trajectoryUnknown
        trajectoryUnknown += 1

        if maze.maze[path[i][1]][path[i][0]] == 1:  # if blockage found, update knownMaze and stop path
            knownMaze.maze[path[i][1]][path[i][0]] = 1
            return path[i - 1]


        else:  # if no blockage found, update knownMaze with neighbors info
        # Comment out else statement to do Q7 and swap trajectory to up in the loop
            for direction in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
                newxc = direction[0] + path[i][0]
                newyc = direction[1] + path[i][1]
                if newxc >= maze.dim or newxc < 0 or newyc >= maze.dim or newyc < 0:  # Legal within bounds
                    continue
                else:
                    knownMaze.maze[newyc][newxc] = maze.maze[newyc][newxc]

    # If finished path, means path from Start to Goal is found and return True
    return path[-1]


"""
createPath - Create path given the matrix with reversible path from G
:param maze: Grid to solve with blocked/unblocked cells
:param S: Start Point
:param G: Goal Point
:param gn: Matrix containing distances of all cells and parents
:return: the path given as a list
"""


def createPath(maze, S, G, gn):
    # two arrays, one to store the directions in reverse, and one to give the actual directions
    path = [(maze.dim - 1, maze.dim - 1)]

    # stores the current coordinates
    xCoord = G[0]
    yCoord = G[1]

    # stores the coordinates of start
    xStart = S[0]
    yStart = S[1]

    # check if goal is reachable
    if gn[G[1]][G[0]][0] == np.inf:
        return []

    # assemble coordinates of path from goal to start before reversing it
    while not (xCoord == xStart and yCoord == yStart):
        # Retrieves parents coords
        nextMove = gn[yCoord][xCoord][1]
        #print(nextMove)

        if nextMove is None:
            break

        # calculates the next coordinate
        xCoord = nextMove[0]
        yCoord = nextMove[1]

        # append to path
        path.append((xCoord, yCoord))

    return list(reversed(path))


"""
isReachableDFS - Checks if G is reachable from S using DFS
:param S: tuple representing (S)tarting square
:param G: tuple representing (G)oal square
:return: boolean stating whether G is reachable from S
"""


def DFSSolve(m, S, G):
    visited = []
    stack = [S]

    while len(stack):
        v = stack.pop()
        if v not in visited:
            visited.append(v)
            if v == G:
                return True
            stack.extend(m.findNeighbors(v))
    return False


"""
    isReachableDFS - Checks if G is reachable from S using DFS
    :param S: tuple representing (S)tarting square
    :param G: tuple representing (G)oal square
    :return: boolean stating whether G is reachable from S
    """


def BFSSolve(m, S, G):
    visited = []
    queue = [S]

    while len(queue):
        v = queue.pop(0)
        if v not in visited:
            visited.append(v)
            if v == G:
                return True
            queue.extend(m.findNeighbors(v))
    return False
