# A* algorithm along with helper methods

import numpy as np
import heapq as hq
import helper as hp

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
    h = abs(xdist) + abs(ydist)  # calculates heuristic
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
AStar - Performs A* search on maze with given parameters
:param maze: Grid to solve with blocked/unblocked cells
:param S: Start Point
:param G: Goal Point
:param heuristic: Heuristic calculation method
:param longReturn: Flag to determine if entire path is returned or only T/F solvable
:return: List of coords with path to goal if possible OR T/F if path exists
"""


def AStar(maze, S, G, heuristic, longReturn):
    start = (0, S[0], S[1])  # store each coordinate as (f(n),x,y)
    fringe = []
    hq.heappush(fringe, start)  # create priority queue with the start as the only element in the fringe

    # Initialize array to keep track of f(n) per cell in grid and parents
    # Matrix allows for constant time access to all necessary information
    dim = maze.dim
    fn = [[(np.inf, None) for i in range(dim)] for j in range(dim)]
    fn[S[1]][S[0]] = (0, None)

    # Iteratively find best path
    helperAStar(maze, G, heuristic, fringe, fn)

    # Construct actual string path from result of Astar search if requested
    if longReturn:
        path = createPath(maze, S, G, fn)
        return path

    # Return T/F is goal is reachable
    if fn[G[0]][G[1]][0] == np.inf:
        return False
    return True


"""
helperAStar - A* Child Generation and Path Checking
:param maze: Grid to solve with blocked/unblocked cells
:param G: Goal Point
:param heuristic: Heuristic calculation method
:param fringe: Priority Queue of different cells
:param fn: Matrix containing distances of all cells and parents
"""


def helperAStar(maze, G, heuristic, fringe, fn):
    if not fringe:  # check for empty fringe meaning no remaining paths to check
        return

    while fringe:
        x = hq.heappop(fringe)  # pop first element and check if goal otherwise create children and call again
        if x[1] == G[0] and x[2] == G[1]:
            return
        createChildren(x, maze, G, heuristic, fringe, fn)
    return


"""
createChildren - Generates children of given point and stores in (fn,x,y) form in fringe when needed 
:param x: Point to generate children
:param maze: Grid to solve with blocked/unblocked cells
:param G: Goal Point
:param heuristic: Heuristic calculation method
:param fringe: Priority Queue of different cells
:param fn: Matrix containing distances of all cells and parents
"""


def createChildren(x, maze, G, heuristic, fringe, fn):
    xc = x[1]
    yc = x[2]
    newfn = fn[yc][xc][0] + 1  # note that python matrix storage is not normal x,y graph coordinates

    # generate legal children for all 4 possible moves based on origin point
    for direction in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
        newxc = direction[0] + xc
        newyc = direction[1] + yc

        if newxc >= maze.dim or newxc < 0 or newyc >= maze.dim or newyc < 0:  # Legal within bounds children only!
            continue

        oldfn = fn[newyc][newxc][0]
        if newfn < oldfn and maze.maze[newyc][newxc] == 0:  # add only if improved distance and open
            fn[newyc][newxc] = (newfn, hp.rev(direction))  # store new fn and reversed direction to parent
            item = (heuristic((newxc, newyc), G, fn), newxc, newyc)  # store value based on heuristic and x,y coords
            hq.heappush(fringe, item)
    return


"""
createPath - Create path given the matrix with reversible path from G
:param maze: Grid to solve with blocked/unblocked cells
:param S: Start Point
:param G: Goal Point
:param fn: Matrix containing distances of all cells and parents
:return: the path given as a list
"""


def createPath(maze, S, G, fn):
    # two arrays, one to store the directions in reverse, and one to give the actual directions
    path = [(maze.dim - 1, maze.dim - 1)]

    # stores the current coordinates
    xCoord = G[0]
    yCoord = G[1]

    # stores the coordinates of start
    xStart = S[0]
    yStart = S[1]

    # check if goal is reachable
    if fn[yCoord][xCoord][0] == np.inf:
        return []

    # assemble coordinates of path from goal to start before reversing it
    while not (xCoord == xStart and yCoord == yStart):
        # direction of previous node
        nextMove = fn[yCoord][xCoord][1]

        # calculates the next coordinate
        xCoord = xCoord + nextMove[0]
        yCoord = yCoord + nextMove[1]

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
