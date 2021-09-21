# Maze class to generate grid and relevant helper functions
import numpy as np


class Maze:
    """
    init - Create maze where 0 is open and 1 is closed
    :param dim: Dimensions
    :param p: Blocked Probability
    """

    def __init__(self, dim, p):
        self.dim = dim
        self.p = p
        # Create dim x dim maze where values are either 0 or 1 with chance 1-p and p respectively
        self.maze = [[int(np.random.choice(2, 1, p=[1 - p, p])) for i in range(dim)] for j in range(dim)]
        self.maze[0][0] = 0
        self.maze[dim - 1][dim - 1] = 0

    """
    plotMaze - Prints maze in terminal
    """

    def plotMaze(self):
        for y in range(self.dim):
            line = ""
            for x in range(self.dim):
                line = line + " " + str(self.maze[y][x])
            print(line)
        print()

    """
    findNeighbors - Determines all valid neighbors for passed in Loc
    :param loc: Location to find neighbors of
    :return: List of valid neighbors
    """

    def findNeighbors(self, loc):
        neighbors = []
        for move in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
            neighbor = (loc[0] + move[0], loc[1] + move[1])
            if 0 < neighbor[0] < self.dim and 0 < neighbor[1] < self.dim and self.maze[neighbor[1]][neighbor[0]] == 0:
                neighbors.append(neighbor)
        return neighbors
