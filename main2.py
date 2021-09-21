# Main file to test our code
import time
import Astar as ast
from maze import Maze


def main():
    totalTraj = 0
    totalCells = 0
    totalFinal = 0
    totalFull = 0
    
    iterations = 10

    for i in range(iterations):
        m = Maze(101, 0.00)

        resK = ast.AStarKnown(m, (0, 0), (100, 100), ast.mDistance, True)
        resU, trajectory, cells = ast.AStarUnknown(m, (0, 0), (100, 100), ast.mDistance, True)

        totalTraj += trajectory
        totalCells += cells
        totalFinal += len(resU)
        totalFull += len(resK)

    print(totalTraj/iterations)
    print(totalCells/iterations)
    print(totalFinal/iterations)
    print(totalFull/iterations)


if __name__ == "__main__":
    main()
