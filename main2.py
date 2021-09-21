# Main file to test our code
import time
import Astar as ast
from maze import Maze


def main():
    totalTraj = 0
    totalCells = 0
    totalFinal = 0
    totalFull = 0
    
    iterations = 100 # desired number of mazes to generate
    density = 0.0 # likliehood of a cell being blocked

    for i in range(iterations):
        m = Maze(101, density)
        
        solvable = ast.DFSSolve(m, (0,0), (100,100)) # use DFS to see if solution exists
        while ~(solvable): # if not solvable regenerate till it is
            m = Maze(101,density)
            if(ast.DFSSolve(m, (0,0), (100,100))): # break when solvable
                break

        resK = ast.AStarKnown(m, (0, 0), (100, 100), ast.mDistance, True)   
        resU, trajectory, cells = ast.AStarUnknown(m, (0, 0), (100, 100), ast.mDistance, True)

        totalTraj += trajectory
        totalCells += cells
        totalFinal += len(resU)
        totalFull += len(resK)
        
        print(i)

    print(totalTraj/iterations)
    print(totalCells/iterations)
    print(totalFinal/iterations)
    print(totalFull/iterations)


if __name__ == "__main__":
    main()
