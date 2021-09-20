# Main file to test our code
import time
import Astar as ast
from maze import Maze


def main():

    # Demonstrate maze solving works
    m = Maze(5, 0.3)
    m.plotMaze()
    for coord in ast.AStar(m, (0, 0), (4, 4), ast.mDistance, True):
        m.set(coord, 'X')
    m.plotMaze()

    # Q4

    timeA = 0
    timeB = 0
    timeD = 0

    t = 0
    f = 0

    for i in range(100):
        m = Maze(101, 0.3)

        start = time.time()
        ast.DFSSolve(m, (0, 0), (100, 100))
        end = time.time()
        timeD += (end - start)

        start = time.time()
        ast.BFSSolve(m, (0, 0), (100, 100))
        end = time.time()
        timeB += (end - start)

        start = time.time()
        res = ast.AStar(m, (0, 0), (100, 100), ast.eDistance, False)
        end = time.time()
        timeA += (end - start)

        if res:
            t += 1
        else:
            f += 1

    print(timeD / 100)
    print(timeB / 100)
    print(timeA / 100)
    print(t)
    print(f)

    # Q5

    timeE = 0
    timeM = 0
    timeC = 0

    for i in range(100):
        m = Maze(101, 0.3)

        start = time.time()
        ast.AStar(m, (0, 0), (100, 100), ast.eDistance, False)
        end = time.time()
        timeE += (end - start)

        start = time.time()
        ast.AStar(m, (0, 0), (100, 100), ast.mDistance, False)
        end = time.time()
        timeM += (end - start)

        start = time.time()
        ast.AStar(m, (0, 0), (100, 100), ast.cDistance, False)
        end = time.time()
        timeC += (end - start)

    print(timeE / 100)
    print(timeM / 100)
    print(timeC / 100)


if __name__ == "__main__":
    main()
