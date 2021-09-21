from maze import Maze

"""
plotMaze - output maze in terminal

:TODO: Feel free to explore the matplot library for better quality plots
"""
def plotMaze(maze: Maze):
    for y in range(maze.dim):
        lineString = ""
        for x in range(maze.dim):
            lineString = lineString + " " + str(maze.maze[y][x])
        print(lineString)
    print()

def printArray(array):
    for y in range(len(array)):
        lineString = ""
        for x in range(len(array[0])):
            lineString = lineString + "\t" + str(array[y][x])
        print(lineString)