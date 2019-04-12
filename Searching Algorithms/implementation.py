#Selen Coşkun / 220201035
import random

def getPossibleFrontiers(x,y): #finds the possible moves from a given point (x,y) and add them in an array
            #My priority is --> Right - Left - Down - Up
            if ((x == 0 and y == 0) or (x == 0 and y == width-1) or (x == height-1 and y == 0) or (x == height-1 and y == width-1)):

                if (x == 0 and y == 0):
                    if(height == 1):
                        possibleFrontierArr.append([x,y+1])
                    elif(width == 1):
                        possibleFrontierArr.append([x+1, y])
                    else:
                        possibleFrontierArr.append([x, y + 1])
                        possibleFrontierArr.append([x+1, y])
                elif(x == 0 and y == width-1):
                    if (height == 1):
                        possibleFrontierArr.append([x, y - 1])
                    else:
                        possibleFrontierArr.append([x, y - 1])
                        possibleFrontierArr.append([x+1, y ])
                elif(x == height-1 and y == 0):
                    if (width == 1):
                        possibleFrontierArr.append([x-1, y])
                    else:
                        possibleFrontierArr.append([x, y + 1])
                        possibleFrontierArr.append([x-1, y])

                elif(x == height-1 and y == width-1):
                    possibleFrontierArr.append([x, y - 1])
                    possibleFrontierArr.append([x-1, y ])

            elif ((x == 0) or (x == height-1) or (y == 0) or (y == width-1)):

                            if(x== 0):
                                if(height == 1):
                                    possibleFrontierArr.append([x, y + 1])
                                    possibleFrontierArr.append([x, y - 1])
                                else:
                                    possibleFrontierArr.append([x, y + 1])
                                    possibleFrontierArr.append([x, y - 1])
                                    possibleFrontierArr.append([x+1, y ])

                            elif(x == height-1):
                                possibleFrontierArr.append([x, y + 1])
                                possibleFrontierArr.append([x, y - 1])
                                possibleFrontierArr.append([x-1, y])

                            elif(y == 0):
                                if(width == 1):
                                    possibleFrontierArr.append([x+1, y])
                                    possibleFrontierArr.append([x-1, y ])
                                else:
                                    possibleFrontierArr.append([x, y + 1])
                                    possibleFrontierArr.append([x + 1, y])
                                    possibleFrontierArr.append([x-1, y])

                            elif(y == width-1):
                                possibleFrontierArr.append([x, y - 1])
                                possibleFrontierArr.append([x+1, y])
                                possibleFrontierArr.append([x-1, y])

            else:
                possibleFrontierArr.append([x, y + 1])
                possibleFrontierArr.append([x, y - 1])
                possibleFrontierArr.append([x + 1, y])
                possibleFrontierArr.append([x-1, y ])

def printPath(parentChildArr): #finds the path from parent-child array and prints it

    path = []
    goal= [goalPointX,goalPointY]
    indexOfGoalPoint = 0

    for i in range(len(parentChildArr)):
            if goal in parentChildArr[i]:
                indexOfGoalPoint = i
                break

    indexOfparent = indexOfGoalPoint
    length = len(parentChildArr)
    parent = parentChildArr[indexOfparent][0]

    while(parent != [startingPointX,startingPointY]):
        parent = parentChildArr[indexOfparent][0]
        path.append(parent)
        for j in range(length):
            if parent == parentChildArr[j][1]:
                indexOfparent = j
                break

    path.reverse()
    print("Path: ",path)

def findStartingPoint(maze): #finds coordinate of starting point

    global startingPointX,startingPointY #in order to change the value of the global variable
    for i in range(height):
        for j in range(width):
            if maze[i][j] == "S":
                startingPointX = i
                startingPointY = j
                break

def findGoalPoint(maze): #finds coordinate of goal point

    global goalPointX, goalPointY #in order to change the value of the global variable
    for i in range(height):
        for j in range(width):
            if maze[i][j] == "G":
                goalPointX = i
                goalPointY = j
                break

def eliminateObstacleFromArr(possibleFrontierArr,explored): #removes the walls and explored coordinates from the frontiter candidate array

    copyArr = possibleFrontierArr[:] #copies the array so that changing one of both does not change the other.
    for i in range(len(possibleFrontierArr)):
        x = copyArr[i][0]
        y = copyArr[i][1]
        if(maze[x][y] == "-"): #removes walls
            possibleFrontierArr.remove([x,y])

    copyArr2 = possibleFrontierArr[:]
    for i in range(len(copyArr2)):
        if(copyArr2[i] in explored): #removes explored coordinates
            possibleFrontierArr.remove(copyArr2[i])

def addFrontiersToArr(possibleFrontierArr,frontier): #adds the frontier candidates in frontier array

    for i in range(len(possibleFrontierArr)):
        if(possibleFrontierArr[i] not in frontier): #do not add frontier candidates if they are already in frontier list
            frontier.append(possibleFrontierArr[i])

def checkParent(possibleFrontier,parentChildArr):

    for i in range(len(parentChildArr)):
        if possibleFrontier in parentChildArr[i]: #if a frontier's parent has been already determined
            return 0
    return 1

def breathFirstSearch(arr):

    frontier = []
    explored = []
    parentChildArr = []
    frontier.append([startingPointX,startingPointY]) #initially the frontier is starting point

    while(not [goalPointX,goalPointY] in frontier):

        getPossibleFrontiers(frontier[0][0], frontier[0][1]) #determines the possible moves from the first element in frontier array
        eliminateObstacleFromArr(possibleFrontierArr,explored) #edits the possible frontier array(ie. remove walls)
        addFrontiersToArr(possibleFrontierArr,frontier) #adds the possibleFrontier array into frontier array
        explored.append(frontier[0]) #explore the first element in frontier array

        for i in range(len(possibleFrontierArr)): #determines the parent and child in each step and stores them in parentChildArr
            if(checkParent(possibleFrontierArr[i],parentChildArr)):
                parentChildArr.append([explored[len(explored)-1],possibleFrontierArr[i]])

        possibleFrontierArr.clear() #clears the possibleFrontierArray for next iteration
        frontier.pop(0) #frontier array is a queue (FIFO)

    printPath(parentChildArr) #finally, find path

def depthFirstSearch(arr):

    frontier = []
    explored = []
    parentChildArr = []
    frontier.append([startingPointX, startingPointY]) #initially the frontier is starting point

    while(not [goalPointX,goalPointY] in frontier):

        getPossibleFrontiers(frontier[(len(frontier)-1)][0], frontier[(len(frontier)-1)][1]) #find possible frontiers of coordinate that will be explored
        explored.append(frontier[(len(frontier) - 1)]) #explores the last item in frontier array

        for i in range(len(possibleFrontierArr)): #determines the parent and child in each step and stores them in parentChildArr
            if(checkParent(possibleFrontierArr[i],parentChildArr)):
                parentChildArr.append([explored[len(explored)-1],possibleFrontierArr[i]])

        frontier.pop(len(frontier) - 1) #frontier array is a stack (LIFO)
        eliminateObstacleFromArr(possibleFrontierArr,explored) #edits the possible frontier array(ie. remove walls)
        addFrontiersToArr(possibleFrontierArr,frontier) #adds the possibleFrontier array into frontier array
        possibleFrontierArr.clear() #clears the possibleFrontierArray for next iteration

    printPath(parentChildArr)

def setCosts(maze,choice,uniformCostArr): #fills the uniformCostArr with coordinates and their costs

    #uniformCostArray is like -> [[x1,y1],Cost1,[x2,y2],Cost2,[x3,y3],Cost3,...]

    if(choice == "withExtraPoints"):

        for i in range(len(maze)):
                for j in range(len(maze[0])):
                    uniformCostArr.append([i, j])
                    if(maze[i][j]=="S" or maze[i][j]=="-"):
                        uniformCostArr.append(0)
                    elif(maze[i][j] == "G"):
                        uniformCostArr.append(goalPointCost+1)
                    else:
                        uniformCostArr.append(maze[i][j]+1)

    elif(choice == "withoutExtraPoints"):

        for i in range(len(maze)):
            for j in range(len(maze[0])):
                uniformCostArr.append([i, j])
                if (maze[i][j] == "S" or maze[i][j] == "-"):
                    uniformCostArr.append(0)
                else:
                    uniformCostArr.append(1)

def compareCost(arr,explored,choice,uniformCostArr,parentChildArr,calculatedFrontiers):

    costArray = [] #in order to compare the costs of frontiers

    if(choice == "withoutExtraPoints"):

        for i in range(len(arr)):
            index = uniformCostArr.index(arr[i])
            if (arr[i] not in calculatedFrontiers): #if a frontier cost has already been calculated, there is no need to calculate it again except for one situation
                calculatedFrontiers.append(arr[i])
                if (maze[arr[i][0]][arr[i][1]] == "S" or maze[arr[i][0]][arr[i][1]] == "-"):
                    cost = 0
                else:
                    cost = 1
                uniformCostArr[index + 1] = uniformCostArr[uniformCostArr.index(explored[len(explored) - 1]) + 1] + cost #adds the previous path costs and current frontier cost
                costArray.append(uniformCostArr[index + 1])

            else:
                cost = uniformCostArr[uniformCostArr.index(arr[i]) + 1] #if a frontier cost has already been calculated, just add its calculated cost in cost array
                costArray.append(cost)

    elif(choice == "withExtraPoints"):

        for i in range(len(arr)):
            index = uniformCostArr.index(arr[i])
            if (arr[i] not in calculatedFrontiers):
                calculatedFrontiers.append(arr[i])
                if (maze[arr[i][0]][arr[i][1]] == "S" or maze[arr[i][0]][arr[i][1]] == "-"):
                    cost = 0
                elif (maze[arr[i][0]][arr[i][1]] == "G"):
                    cost = goalPointCost + 1
                else:
                    cost = maze[arr[i][0]][arr[i][1]] + 1
                uniformCostArr[index + 1] = uniformCostArr[uniformCostArr.index(explored[len(explored) - 1]) + 1] + cost
                costArray.append(uniformCostArr[index + 1])

            else:
                cost = uniformCostArr[uniformCostArr.index(arr[i]) + 1]
                costArray.append(cost)

    minIndex = arr[costArray.index(min(costArray))] #decides which frontier will be explored
    possibleFrontierArr.clear()
    getPossibleFrontiers(minIndex[0], minIndex[1])
    eliminateObstacleFromArr(possibleFrontierArr,explored)

    #Frontier cost may have to be calculated again if new explored coordinate has contain that frontier in its possible frontiers
    #If new cost is smaller than the old cost, the cost of that frontier has to be updated
    #Also the parent of that frontier has to be updated
    #The below code does above operations
    for i in range(len(possibleFrontierArr)):
        if (possibleFrontierArr[i] in arr and calculatedFrontiers):
            if(choice == "withoutExtraPoints"):
                cost = 0
            elif(choice=="withExtraPoints"):
                if (possibleFrontierArr[i] == [goalPointX, goalPointY]):
                    cost = goalPointCost
                else:
                    cost = maze[possibleFrontierArr[i][0]][possibleFrontierArr[i][1]]
            if (uniformCostArr[uniformCostArr.index(possibleFrontierArr[i]) + 1] > (1 + uniformCostArr[uniformCostArr.index(minIndex) + 1] + cost)): #if new cost is smaller than the old cost
                editedIndex = uniformCostArr.index(possibleFrontierArr[i])
                uniformCostArr[editedIndex + 1] = uniformCostArr[uniformCostArr.index(minIndex) + 1] + 1 + cost
                for j in range(len(parentChildArr)): #the parent of that frontier has to be updated
                    if (possibleFrontierArr[i] == parentChildArr[j][1]):
                        parentChildArr[j][0] = minIndex

    return minIndex

def uniformCostSearch(arr,choice):

    frontier = []
    explored = []
    parentChildArr = []
    uniformCostArr = []
    calculatedFrontiers = []
    setCosts(arr,choice,uniformCostArr)
    explored.append([startingPointX,startingPointY]) #we can directly explore the starting point for ease

    while (not [goalPointX,goalPointY] in explored):

        getPossibleFrontiers(explored[len(explored) - 1][0], explored[len(explored) - 1][1])
        eliminateObstacleFromArr(possibleFrontierArr,explored)
        addFrontiersToArr(possibleFrontierArr,frontier)

        for i in range(len(possibleFrontierArr)):
            if (checkParent(possibleFrontierArr[i],parentChildArr)):
                parentChildArr.append([explored[len(explored) - 1], possibleFrontierArr[i]])

        explored.append(compareCost(frontier,explored,choice,uniformCostArr,parentChildArr,calculatedFrontiers))
        frontier.remove(explored[len(explored) - 1])
        possibleFrontierArr.clear()

    printPath(parentChildArr)
    print("Cost of Path:",uniformCostArr[uniformCostArr.index([goalPointX,goalPointY])+1])

def createHeuristic(choice,admissibleHeuristics,inadmissibleHeuristics):

    if(choice == "admissible"):
        #creates an admissible heuristic array according to maze dimensions
        for j in range(height):
            admissibleHeuristics.append([])
            for i in range(width):
                if (goalPointX == j and goalPointY == i):
                    admissibleHeuristics[j].append(0) #goal point has 0 heuristic
                else:
                    admissibleHeuristics[j].append(1) #other points has 1 heuristic

    elif(choice == "inadmissible"):
        # creates an inadmissible heuristic array according to maze dimensions
        for j in range(height):
            inadmissibleHeuristics.append([])
            for i in range(width):
                if (goalPointX == j and goalPointY == i):
                    inadmissibleHeuristics[j].append(0) #goal point has 0 heuristic
                else:
                    inadmissibleHeuristics[j].append(1)  #other points has 1 heuristic except one point
        #creates random x and y indexes
        randomX = random.randrange(0,width)
        randomY = random.randrange(0,height)

        if(goalPointX == randomX and goalPointY == randomY): #in case of random indexes are equal to goal point indexes
            randomX = randomX - 1 #instead of refind random indexes, just substract 1 from x index
        inadmissibleHeuristics[randomX][randomY] = 999999999 #this value makes the heuristic inadmissible

def setHeuristicsAndCosts(maze,choice,admissibleHeuristics,inadmissibleHeuristics,aStarArr):

     # aStarArr is like -> [[x1,y1],Cost1,Heuristic1,[x2,y2],Cost2,Heuristic2,[x3,y3],Cost3,Heuristic3,...]
     # aStarArr -> [[Coordinate],Cost,Heuristic]
     for i in range(len(maze)):
        for j in range(len(maze[0])):
            #Coordinate
            aStarArr.append([i,j])
            #Cost
            if (maze[i][j] == "-"):
                aStarArr.append(0)
            elif(maze[i][j] == "S"):
                aStarArr.append(0)
            elif(maze[i][j] == "G"):
                aStarArr.append(goalPointCost + 1)
            else:
                aStarArr.append(maze[i][j]+1)
            #Heuristic
            if(choice == "admissible"):
                aStarArr.append(admissibleHeuristics[i][j])
            elif(choice == "inadmissible"):
                aStarArr.append(inadmissibleHeuristics[i][j])

def compareEvalutionFunction(arr,explored,parentChildArr,aStarArr,evalWithValues,calculatedFrontiers):

    evalutionFuncArr = [] #in order to compare the evalution functions

    for i in range(len(arr)):
        index = aStarArr.index(arr[i])
        if (arr[i] not in calculatedFrontiers):
            calculatedFrontiers.append(arr[i])

            if (maze[arr[i][0]][arr[i][1]] == "S" or maze[arr[i][0]][arr[i][1]] == "-"):
                cost = 0
            elif (maze[arr[i][0]][arr[i][1]] == "G"):
                cost = goalPointCost + 1
            else:
                cost = maze[arr[i][0]][arr[i][1]] + 1

            aStarArr[index + 1] = aStarArr[aStarArr.index(explored[len(explored) - 1]) + 1] + cost #adds the previous path costs and current frontier cost
            evalutionFuncArr.append(aStarArr[index + 1] + aStarArr[index + 2]) #calculates evalution func and adds it in evalution function array
            evalWithValues.append([aStarArr[index],aStarArr[index + 1] + aStarArr[index + 2]]) #evalWithValues arr is like -> [[[x1,y1],EvalutionFunction1],[[x2,y2],EvalutionFunction2],...]

        # if the frontier evalution func has already been calculated, just its evalution func value has to be found in evalWithValues array and added to evalutionFuncArr for comparison
        else:
            for i in range(len(evalWithValues)):
                if(evalWithValues[i][0] == aStarArr[index]):
                    evalutionFuncArr.append(evalWithValues[i][1])
                    break

    minIndex = arr[evalutionFuncArr.index(min(evalutionFuncArr))] #decides which frontier will be explored
    possibleFrontierArr.clear()
    getPossibleFrontiers(minIndex[0], minIndex[1])
    eliminateObstacleFromArr(possibleFrontierArr,explored)

    # Frontier cost may have to be calculated again if new explored coordinate has contain that frontier in its possible frontiers
    # If new cost is smaller than the old cost, the cost of that frontier has to be updated
    # Also the parent of that frontier has to be updated
    # The below code does above operations
    for i in range(len(possibleFrontierArr)):
        if(possibleFrontierArr[i] in arr and calculatedFrontiers):
            if(possibleFrontierArr[i] == [goalPointX,goalPointY]):
                cost = goalPointCost
            else:
                cost = maze[possibleFrontierArr[i][0]][possibleFrontierArr[i][1]]
            if (aStarArr[aStarArr.index(possibleFrontierArr[i]) + 1] > (1 + aStarArr[aStarArr.index(minIndex) + 1] + cost)):
                editedIndex = aStarArr.index(possibleFrontierArr[i])
                aStarArr[editedIndex + 1] = aStarArr[aStarArr.index(minIndex) + 1] + 1 + cost
                for j in range(len(parentChildArr)):
                    if (possibleFrontierArr[i] == parentChildArr[j][1]):
                        parentChildArr[j][0] = minIndex
    return minIndex

def aStarSearch(arr,choice):

    frontier = []
    explored = []
    parentChildArr = []
    admissibleHeuristics = []
    inadmissibleHeuristics = []
    aStarArr = []
    evalWithValues = []
    calculatedFrontiers = []

    createHeuristic(choice,admissibleHeuristics,inadmissibleHeuristics)
    setHeuristicsAndCosts(arr,choice,admissibleHeuristics,inadmissibleHeuristics,aStarArr)
    explored.append([startingPointX,startingPointY])

    while (not [goalPointX,goalPointY] in explored):

        getPossibleFrontiers(explored[len(explored) - 1][0], explored[len(explored) - 1][1])
        eliminateObstacleFromArr(possibleFrontierArr,explored)
        addFrontiersToArr(possibleFrontierArr,frontier)

        for i in range(len(possibleFrontierArr)):
            if (checkParent(possibleFrontierArr[i],parentChildArr)):
                parentChildArr.append([explored[len(explored) - 1], possibleFrontierArr[i]])

        explored.append(compareEvalutionFunction(frontier, explored,parentChildArr,aStarArr,evalWithValues,calculatedFrontiers))
        frontier.remove(explored[len(explored) - 1])
        possibleFrontierArr.clear()

    printPath(parentChildArr)
    print("Cost of Path is: ",aStarArr[aStarArr.index([goalPointX,goalPointY])+1])

if __name__ == "__main__":

    maze = [["S", 0, 1, 0, 1], ["-", "-", 2, 1, 2], [2, "-", 3, 0, "-"], [0, 2, 1, 1, "-"], [1, 0, "G", 1, "-"]] #please enter maze
    goalPointCost = 3 #please enter the goal point cost

    height = len(maze)
    width = len(maze[0])

    possibleFrontierArr = []

    #initializes goal point and starting point
    goalPointX = -1
    goalPointY = -1

    startingPointX = -1
    startingPointY = -1

    #finds goal point and starting point
    findStartingPoint(maze)
    findGoalPoint(maze)

    print("\nStarting Point:",[startingPointX,startingPointY],"\nGoal:",[goalPointX,goalPointY],"\n")

    print("----Breath First Search----")
    breathFirstSearch(maze)
    print()
    print("----Depth First Search----")
    depthFirstSearch(maze)
    print()
    print("----Uniform Cost Search(Without Extra Points)----")
    uniformCostSearch(maze,"withoutExtraPoints")
    print()
    print("----Uniform Cost Search(With Extra Points)----")
    uniformCostSearch(maze, "withExtraPoints")
    print()
    print("----A* Star Search(Admissible)----")
    aStarSearch(maze, "admissible")
    print()
    print("----A* Star Search(Inadmissible)----")
    aStarSearch(maze, "inadmissible")