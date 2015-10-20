# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST

    if problem.isGoalState(problem.getStartState()):
        return []

    currentNode = (problem.getStartState(),0,0)
    visited = []
    directions = {}
    parents = {}
    frontier = util.Stack()
    shouldContinue = 1

    while (shouldContinue == 1):
        current, parent, direction = currentNode
        if (current not in visited):
            visited.append(current)
            parents[current] = parent
            directions[current] = direction
            if (problem.isGoalState(current)):
                shouldContinue = 0
            else:
                for child,move,cost in problem.getSuccessors(current):                       
                    frontier.push((child, current, move))

        if not (frontier.isEmpty()):
            currentNode = frontier.pop()

        if problem.isGoalState(current):
            shouldContinue = 0

    shouldContinue = 1
    # Find direction to child from parent
    directionToGoal = []

    child = current
    parent = parents[child]

    while (shouldContinue):

        directionToGoal.insert(0,directions[child])
        child = parent
        # print parents[child]
        if (child == problem.getStartState()):
            shouldContinue = 0
        else:
            parent = parents[child]
    return directionToGoal


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST


    # while frontier is not empty
    #     pop Node
    #     append node.move to path
    #     stop if isGoalState
    
    start = problem.getStartState()
    currentNode = Node(start,0,0)

    if isinstance(problem.getStartState()[0], tuple):
        cornersProblem = True

    else:
        cornersProblem = False

    visited = []

    frontier = util.Queue()
    shouldContinue = True
    start = problem.getStartState()

    if problem.isGoalState(problem.getStartState()):
        return []
    total = 0

    while (shouldContinue):
        # print "--------------------------------------------------------------------------------"
        total += 1
        if (currentNode.state not in visited):
            visited.append(currentNode.state)
            if (problem.isGoalState(currentNode.state)):
                shouldContinue = False
            else:
                # print "currNode.state: ", currentNode.state.loc
                for state,move,cost in problem.getSuccessors(currentNode.state):
                    # if cornersProblem:
                    newNode = Node(state, move, currentNode) 
                    # else:
                        # newNode = Node(state, move, currentNode)   
                    frontier.push(newNode)

        if not (frontier.isEmpty()):
            currentNode = frontier.pop()
            
        if problem.isGoalState(currentNode.state):# or total > 25:
            # print "test"
            shouldContinue = False
    # print "while done"

    return pathHelper(currentNode, start)
    

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST

    start = problem.getStartState()
    currentNode = Node(start,0,0)

    if isinstance(problem.getStartState()[0], tuple):
        cornersProblem = True

    else:
        cornersProblem = False

    visited = []

    frontier = util.PriorityQueue()
    shouldContinue = True
    start = problem.getStartState()

    if problem.isGoalState(problem.getStartState()):
        return []
    total = 0

    while (shouldContinue):
        total += 1
        if (currentNode.state not in visited):
            visited.append(currentNode.state)
            if (problem.isGoalState(currentNode.state)):
                shouldContinue = False
            else:

                for state,move,cost in problem.getSuccessors(currentNode.state):

                    newNode = Node(state, move, currentNode) 
                    cost = cost + problem.getCostOfActions(pathHelper(newNode, start))
                    frontier.push(newNode, cost)

        if not (frontier.isEmpty()):
            currentNode = frontier.pop()
            
        if problem.isGoalState(currentNode.state):

            shouldContinue = False


    return pathHelper(currentNode, start)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def pathHelper(current, start):

    directionToGoal = []
    shouldContinue = True

    while (current.state != start):
        directionToGoal.insert(0,current.actionTaken)
        current = current.parentNode

    return directionToGoal

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    n = Directions.NORTH
    e = Directions.EAST

    start = problem.getStartState()
    currentNode = Node(start,0,0)
    if isinstance(problem.getStartState()[0], tuple):
        cornersProblem = True

    else:
        cornersProblem = False

    visited = []
    frontier = util.PriorityQueue()
    frontierList = []
    parents = {}
    directions = {}
    g_score = {}
    f_score = {}

    g_score[start] = 0
    f_score[start] = g_score[start] + heuristic(start, problem)
    frontier.push(currentNode, f_score[start])
    improvedCost = False

    while not frontier.isEmpty():
        currentNode = frontier.pop()
        if currentNode.state in frontierList:
            frontierList.remove(currentNode.state)
        # print current
        if (problem.isGoalState(currentNode.state)):
            return pathHelper(currentNode, start)#problem, parents, directions, current, start)
        if currentNode not in visited:
            visited.append(currentNode)
            for neighbor, move, g_cost in problem.getSuccessors(currentNode.state):
                if neighbor not in visited:
                    tentative_g_score = g_cost + problem.getCostOfActions(pathHelper(currentNode, start))+ heuristic(neighbor, problem)

                    if (neighbor not in g_score) or (tentative_g_score < g_score[neighbor]):

                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, problem)
                        newNode = Node(neighbor, move, currentNode)

                        # frontierList.append(neighbor)

                        frontier.push(newNode, f_score[neighbor])

    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

class cornerState():

    def __init__(self, loc, corners):
        self.loc = loc
        self.corners = corners

    def __eq__(self, obj):
        if not isinstance(obj, cornerState):
            return False
        obj.corners.sort()
        self.corners.sort()
        return obj.loc == self.loc and obj.corners == self.corners

    def __ne__(self, obj):
        return not self == obj

    def __hash__(self):
        return hash(self.loc) + hash((self.corners[-1] if len(self.corners)> 0 else 1))

    def __getitem__(self, index):
        if index == 0:
            return self.loc
        else:
            return self.corners


class searchState():

    def __init__(self, loc):
        self.loc = loc

    def __eq__(self, obj):
        if not isinstance(obj, searchState):
            return False
        return obj.loc == self.loc

    def __ne__(self, obj):
        return not self == obj

    def __hash__(self):
        return hash(self.loc)

class Node():

    def __init__(self, state, actionTaken, parentNode):
        self.state = state
        self.actionTaken = actionTaken
        self.parentNode = parentNode

    def __eq__(self, obj):
        if not isinstance(obj, Node):
            return False
        return obj.state == self.state and self.actionTaken == obj.actionTaken and self.parentNode == obj.parentNode

    def __ne__(self, obj):
        return not self == obj

    def __hash__(self):
        return hash(self.state) + hash(self.actionTaken) + hash(self.parentNode)
