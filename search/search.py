import pdb
from game import Directions
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def getDirections(start, dir, parentMap, dst):

    # End condition of recursion
    if(dst==start):
        return dir
    
    src = parentMap[dst]
    xdiff = src[0] - dst[0]
    ydiff = src[1] - dst[1]
    if( xdiff==0 and ydiff ==0 ):
        return dir

    # Main portion of recursion
    if(ydiff>0):
        dir.append(Directions.SOUTH)
        return getDirections(start, dir, parentMap, src)
    if(ydiff<0):
        dir.append(Directions.NORTH)
        return getDirections(start, dir, parentMap, src)
    if(xdiff>0):
        dir.append(Directions.WEST)
        return getDirections(start, dir, parentMap, src)
    if(xdiff<0):
        dir.append(Directions.EAST)
        return getDirections(start, dir, parentMap, src)

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    moves = []
    closedList = []
    isInFringe = {}
    parentMap = {}

    fringe = util.Stack()
    node = problem.getStartState()

    while(1): # any way to write the code so that the exit condition is checked here rather than a while(1) loop?

        #pdb.set_trace()
        if(problem.isGoalState(node)):
            # return moves
            break

        # Update the fringe
        # make sure the node is not already in the closed set
        elif( node not in closedList ):
            #add the node to closed list on getting its fringe
            successors = problem.getSuccessors(node)
            closedList.append(node)

            # associate to parent node
            for s in successors:
                nd = s[0]
                # if (nd not in closedList) and (nd not in fringe): # Only if this is a completely new node that is visited, add it. ELSE may get assigned to the wrong parent
                if ((nd not in isInFringe.keys()) and  (nd not in closedList)):
                    parentMap[nd] = node
                    fringe.push(nd)
                    isInFringe[nd] = 1 # dummy value ... in C++ the condition in the if would be: "isInFringe[nd] > 0" && ...
                else:
                    continue

        if ( fringe.isEmpty() ):
            break
        else:
            node = fringe.pop()
            isInFringe[nd] = 0 # will be removed from fringe, but then added to closedList in the start of the next loop

    moves = getDirections(problem.startState, moves, parentMap, problem.goal)
    moves.reverse()

    return moves

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    moves = []
    closedList = []
    isInFringe = {}
    parentMap = {}

    fringe = util.Queue()
    node = problem.getStartState()

    while(1): # any way to write the code so that the exit condition is checked here rather than a while(1) loop?

        #pdb.set_trace()
        if(problem.isGoalState(node)):
            # return moves
            break

        # Update the fringe
        # make sure the node is not already in the closed set
        elif( node not in closedList ):
            #add the node to closed list on getting its fringe
            successors = problem.getSuccessors(node)
            closedList.append(node)

            # associate to parent node
            for s in successors:
                nd = s[0]
                # if (nd not in closedList) and (nd not in fringe): # Only if this is a completely new node that is visited, add it. ELSE may get assigned to the wrong parent
                if ((nd not in isInFringe.keys()) and  (nd not in closedList)):
                    parentMap[nd] = node
                    fringe.push(nd)
                    isInFringe[nd] = 1 # dummy value ... in C++ the condition in the if would be: "isInFringe[nd] > 0" && ...
                else:
                    continue

        if ( fringe.isEmpty() ):
            break
        else:
            node = fringe.pop()
            isInFringe[nd] = 0 # will be removed from fringe, but then added to closedList in the start of the next loop

    moves = getDirections(problem.startState, moves, parentMap, problem.goal)
    moves.reverse()

    return moves

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    moves = []
    closedList = []
    isInFringe = {}
    parentMap = {}

    fringe = util.PriorityQueue()
    node = problem.getStartState()

    while(1): # any way to write the code so that the exit condition is checked here rather than a while(1) loop?

        #pdb.set_trace()
        if(problem.isGoalState(node)):
            # return moves
            break

        # Update the fringe
        # make sure the node is not already in the closed set
        elif( node not in closedList ):
            #add the node to closed list on getting its fringe
            successors = problem.getSuccessors(node)
            closedList.append(node)

            # associate to parent node
            for s in successors:
                nd = s[0]
                # if (nd not in closedList) and (nd not in fringe): # Only if this is a completely new node that is visited, add it. ELSE may get assigned to the wrong parent
                if ((nd not in isInFringe.keys()) and  (nd not in closedList)):
                    parentMap[nd] = node
                    cost = problem.costFn(nd)
                    fringe.push(nd, cost)
                    isInFringe[nd] = 1 # dummy value ... in C++ the condition in the if would be: "isInFringe[nd] > 0" && ...
                else:
                    continue

        if ( fringe.isEmpty() ):
            break
        else:
            node = fringe.pop()
            isInFringe[nd] = 0 # will be removed from fringe, but then added to closedList in the start of the next loop

    moves = getDirections(problem.startState, moves, parentMap, problem.goal)
    moves.reverse()

    return moves

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    moves = []
    closedList = []
    isInFringe = {}
    parentMap = {}

    fringe = util.PriorityQueue()
    node = problem.getStartState()

    while(1): # any way to write the code so that the exit condition is checked here rather than a while(1) loop?

        #pdb.set_trace()
        if(problem.isGoalState(node)):
            # return moves
            break

        # Update the fringe
        # make sure the node is not already in the closed set
        elif( node not in closedList ):
            #add the node to closed list on getting its fringe
            successors = problem.getSuccessors(node)
            closedList.append(node)

            # associate to parent node
            for s in successors:
                nd = s[0]
                # if (nd not in closedList) and (nd not in fringe): # Only if this is a completely new node that is visited, add it. ELSE may get assigned to the wrong parent
                if ((nd not in isInFringe.keys()) and  (nd not in closedList)):
                    parentMap[nd] = node
                    pathCost = problem.costFn(nd)
                    heurCost = heuristic(nd, problem)
                    fringe.push(nd, pathCost+heurCost)
                    isInFringe[nd] = 1 # dummy value ... in C++ the condition in the if would be: "isInFringe[nd] > 0" && ...
                else:
                    continue

        if ( fringe.isEmpty() ):
            break
        else:
            node = fringe.pop()
            isInFringe[nd] = 0 # will be removed from fringe, but then added to closedList in the start of the next loop

    moves = getDirections(problem.startState, moves, parentMap, problem.goal)
    moves.reverse()

    return moves


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
