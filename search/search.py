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
    moves = [] # move to goal from start state
    closedList = [] # All explored/expanded nodes
    #isInFringe = {} # All nodes explored and being considered, with state of being in fringe currently or not

    fringe = util.Stack()
    fringe.push( (problem.getStartState(), moves) ) # Let fringe not only hold nodes to explore, but path/moves to that node from startState
    while(not fringe.isEmpty()):

        node, currMoves = fringe.pop()
        #isInFringe[node] = 0

        if(problem.isGoalState(node)):
            moves = currMoves
            break

        # Update the fringe
        # make sure the node is not already in the closedList 
        # AND as per specs of this problem: ensure node not currently being concidered in Fringe
        # ie. don't readd to a node to fringe if in closedList or already in fringe
        elif( node not in closedList ):
            successors = problem.getSuccessors(node)
            closedList.append(node) # add the node to closed list on getting its fringe

            for s in successors:
                nd = s[0] # successor node
                mv = s[1] # move to the successor node from current node
                cst = s[2] # cost to successor node

                # if ((nd not in isInFringe.keys()) and  (nd not in closedList)):
                # This condition is not required. We can add the node again to fringe (ie to the top of the fringe)
                # This would mean, this latest path through which a node is being visited will be at the top of the
                # fringe/stack. The older entry for the node (if any will also be there in the fringe). Which of these
                # two entries gets expanded will depend on the search algo. If it is DFS, the latest value will get
                # expanded. If it is BFS the older value will get expanded. 
                # HOWEVER, this still is uses a valid "graph search" technique of visiting a node only once, since 
                # the elif (node not in closedList) condition above will prevent any action from being taken if the 
                # node is revisited
                fringe.push( (nd, currMoves+[mv]) )
                #isInFringe[nd] = 1

    return moves

def singleGoalBFS(problem):
    """
    While using autograder, the "problem" variable does not have access to the goal state. It can only check
    if a given node is the goal state. So, can not provide goal to the parentMap to get path to goal 
    by retracing steps. Instead, need to have a different way of keep track of path to goal. Should make use of
    getSuccessors() (and the direction that it returns as one of the args) 
    """
    moves = [] # move to goal from start state
    closedList = [] # All explored/expanded nodes

    fringe = util.Queue()
    fringe.push( (problem.getStartState(), moves) ) # Let fringe not only hold nodes to explore, but path/moves to that node from startState
    while(not fringe.isEmpty()):

        node, currMoves = fringe.pop()

        if(problem.isGoalState(node)):
            moves = currMoves
            break

        # Update the fringe
        # make sure the node is not already in the closedList 
        # AND as per specs of this problem: ensure node not currently being concidered in Fringe
        # ie. don't readd to a node to fringe if in closedList or already in fringe
        elif( node not in closedList ):
            successors = problem.getSuccessors(node)
            closedList.append(node) # add the node to closed list on getting its fringe

            for s in successors:
                nd = s[0] # successor node
                mv = s[1] # move to the successor node from current node
                cst = s[2] # cost to successor node

                # if ((nd not in isInFringe.keys()) and  (nd not in closedList)):
                # Condition not needed. Explanation in DFS algo
                fringe.push( (nd, currMoves+[mv]) )

    return moves

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    # Deleted multiStageBFS
    # Can not track each corner sequentially by making each corner reached as new start state. 
    # This would be equivalent to saying find first corner and from there start finding next closest corner till all corners found
    # Such logic would not lead to shortest path through all four corners. Seen easily in tinyCorners, 
    # that it would follow a different route than that which we would use if we were to manually control the agent
    # SO, need to allow all paths to be explored continuously/in parallel, and stopping when one of them hits all four corners
    # ie. keep letting multiple paths explore the maze in parallel till one of them reports via isGoalState() that it hit all four corners
    # The first to report such an event would be the shortest path through all corners.

    # So, 
    # 1. need to change isGoalState to track number of corners each path has visited so far
    # 2. AKA. need to encode corners visited into the "node state"/each path 
    # 3. Need to allow retracing of steps at deadends...ie. change closedList condition?
    # Now since nodes will be represented with two states (coordinate, cornersVisited) = ((x,y), [cornersVisited])
    # Once the path goes into a deadend, if that deadend is a "useful deadend" (ie. one with food/corner of desire), then there is a good 
    # reason for the pacman to return/retrace its steps. And this will be allowed by the closedList since on reaching the corner
    # node state will change and include a corner in the cornersVisited list. Thus that specific changed form of the node will
    # not exist in the closedList and the successor nodes to retrace the deadend path if the deadend led to a corner will be allowed/
    # will pass the "node not in closedList" test

    return singleGoalBFS(problem)


def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """
    moves = [] # move to goal from start state
    closedList = [] # All explored/expanded nodes

    fringe = util.PriorityQueue()
    fringe.push( (problem.getStartState(), moves), 0 ) # Let fringe not only hold nodes to explore, but path/moves to that node from startState
    while(not fringe.isEmpty()):

        node, currMoves = fringe.pop()

        if(problem.isGoalState(node)):
            moves = currMoves
            break

        # Update the fringe
        # make sure the node is not already in the closedList 
        # AND as per specs of this problem: ensure node not currently being concidered in Fringe
        # ie. don't readd to a node to fringe if in closedList or already in fringe
        elif( node not in closedList ):
            successors = problem.getSuccessors(node)
            closedList.append(node) # add the node to closed list on getting its fringe

            for s in successors:
                nd = s[0] # successor node
                mv = s[1] # move to the successor node from current node
                #cst = s[2] # cost of successor node from current location. Not needed since cost from start till sucessor is full path cost that UCS/Astar uses

                updatedMoves = currMoves + [mv]
                cst = problem.getCostOfActions(updatedMoves) # cost from start till successor/potential next node
                fringe.push( (nd, updatedMoves), cst )

    return moves

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    moves = [] # move to goal from start state
    closedList = [] # All explored/expanded nodes

    fringe = util.PriorityQueue()
    fringe.push( (problem.getStartState(), moves), 0 ) # Let fringe not only hold nodes to explore, but path/moves to that node from startState
    while(not fringe.isEmpty()):

        node, currMoves = fringe.pop()

        if(problem.isGoalState(node)):
            moves = currMoves
            break

        # Update the fringe
        # make sure the node is not already in the closedList 
        # AND as per specs of this problem: ensure node not currently being concidered in Fringe
        # ie. don't readd to a node to fringe if in closedList or already in fringe
        elif( node not in closedList ):
            successors = problem.getSuccessors(node)
            closedList.append(node) # add the node to closed list on getting its fringe

            for s in successors:
                nd = s[0] # successor node
                mv = s[1] # move to the successor node from current node
                #cst = s[2] # cost of successor node from current location. Not needed since cost from start till sucessor is full path cost that UCS/Astar uses

                updatedMoves = currMoves + [mv]
                cst = problem.getCostOfActions(updatedMoves) # cost from start till successor/potential next node
                heu = heuristic(nd, problem) # heuristic of curr node (to goal ofc.)
                fringe.push( (nd, updatedMoves), cst+heu )

    return moves


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch