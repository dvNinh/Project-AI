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
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    startState = problem.getStartState()

    fringe = util.Stack()
    visited = []

    fringe.push((startState, [], 0))

    while not fringe.isEmpty():
        currentState, actions, costs = fringe.pop()
        if not currentState in visited:
            "update visited status"
            visited.append(currentState)
            "if this goal state return the actions to reach it"
            if problem.isGoalState(currentState):
                return actions
            "push all successors not in visited"
            for state, action, cost in problem.getSuccessors(currentState):
                if not state in visited:
                    fringe.push((state, actions + [action], cost))
    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    startState = problem.getStartState()

    fringe = util.Queue()
    visited = []

    fringe.push((startState, [], 0))

    while not fringe.isEmpty():
        currentState, actions, costs = fringe.pop()
        if not currentState in visited:
            "update visited status"
            visited.append(currentState)
            "if this goal state return the actions to reach it"
            if problem.isGoalState(currentState):
                return actions
            "push all successors not in visited"
            for state, action, cost in problem.getSuccessors(currentState):
                if not state in visited:
                    fringe.push((state, actions + [action], cost))
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    startState = problem.getStartState()
    "ucs using priority queue so as to prioriize the successors with least cost"
    fringe = util.PriorityQueue()
    visited = []

    "the fringe apart from the state , action, cost also has priority 0 here"
    "which is same as the cost as we want the least total cost first"
    fringe.push((startState, [], 0), 0 )

    "keep popping till no more nodes in the fringe"
    while not fringe.isEmpty():
        currentState, actions, costs = fringe.pop()
        "curcial as this prevents expanding the same node twice"
        if not currentState in visited:
            "update visited status"
            visited.append(currentState)
            "if this goal state return the actions to reach it"
            if problem.isGoalState(currentState):
                return actions
            "push all successors not in visited"
            for state, action, cost in problem.getSuccessors(currentState):
                if not state in visited:
                    "update cost to reflect total cost and prioritize the least"
                    "as the priority queue is implemeneted using heapq which pops"
                    "smallest element first and pushes such to maintain this order"
                    fringe.push((state, actions + [action], costs + cost), costs + cost)
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    startState = problem.getStartState()
    startPath = ()
    frontier = util.PriorityQueue()
    frontier.push((startState, startPath), 0)
    pathCost = {startState: 0}

    while True:
        if frontier.isEmpty(): return None
        visitState, visitPath = frontier.pop()
        if problem.isGoalState(visitState): return list(visitPath)
        for state, action, cost in problem.getSuccessors(visitState):
            path = visitPath + (action, )
            gn = problem.getCostOfActions(path)
            hn = heuristic(state, problem)
            fn = gn + hn

            if state in pathCost and pathCost[state] <= gn: continue
            frontier.push((state, path), fn)
            pathCost[state] = gn


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
