# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""
import curses
from locale import currency

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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem, discovered = [], path = [], position = None):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    if position is None:
        position = problem.getStartState()


    if position not in discovered:
        discovered.append(position)

        #getting the new frontier
        frontier = problem.getSuccessors(position)

        # begin IMPLEMENTATION WITH STACK
        stack = util.Stack()
        for next_position in frontier:
            stack.push(next_position)

        while (not stack.isEmpty()):
            next_position = stack.pop()
        # finish IMPLEMENTATION WITH STACK

        # begin MY IMPPLEMENTATION
        #for next_position in frontier:
        # finish MY IMPPLEMENTATION


            if problem.isGoalState(next_position[0]): #check if I finally arrive to Goal
                path.append(next_position)

            else:
                path.append(next_position)
                depthFirstSearch(problem,discovered,path,next_position[0])

                if problem.isGoalState(path[len(path)-1][0]):
                    if(position == problem.getStartState()):
                        moves =[]
                        for move in path:
                            moves.append(move[1])
                        return moves

                else:
                    path.pop()

    return path


class Node():

    def __init__(self,state, cost=0):
        self.state = state
        self.parent = None
        self.children = []
        self.action = None
        self.cost = cost

    def setChild(self, state, action, cost = 0):
        child = Node(state)
        child.action =action
        child.cost = cost
        child.parent = self
        self.children.append(child)

        return child




def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    node = Node( problem.getStartState() )
    frontier = util.Queue()
    frontier.push(node)

    discovered = []

    while True:
        if frontier.isEmpty():
            return None
        node = frontier.pop()
        discovered.append(node)

        successors = problem.getSuccessors(node.state)

        for successor in successors:
            child = node.setChild(successor[0],successor[1])

            # check if it's already on frontier
            inFrontier = False
            for f in frontier.list:
                if f.state == child.state:
                    inFrontier = True

            # check if it's already on discovered
            inDiscovered = False
            for d in discovered:
                if d.state == child.state:
                    inDiscovered = True

            if inFrontier == False and inDiscovered ==False:
                # check if this is teh goal
                if problem.isGoalState(child.state):
                    #making the final transformation
                    moves=getActionsToChild(child)
                    return moves
                # child is no frontier nor discovered so it's a new frontier
                frontier.push(child)

def getActionsToChild(node):
    actions=[]
    while node.parent != None:
        actions.append(node.action)
        node = node.parent
    actions.reverse()
    return actions



def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    node = Node( problem.getStartState() )
    frontier = util.PriorityQueue()
    frontier.push(node,999999)

    discovered = []

    while True:
        if frontier.isEmpty():
            return None

        node = frontier.pop()

        moves = getActionsToChild(node)

        if problem.isGoalState(node.state):
            # making the final transformation
            return moves

        # add node as been discovered
        discovered.append(node)

        successors = problem.getSuccessors(node.state)

        for successor in successors:

            child = node.setChild(successor[0], successor[1])

            moves = getActionsToChild(child)

            child.cost = problem.getCostOfActions(moves)

            # check if it's already on frontier
            inFrontier = False

            n = -1
            for f in frontier.heap:
                n += 1
                if f[1].state == child.state:
                    #is already in the frontier
                    inFrontier = True

                    if child.cost < f[1].cost:
                        #this is a better way to get there
                        frontier.heap.pop(n)
                        frontier.push(child, child.cost)
                        break



            # check if it's already on discovered
            inDiscovered = False
            for d in discovered:
                if d.state == child.state:
                    inDiscovered = True

            if inFrontier == False and inDiscovered == False:
                frontier.push(child,child.cost)





def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    node = Node( problem.getStartState() )
    frontier = util.PriorityQueue()
    frontier.push(node,999999)

    discovered = []

    while True:
        if frontier.isEmpty():
            return None

        node = frontier.pop()

        moves = getActionsToChild(node)

        if problem.isGoalState(node.state):
            # making the final transformation
            return moves

        # add node as been discovered
        discovered.append(node)

        successors = problem.getSuccessors(node.state)

        for successor in successors:

            child = node.setChild(successor[0], successor[1])

            moves = getActionsToChild(child)

            #child.cost = problem.getCostOfActions(moves)
            g_n = problem.getCostOfActions(moves)
            h_n = heuristic(child.state,problem)
            f_n = g_n+h_n
            child.cost = f_n

            # check if it's already on frontier
            inFrontier = False

            n = -1
            for f in frontier.heap:
                n += 1
                if f[1].state == child.state:
                    #is already in the frontier
                    inFrontier = True

                    if child.cost < f[1].cost:
                        #this is a better way to get there
                        frontier.heap.pop(n)
                        frontier.push(child, child.cost)
                        break



            # check if it's already on discovered
            inDiscovered = False
            for d in discovered:
                if d.state == child.state:
                    inDiscovered = True

            if inFrontier == False and inDiscovered == False:
                frontier.push(child,child.cost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

