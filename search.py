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
import sys
import copy
import operator

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

    def goalTest(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
        Given a state, returns available actions.
        Returns a list of actions
        """        
        util.raiseNotDefined()

    def getResult(self, state, action):
        """
        Given a state and an action, returns resulting state.
        """
        util.raiseNotDefined()

    def getCost(self, state, action):
        """
        Given a state and an action, returns step cost, which is the incremental cost 
        of moving to that successor.
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

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.

    You are not required to implement this, but you may find it useful for Q5.
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def iterativeDeepeningSearch(problem):
    """
    Perform DFS with increasingly larger depth.

    Begin with a depth of 1 and increment depth by 1 at every step.
    """
    "*** YOUR CODE HERE ***"
    x = 1;
    while True:
        visited = util.Queue() #hummmmmmm
        solution = util.Queue() #hummmmm
        border = util.Stack() #border??? frontierrr????
        result = BPLRecursive(problem.getStartState(), problem, x, solution, visited, border)
        x += 1
        if result != 0:            
            return solution.list

def BPLRecursive(node, problem, limit, solution, visited, border):
    visited.push(node)
    if problem.goalTest(node):
        return True
    elif limit == 0:
        return 0
    else:
        cut = False
        actions = util.Queue()
        for action in problem.getActions(node):
            child = problem.getResult(node, action)
            border.push(child)
            actions.push(action)
        for action in actions.list:
            child = border.pop()
            if visited.list.count(child) == 0 and border.list.count(child) == 0:
                result = BPLRecursive(child, problem, limit - 1, solution, visited, border)
                if result == 0:
                    cut = True
                elif result is not None:
                    solution.push(action)
                    return True
        if cut:
            return 0
        else:
            return None

"""
def aStarSearch(problem, heuristic=nullHeuristic):
    start = problem.getStartState()
    frontier = util.PriorityQueue()
    frontier.push(start, 0)
    visited = util.Queue()
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.isEmpty():
        current = frontier.pop()
        visited.push(current)

        if problem.goalTest(current):
            break

        for action in problem.getActions(current):
            next = problem.getResult(current, action)

            if next not in visited.list:  

                new_cost = cost_so_far[current] + problem.getCost(current, action)
                priority = new_cost + heuristic(next, problem)
                frontier.push(next, priority)
                
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    came_from[next] = current
                    
    
    current = next
    solution = []
    while came_from[current] is not None:
        actions = problem.getActions(came_from[current])
        for action in actions:
            state = problem.getResult(came_from[current], action)
            if state == current:
                solution.append(action)
        current = came_from[current]

    return solution[::-1]
"""

def aStarSearch(problem, heuristic=nullHeuristic):
    frontier = util.Stack() #open set
    visited = util.Queue()  #closed set
    state = problem.getStartState() #initial state
    paths = {state: {'action': None, 'cameFrom': None}}
    costs = [{ 'state': state, 'g': 0, 'f': heuristic(state, problem)}]
    frontier.push(state)

    def getCost(state):
        for cost in costs:
            if cost['state'] == state:
                return cost
        return None

    def setCost(state, new_cost):
        state_cost = getCost(state)
        new_f_cost = new_cost + heuristic(state, problem)
        if not state_cost:
            # new path
            state_cost = {
                'state': state,
                'g': new_cost,
                'f': new_cost + heuristic(state, problem)
            }
            costs.append(state_cost)
        elif new_f_cost < state_cost['f'] or (new_f_cost == state_cost['f'] and new_cost < state_cost['g']):
            # Best path until now
            state_cost['g'] = new_cost
            state_cost['f'] = new_f_cost

    def reconstruct(paths, current):
        current = paths[current]
        solution = [current['action']]
        while current['cameFrom'] != None:
            current = paths[current['cameFrom']]
            if current['action']:
                solution.append(current['action'])
        return solution[::-1]

    while not frontier.isEmpty():
        current = sorted([cost for cost in costs if cost['state'] in frontier.list], key=lambda cost: cost['f'])[0]['state']
        if problem.goalTest(current):
            return reconstruct(paths, current)

        frontier.list = filter(lambda state: state != current, frontier.list)
        visited.push(current)

        for action in problem.getActions(current):
            neighbor = problem.getResult(current, action)

            if neighbor not in visited.list:

                if neighbor not in frontier.list:
                    frontier.push(neighbor)

                # distance from start to current state
                current_cost = getCost(current)
                new_cost = current_cost['g'] + problem.getCost(current, action)
                paths[neighbor] = {'action': action, 'cameFrom': current}
                setCost(neighbor, new_cost)
    return False

# Abbreviations
bfs = breadthFirstSearch
astar = aStarSearch
ids = iterativeDeepeningSearch
