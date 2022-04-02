#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems


def corner_box(box, state):
    """Return True if the box is in the corner"""
    # Check if up down left right are blocking by the wall of the obstacles
    up = box[1] == 0 or ((box[0], box[1] + 1) in state.obstacles)
    down = (box[1] == state.height - 1) or ((box[0], box[1] - 1) in state.obstacles)

    left = (box[0] == 0) or ((box[0] - 1, box[1]) in state.obstacles)
    right = (box[0] == state.width - 1) or ((box[0] + 1, box[1]) in state.obstacles)

    # if two near sides have blocked return True
    return (up or down) and (left or right)


def edge_box(box, state):
    return None


# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current
    # state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your
    # solution file, to facilitate marking.

    box_distance = 0
    rob_distance = 0

    box_lst = []
    # Check if the box in the storage point.
    for box in state.boxes:
        if box not in state.storage:
            box_lst.append(box)
    if not box_lst:     # if box_lst is left, every box is in the storage point. Return immediately
        return 0

    # Find the min distance between robot and the boxes
    for robot in state.robots:
        distance1 = 2**31
        for box in state.boxes:
            # calculating manhattans distance
            temp = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
            distance1 = min(distance1, temp)
        rob_distance += distance1

    # Checking if there are any leftover boxes in the corner
    # Find the min distance between box and storage points
    for box in box_lst:
        distance2 = 2**31
        if corner_box(box, state):
            return 2**31
        for pos in state.storage:
            # calculating manhattans distance
            dx = box[0] - pos[0]
            dy = box[1] - pos[1]
            temp = abs(dx) + abs(dy)
            if distance2 is None:
                distance2 = temp
            else:
                distance2 = min(distance2, temp)

        box_distance += distance2

    return rob_distance + 0.5 * box_distance


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to
    # it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    """
    Return manhattan distance. Checking every box in the state, and find the nearest storage for each box
    and upload it into count. 
    """
    count = 0
    for box in state.boxes:
        distance = None
        for storage in state.storage:
            dx = box[0] - storage[0]
            dy = box[1] - storage[1]
            temp = abs(dx) + abs(dy)
            if distance is None:
                distance = temp
            else:
                distance = min(distance, temp)
        count += distance

    return count


def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval


# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))

    se = SearchEngine()     # Initialize a SearchEngine
    se.set_strategy('custom')
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)

    return se.search(timebound)


def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    # Continue to search
    # 1. there are no nodes left to expand (and our best solution is the optimal one)
    # 2. it runs out of time

    # Initialize time
    time = os.times()[0]            # Get the current system time
    end_time = time + timebound     # The current time + timebound -> the max time bound (end_time)
    new_timebound = timebound
    # Initialize costbound into infinite large.
    costbound = (float('inf'), float('inf'), float('inf'))
    # The first weighted A*
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se = SearchEngine()  # Initialize a SearchEngine
    se.set_strategy('custom')
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    result = se.search(timebound)
    best = result
    # while loop if there still have time left.
    while time < end_time:
        if not result[0]:     # did not find the solution where result is False
            return best

        # Updating the current time and new timebound
        # Collect the different between current and previous time to know the time that previous search has used.
        # So the new timebound will be the leftover time, which is iteratively decreasing timebound.
        diff_time = os.times()[0] - time
        time = os.times()[0]
        new_timebound = new_timebound - diff_time

        # Check if the gval is smaller than costbound, update the best if so. Since this means we find a better solution
        if result[0].gval < costbound[0]:
            costbound = (result[0].gval, result[0].gval, result[0].gval)
            best = result

        # Iteratively increasing weight with small amount.
        weight += 1
        wrapped_fval_function = (lambda sN: fval_function(sN, weight))
        se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
        result = se.search(new_timebound, costbound)
    return best


def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''

    # Initialize time
    time = os.times()[0]  # Get the current system time
    end_time = time + timebound  # The current time + timebound -> the max time bound (end_time)
    new_timebound = timebound
    # Initialize costbound into infinite large.
    costbound = (float('inf'), float('inf'), float('inf'))
    # The first weighted A*
    se = SearchEngine()  # Initialize a SearchEngine
    se.set_strategy('best_first', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    result = se.search(timebound)
    best = result
    # while loop if there still have time left.
    while time < end_time:
        if not result[0]:  # did not find the solution where result is False
            return best

        # Updating the current time and new timebound
        # Collect the different between current and previous time to know the time that previous search has used.
        # So the new timebound will be the leftover time, which is iteratively decreasing timebound.
        diff_time = os.times()[0] - time
        time = os.times()[0]
        new_timebound = new_timebound - diff_time

        # Check if the gval is smaller than costbound, update the best if so. Since this means we find a better solution
        if result[0].gval < costbound[0]:
            costbound = (result[0].gval, result[0].gval, result[0].gval)
            best = result

        # Iteratively increasing weight with small amount.
        se.init_search(initial_state, sokoban_goal_state, heur_fn)
        result = se.search(new_timebound, costbound)
    return best



