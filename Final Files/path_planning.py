"""
* Team Id : GG_1949
* Author List : Ayush Karapagale, Akshit Gangawar, Prakhar Shukla, Sparsh Gautam
* Filename: path_planning.py
* Theme: Geo Guide -- EYRC
* Functions: dfs_paths, calculate_length, calculate_turns, find_forward_paths, find_backward_paths, find_nearest_event_node, complete_path
* Global Variables: arena, coordinates, event_nodes
"""

# Adjacency matrix for the arena
arena = [
    # Row 0 to 11 with obstacles and weights
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],      # 0
    [1, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0],      # 1
    [0, 3, 0, 0, 1, 100, 0, 0, 0, 0, 0, 0],    # 2
    [0, 1, 0, 0, 3, 0, 1, 0, 0, 0, 0, 0],      # 3
    [0, 0, 1, 3, 0, 3, 0, 1, 0, 0, 0, 0],      # 4
    [0, 0, 100, 0, 3, 0, 0, 0, 1, 0, 0, 0],    # 5
    [0, 0, 0, 1, 0, 0, 0, 3, 0, 1, 0, 0],      # 6
    [0, 0, 0, 0, 1, 0, 3, 0, 3, 0, 1, 0],      # 7
    [0, 0, 0, 0, 0, 1, 0, 3, 0, 0, 0, 1],      # 8 
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 3, 12],     # 9
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 3, 0, 3],      # 10
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 12, 3, 0],     # 11
]

# Mapping grid positions to (x, y) coordinates
coordinates = [(0, 0), (0, 1), (1, 1), (0, 2), (1, 2), (2, 2), (0, 3), (1, 3), (2, 3), (0, 4), (1, 4), (2, 4)]

event_nodes = {
    # The events and the corresponding events between which those events fall
    'A': (1, 2),
    'B': (4, 5),
    'C': (7, 8),
    'D': (6, 7),
    'E': (9, 11)
}

def dfs_paths(graph, start, goal, path=None):
    """
    Function Name: dfs_paths
    Input: graph (2D list), start (int), goal (int), path (list)
    Output: Generator of ALL paths from start to goal
    Logic: Performs depth-first search to find all paths from start to goal
    Example Call: dfs_paths(arena, 0, 11)
    """
    if path is None:
        path = [start]
    if start == goal:
        yield path
    for next_node in range(len(graph[start])):
        if graph[start][next_node] != 0 and next_node not in path:
            yield from dfs_paths(graph, next_node, goal, path + [next_node])

def calculate_length(path):
    """
    Function Name: calculate_length
    Input: path (list)
    Output: int
    Logic: Calculates the total length of the given path
    Example Call: calculate_length([0, 1, 2])
    """
    return sum(arena[path[i]][path[i+1]] for i in range(len(path)-1))

def calculate_turns(path):
    """
    Function Name: calculate_turns
    Input: path (list)
    Output: int
    Logic: Calculates the number of turns in the given path. If the x_curr - x_prev, y_curr - y_prev are not the same as x_next - x_curr, y_next - y_curr, then there exists a turn.
    Example Call: calculate_turns([0, 1, 2])
    """
    turns = 0
    for i in range(1, len(path)-1):
        dx1 = coordinates[path[i]][0] - coordinates[path[i-1]][0]
        dx2 = coordinates[path[i+1]][0] - coordinates[path[i]][0]
        dy1 = coordinates[path[i]][1] - coordinates[path[i-1]][1]
        dy2 = coordinates[path[i+1]][1] - coordinates[path[i]][1]
        if dx1 != dx2 or dy1 != dy2:
            turns += 1
    return turns

def find_forward_paths(start, goal):
    """
    Function Name: find_forward_paths
    Input: start (int), goal (int)
    Output: list
    Logic: Finds the shortest path with minimum turns from start to goal
    Example Call: find_forward_paths(0, 11)
    """
    all_paths = list(dfs_paths(arena, start, goal))
    min_length = min(map(calculate_length, all_paths))
    shortest_paths = [path for path in all_paths if calculate_length(path) == min_length]
    min_turns = min(map(calculate_turns, shortest_paths))
    path_forward = [path for path in shortest_paths if calculate_turns(path) == min_turns][0]
    return path_forward

def find_backward_paths(start, goal, path_forward):
    """
    Function Name: find_backward_paths
    Input: start (int), goal (int), path_forward (list)
    Output: list
    Logic: Finds an alternate path from start to goal avoiding the second last node in path_forward. i.e. avoiding any u-turns.
    Example Call: find_backward_paths(0, 11, [0, 1, 2])
    """
    all_paths = list(dfs_paths(arena, start, goal))
    valid_paths = [path for path in all_paths if path_forward[-2] != path[1]]
    min_length = min(map(calculate_length, valid_paths))
    shortest_paths = [path for path in valid_paths if calculate_length(path) == min_length]
    min_turns = min(map(calculate_turns, shortest_paths))
    path_backward = [path for path in shortest_paths if calculate_turns(path) == min_turns][0]
    return path_backward

def find_nearest_event_node(start, event_nodes):
    """
    Function Name: find_nearest_event_node
    Input: start (int), event_nodes (dict)
    Output: int
    Logic: Finds the nearest node corresponding to an event from the given position.
    Example Call: find_nearest_event_node(0, event_nodes)
    """
    distances = [(abs(coordinates[start][0] - coordinates[node][0]) + abs(coordinates[start][1] - coordinates[node][1]), node) for node in event_nodes]
    nearest_node = min(distances)[1]
    return nearest_node

def complete_path(events):
    """
    Function Name: complete_path
    Input: events (list of str)
    Output: list
    Logic: Constructs a complete path visiting the event nodes in the given order
    Example Call: complete_path(['E', 'D', 'B', 'A', 'C'])
    """
    end = 0
    goal = 0
    curr = 0

    path = []

    e = event_nodes[events[0]]
    nearest_node = find_nearest_event_node(curr, e)
    other_node = e[1] if nearest_node == e[0] else e[0]

    path = find_forward_paths(curr, nearest_node)
    path.append(other_node)
    curr = other_node

    for event in events[1:]:
        e = event_nodes[event]
        nearest_node = find_nearest_event_node(curr, e)
        other_node = e[1] if nearest_node == e[0] else e[0]
        
        path.extend(find_backward_paths(curr, nearest_node, path))
        path.append(other_node)
        curr = other_node
        
    path.extend(find_backward_paths(curr, end, path))

    resultant_path = [num for i, num in enumerate(path) if i == 0 or num != path[i-1]]

    return resultant_path

if __name__ == "__main__":
    events = ['E', 'D', 'B', 'A', 'C']
    print(complete_path(events))