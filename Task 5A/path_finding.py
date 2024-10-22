arena = [
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],      #0
    [1, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0],      #1
    [0, 3, 0, 0, 1, 7, 0, 0, 0, 0, 0, 0],      #2
    [0, 1, 0, 0, 3, 0, 1, 0, 0, 0, 0, 0],      #3
    [0, 0, 1, 3, 0, 3, 0, 1, 0, 0, 0, 0],      #4
    [0, 0, 7, 0, 3, 0, 0, 0, 1, 0, 0, 0],      #5
    [0, 0, 0, 1, 0, 0, 0, 3, 0, 1, 0, 0],      #6
    [0, 0, 0, 0, 1, 0, 3, 0, 3, 0, 1, 0],      #7
    [0, 0, 0, 0, 0, 1, 0, 3, 0, 0, 0, 1],      #8 
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 3, 12],     #9
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 3, 0, 3],      #10
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 12, 3, 0],     #11
]

event_nodes = {
    'A': [1, 2],
    'B': [4, 5],
    'C': [7, 8],
    'D': [6, 7],
    'E': [9, 11],
}

coordinates = [(0, 0), (0, 1), (1, 1), (0, 2), (1, 2), (2, 2), (0, 3), (1, 3), (2, 3), (0, 4), (1, 4), (2, 4)]

def find_paths_with_turns(adj_matrix, source, destination):
    paths = []
    visited = set()

    def dfs(current, path):
        visited.add(current)

        for neighbor, weight in enumerate(adj_matrix[current]):
            if neighbor not in visited and weight != 0:
                new_path = path + [neighbor]
                if neighbor == destination:
                    paths.append(new_path)
                else:
                    dfs(neighbor, new_path)

        visited.remove(current)

    dfs(source, [source])
    return paths

def calculate_turns(path):
    turns = 0

    for i in range(1, len(path) - 1):
        current_coord = coordinates[path[i]]
        prev_coord = coordinates[path[i - 1]]
        next_coord = coordinates[path[i + 1]]

        dx1, dy1 = current_coord[0] - prev_coord[0], current_coord[1] - prev_coord[1]
        dx2, dy2 = next_coord[0] - current_coord[0], next_coord[1] - current_coord[1]

        if (dx1, dy1) != (dx2, dy2):
            turns += 1

    return turns

def fastest_paths(adj_matrix, source, destination):
    paths = find_paths_with_turns(adj_matrix, source, destination)

    if not paths:
        return [], [], float('inf'), float('inf')  

    path_details = []

    for path in paths:
        length = sum(adj_matrix[path[i]][path[i + 1]] for i in range(len(path) - 1))
        turns = calculate_turns(path)
        path_details.append((path, length, turns))

    path_details.sort(key=lambda x: (x[1], x[2]))

    first_shortest_path, second_shortest_path = [], []
    first_min_length, second_min_length = float('inf'), float('inf')

    if len(path_details) > 0:
        first_shortest_path, first_min_length, _ = path_details[0]
        if len(path_details) > 1:
            second_shortest_path, second_min_length, _ = path_details[1]

    return first_shortest_path, second_shortest_path, first_min_length, second_min_length


def find_nearest_node_from_source(adj_matrix, source, nodes):
    min_length = float('inf')
    nearest_node = None

    for node in nodes:
        path, _ , length, _ = fastest_paths(adj_matrix, source, node)

        if length < min_length:
            min_length = length
            nearest_node = node
    return nearest_node

def find_path_through_event(adj_matrix, source, event_nodes):
    path_to_nearest_node = []

    if source in event_nodes:
        nearest_node = source
        path_to_nearest_node.append(nearest_node)
        final_event_node = [node for node in event_nodes if node != nearest_node][0]
        path_to_nearest_node.append(final_event_node)
    else:
        nearest_node = find_nearest_node_from_source(adj_matrix, source, event_nodes)

        if nearest_node is None:
            return []

        path_to_nearest_node, _, _, _ = fastest_paths(adj_matrix, source, nearest_node)

        final_event_node = [node for node in event_nodes if node != nearest_node][0]
        path_to_nearest_node.append(final_event_node)

    return path_to_nearest_node, final_event_node

def priority_route(adj_matrix, priority_order):
    source = 0
    prioritized_path = []

    for event in priority_order:
        fastest_path, final_node = find_path_through_event(adj_matrix, source, event_nodes[event])

        if not prioritized_path:
            prioritized_path.append(fastest_path)
        else:
            if prioritized_path[-1][-2] == fastest_path[1]:
                _, second_fastest, _, _ = fastest_paths(adj_matrix, prioritized_path[-1][-1], prioritized_path[-1][-2])
                print(f'second fastest: {second_fastest}')
                prioritized_path.append(second_fastest)
                prioritized_path.append(fastest_path[1:])
            else:
                prioritized_path.append(fastest_path)

        source = final_node
    
    to_end, _, _, _ = fastest_paths(adj_matrix, source, 0)

    print(prioritized_path)
    print(to_end)
    if (prioritized_path[-1][-2] == to_end[1]):
        _,to_end, _, _ = fastest_paths(adj_matrix, prioritized_path[-1][-1], 0)
        prioritized_path.append(to_end)
        print(to_end)
    else:
        prioritized_path.append(to_end)

    flattened_path = [item for sublist in prioritized_path for item in sublist]

    unique_path = [flattened_path[0]]  

    for item in flattened_path[1:]:
        if item != unique_path[-1]:  
            unique_path.append(item)

    return unique_path

if __name__ == "__main__":

    event_priority = ['E', 'B', 'A', 'D', 'C']
    entire_path = priority_route(arena, event_priority)

    print(entire_path) 