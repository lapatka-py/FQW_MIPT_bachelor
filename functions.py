#!/usr/bin/env python
# coding: utf-8

# In[1]:


import matplotlib.pyplot as plt
import numpy as np


# In[2]:


import json

json_file_path = r"data/ways_with_points_cleared_5m_v2.json"
with open(json_file_path, 'r') as j:
    ways_with_points = json.loads(j.read())
    
json_file_path = r"data/way_with_neighbours.json"
with open(json_file_path, 'r') as j:
    ways_with_neighbours = json.loads(j.read())   
    
json_file_path = r"data/points_with_ways_new.json"
with open(json_file_path, 'r') as j:
    points_with_ways = json.loads(j.read())  


# In[3]:


ways = np.array([int(i) for i in list(ways_with_neighbours.keys())])
lat = np.array([float(i.split(', ')[0]) for i in list(points_with_ways.keys())])
lon = np.array([float(i.split(', ')[1]) for i in list(points_with_ways.keys())])
point_names = list(points_with_ways.keys())
coordinates = np.vstack((lat, lon)).T#.shape


# # bfs

# In[14]:


def bfs(graph, start_node, end_node):
    visited = [] 
    queue = []     
    sequence = {}
    max_seq_len_exceeded = False
    
    visited.append(start_node)
    queue.append(start_node)
    
    current_node = start_node
    sequence[current_node] = [current_node]
    
    while current_node != end_node:
        if (len(queue) == 0) or (max_seq_len_exceeded):
            return []
        current_node = queue.pop(0)
        #print (current_node, graph[current_node],  visited) 
        for neighbour in graph[current_node]:
            
            if neighbour == end_node:
                #print('fin', sequence[current_node]+[neighbour])
                sequence[neighbour] = sequence[current_node]+[neighbour]
                current_node = neighbour
                    
            elif neighbour not in visited and neighbour in graph.keys():
                #print('ok', neighbour)
                visited.append(neighbour)
#                 print('Путь', neighbour,'его соседи', graph[neighbour],'посещены ли они уже', [(i in visited) for i in graph[neighbour]])
                sequence[neighbour] = sequence[current_node]+[neighbour]
                if len(sequence[neighbour]) > 70:
                    print('far route')
                    max_seq_len_exceeded = True
    
                queue.append(neighbour)

    
    return sequence[current_node]


# # find closest nodes

# In[15]:


def find_closest_nodes(start_point, end_point):
    
    start_n  = np.argmin(np.linalg.norm(coordinates - start_point, axis=1))
    end_n    = np.argmin(np.linalg.norm(coordinates - end_point, axis=1))
    start_point = point_names[start_n]
    end_point   = point_names[end_n]
    
    return points_with_ways[start_point][0], points_with_ways[end_point][0], start_point, end_point


# # find route

# In[16]:


def find_route(start_point, end_point, ways_with_nodes):   
    
    start_way, end_way, start_node, end_node = find_closest_nodes(start_point, end_point)
    
    if get_distance(float(start_node.split(',')[0]), float(start_node.split(',')[1]), 
                    float(end_node.split(',')[0]), float(end_node.split(',')[1])) < 0.1:
        return 'it is idle', [], [], [] # проверка на то, что между ними 100 метров
    
    if start_way == end_way:
        one_way = start_way
        our_one_way_nodes = ways_with_nodes[start_way]
        start_point_index = our_one_way_nodes.index(start_node)
        end_point_index = our_one_way_nodes.index(end_node)
        
        if start_point_index < end_point_index:
            nodes_on_route = our_one_way_nodes[start_point_index:end_point_index+1]
        else:
            nodes_on_route = our_one_way_nodes[end_point_index:start_point_index+1][::-1]
            
        return nodes_on_route, [one_way], [1]*len(nodes_on_route), []
        
    #print('1-2')    
    way_on_route = bfs(ways_with_neighbours, start_way, end_way)
    
    # получили список путей
    if len(way_on_route) == 0:
        return 'it is not bound points', [], [], []
    
    nodes_on_route = []
    colors_for_ways = []
    
    intersect = start_node
    intersects = []
    #print('2-2')
    for i in range(1,len(way_on_route)):
        # найдем индекс в старом и новом путях
        previous_way_nodes = ways_with_nodes[way_on_route[i-1]]
        next_way_nodes = ways_with_nodes[way_on_route[i]]
        
        # найдем пересечение нового со старым
        new_intersect = [j for j in previous_way_nodes if j in next_way_nodes][0] 
        #print('intersection:',way_on_route[i-1], [j for j in previous_way_nodes if j in next_way_nodes])
        
        pr_start_index = previous_way_nodes.index(intersect)
        pr_end_index = previous_way_nodes.index(new_intersect)
 
        if pr_start_index < pr_end_index:
            additional_nodes = previous_way_nodes[pr_start_index:pr_end_index+1]
        else:          
            additional_nodes = previous_way_nodes[pr_end_index:pr_start_index+1][::-1]
        
        intersect = new_intersect
        intersects.append(intersect)
        
        nodes_on_route += additional_nodes    
        colors_for_ways += [way_on_route[i-1]]*len(additional_nodes)

    pr_start_index = next_way_nodes.index(intersect)
    pr_end_index = next_way_nodes.index(end_node)        
    if pr_start_index < pr_end_index:
        additional_nodes = next_way_nodes[pr_start_index:pr_end_index+1]
    else:
        additional_nodes = next_way_nodes[pr_end_index:pr_start_index+1][::-1]
        
    old_way_on_route = way_on_route[0]
    nodes_on_route += additional_nodes 
    colors_for_ways += [way_on_route[i]]*len(additional_nodes)
    
    return nodes_on_route, way_on_route, colors_for_ways, intersects


# # get distance

# In[17]:


def get_distance(lat1, lon1, lat2, lon2):
    
    import numpy as np
    lat1 = np.radians(lat1)
    lon1 = np.radians(lon1)
    lat2 = np.radians(lat2)
    lon2 = np.radians(lon2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2
    c = 2 * np.arctan(np.sqrt(a) / np.sqrt(1 - a))
    
    # approximate radius of earth in km
    R = 6373.0
    
    distance = R * c
    return distance


# # get way

# In[18]:


def get_way(start_point, end_point):
#     lat_all, lon_all = [], []

    nodes_on_route, ways_on_route, colors_for_ways, intersects = find_route(start_point, end_point, ways_with_points)
    
#     plt.figure(figsize=(18,10))
#     for way0 in ways_on_route:
#         way = ways_with_points[way0]
#         lat, lon = [float(x.split(', ')[0]) for x in way], [float(x.split(', ')[1]) for x in way]
#         #print(way0,way)
#         plt.plot(lon,lat)
#         plt.title("Все участки")
#         lat_all+=lat
#         lon_all+=lon

    lat, lon = [], []
    
    if nodes_on_route == 'it is not bound points' or nodes_on_route == 'it is idle':
        
        return [], 0
        
    else:
        
        for node_coordinate in nodes_on_route:
            lat.append(float(node_coordinate.split(', ')[0]) )
            lon.append(float(node_coordinate.split(', ')[1]) )
 
    distance = 0
            
    for lat1, lat2, lon1, lon2 in zip(lat[:-1], lat[1:], lon[:-1], lon[1:]):
        additional_distance = get_distance(lat1, lon1,  lat2, lon2)
        distance += additional_distance
        
    return nodes_on_route, distance


# In[19]:


start_point = np.array((63.428844, 53.607670))
end_point = np.array((67.470200, 64.026861))
nodes, dist = get_way(start_point,end_point)


# In[20]:


dist


# In[ ]:




