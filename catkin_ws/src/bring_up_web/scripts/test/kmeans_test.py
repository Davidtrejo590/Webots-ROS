#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from random import uniform
import math
import copy


# POINTS DETECTED
x = [   -6.753, -6.749, -6.745, -6.741, -6.737, -5.577, -5.417, -5.275, -5.241, -5.223, -5.222, -5.239,
        -5.278, -5.357, -6.689, -6.685, -6.681, -6.677, -3.328, -3.106, -2.912, -2.735, -2.576, 2.227,
        2.521, 2.822, 3.128, 3.459, 3.556, 3.704, 3.873, 4.079, 10.807, 10.795, 8.217, 8.109, 8.076,
        8.122, 8.222, 8.474, 10.703, 10.691, 10.679
    ]

y = [   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0
    ]


z = [   0.0, 0.118, 0.237, 0.355, 0.474, 0.491, 0.573, 0.651, 0.741, 0.832, 0.926, 1.024, 1.128, 1.244,
        1.678, 1.802, 1.927, 2.053, 10.492, 10.425, 10.438, 10.513, 10.669, 16.822, 16.771, 16.768,
        16.783, 10.277, 9.982, 9.847, 9.772, 9.787, 1.916, 1.72, 1.161, 1.001, 0.854, 0.715, 0.578,
        0.447, 0.376, 0.188, 0.0
    ]


def generate_centroids(dataset, k):
    min_x, min_y, min_z = np.amin(dataset, axis=0)                                              # GET MIN VALUE FOR X-AXIS AND Y-AXIS
    max_x, max_y, max_z = np.amax(dataset, axis=0)                                              # GET MAX VALUE FOR X-AXIS AND Y-AXIS
    print(min_x, max_x)
    print(min_y, max_y)
    print(min_z, max_z)
    centroids = [ 
                    np.array([ 
                        uniform(min_x, max_x), 
                        uniform(min_y, max_y), 
                        uniform(min_z, max_z)  
                        ]) 
                        for i in range(k)]

    return centroids                                                                            # RETURN K-CENTROIDS


def calculate_centroids(point_cloud, centroids):
    new_centroids = []                                                                          # CENTROITS RECALCULATED          
    clusters = [[] for c in centroids]                                                          # CLUSTERS (GROUPS)

    # ASSIGN EACH POINT IN ITS CORRESPOND CLUSTER
    for p in point_cloud:
        distances = [ 
                        math.sqrt((c[0] - p[0])**2 + (c[1] - p[1])**2 + (c[2] - p[2])**2 )      # EUCLEDIAN DISTANCE BETWEEN EACH POINT WITH EACH CENTROID
                        for c in centroids
                    ]     
        k_index = distances.index(min(distances))                                               # GET THE MINIMUM DISTANCES FOR EACH POINT
        clusters[k_index].append(p)                                                             # STORE IN THE CORRESPOND CLUSTER (GROUP)


    # RECOMPUTE CENTROIDS WITH THE MEAN OF EACH CLUSTER
    for cl in clusters:
        if cl:
            mean_k = np.mean(cl, axis=0)                                                        # COMPUTE THE MEAN OF EACH CLUSTER
            new_centroids.append(mean_k)                                                        # ADD THE NEW CENTROID
            

    return new_centroids                                                                        # RETURN A LIST WITH THE NEW CENTROIDS


# CALCUALTE THE DISTANCE BETWEEN EACH CENTROID D(OLD_CENTROID, NEW_CENTROID)
def compare_centroids(new_c, old_c):
    total_distance = 0.0

    for (oc, nc) in zip(old_c, new_c):                          
        distance = math.sqrt( (nc[0] - oc[0])**2 + (nc[1] - oc[1])**2 + (nc[2] - oc[2])**2 )    # EUCLEDIAN DISTANCE D(OLD_CENTROID, NEW_CENTROID)
        # print('Centroid Distance:', oc, nc, distance)
        total_distance = total_distance + distance                                              # SUM OF EACH DISTANCE 
    
    return total_distance                                                                       # RETURN THE SUM OF THE DISTANCES


# TO GRAPH
def destructure(points):
    xc, yc, zc = [[], [], []]
    for point in points:
        xc.append(point[0])
        yc.append(point[1])
        zc.append(point[2])

    return [xc, yc, zc]


def main():

    point_cloud = np.column_stack((x, y, z))                                                    # GET POINTS AS A NUMPY ARRAY
    k = 3                                                                                       # DEFINE NUMBER OF CLUSTERS
    tol = 0.1                                                                                   # DEFINE MINIMU TOLERANCE
    attempts = 0                                                                                # INIT ATTEMPTS
    max_attempts = 100                                                                          # DEFINE MAX ATTEMPTS


    # MY K MEANS 
    initial_centroids = generate_centroids(point_cloud, k)                                             # GENERATE K-CENTROIDS
    print('Centroides Iniciales: ', initial_centroids, '\n')
    ic_x, ic_y, ic_z = destructure(initial_centroids)                                                  # TO GRAPH
    new_centroids = calculate_centroids(point_cloud, initial_centroids)
    print('Nuevos Centroides', new_centroids, '\n')
    total_distance = compare_centroids(new_centroids, initial_centroids)
    print('Distancia total entre centroides: ', total_distance, '\n')
    
    while total_distance > tol and attempts < max_attempts :
        print('IN WHILE')
        centroids = copy.deepcopy(new_centroids)
        print('Centroides: ', centroids, '\n')
        c_x, c_y, c_z = destructure(centroids)                                                         # TO GRAPH
        new_centroids = calculate_centroids(point_cloud, centroids)
        print('Nuevos Centroides', new_centroids, '\n')
        nc_x, nc_y, nc_z = destructure(new_centroids)                                                  # TO GRAPH
        total_distance = compare_centroids(new_centroids, centroids)
        print('Distancia total entre centroides: ', total_distance)
        attempts+=1
        print('ATTEMPTS: ', attempts)
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(x, y, z, s = 50, c = 'black', marker ='.', label = 'Cloud Point')
    ax.scatter(ic_x, ic_y, ic_z, s = 50, c = 'red', marker = '*', label = 'Initial Centroids')
    ax.scatter(c_x, c_y, c_z, s = 50, c = 'blue', marker = '*', label = 'Old Centroids')
    ax.scatter(nc_x, nc_y, nc_z, s = 50, c = 'forestgreen', marker = '*', label = 'New Centroids')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.legend()
    plt.show()



    

if __name__ == '__main__':
    try:
        main()
    except:
        print('Error')
        pass
