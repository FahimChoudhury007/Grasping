#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter code for EE106B grasp planning project.
Author: Amay Saxena, Tiffany Cappellari
Modified by: Kirthi Kumar
"""
# may need more imports
import numpy as np
from utils import *
import math
import trimesh
import vedo
import rospy
import random
import os

np.bool = np.bool_

MAX_GRIPPER_DIST = 0.075
MIN_GRIPPER_DIST = 0.03
GRIPPER_LENGTH = 0.105

import cvxpy as cvx # suggested, but you may change your solver to anything you'd like (ex. casadi)

def compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Compute the force closure of some object at contacts, with normal vectors 
    stored in normals. Since this is two contact grasp, we are using a basic algorithm
    wherein a grasp is in force closure as long as the line connecting the two contact
    points lies in both friction cones.

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
    object_mass (float): mass of the object

    Returns
    -------
    (float): quality of the grasp
    """
    normal1 = normals[0,:]
    normal1[2] = -normal1[2]

    normal2 = normals[1,:]
    normal2 = -normal2[2]

    vertex1 = vertices[0, :]
    vertex2 = vertices[1, :]

    line_eq = vertex1 - vertex2

    def unit_vector(vector):
        return vector / np.linalg.norm(vector)

    def angle_between(v1,v2):
        v1_u = unit_vector(v1)
        v2_u = unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    angle1 = angle_between(line_eq,normal1)
    angle2 = angle_between(line_eq,normal2)

    alpha = math.atan(mu)

    if angle1 > alpha and angle2 > alpha:    ### The coefficient of friction condition
        return 0
    if gamma == 0:
        return 0
                                                  ## Have to calculate the f's(friction cone) inorder to compare the torsional friction coeff cond##
    return 1

def get_grasp_map(vertices, normals, num_facets, mu, gamma):
    """ 
    Defined in the book on page 219. Compute the grasp map given the contact
    points and their surface normals

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient

    Returns
    -------
    (np.ndarray): grasp map
    """
    b_matrix = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,0],
                        [0,0,0,0],
                        [0,0,0,1]])
    grasp_map = []
    for vertex, normal in zip(vertices,normals):
        g, adj_g = look_at_general(vertex, -normal), adj(np.linalg.inv(g))
        grasp_map.append(adj_g@b_matrix)
    grasp_map = np.hstack(grasp_map)
    return grasp_map

def contact_forces_exist(vertices, normals, num_facets, mu, gamma, desired_wrench):
    """
    Compute whether the given grasp (at contacts with surface normals) can produce 
    the desired_wrench. Will be used for gravity resistance. 

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
    desired_wrench (np.ndarray):potential wrench to be produced

    Returns
    -------
    (bool): whether contact forces can produce the desired_wrench on the object
    """
    # YOUR CODE HERE
    grasp_map = get_grasp_map(vertices, normals, num_facets, mu, gamma)

    top = np.array([])
    bottom = np.array([])

    for i in range(num_facets):
        top = np.append(top, np.cos(i*2*np.pi/num_facets))
        bottom = np.append(bottom, np.sin(i*2*np.pi*num_facets))

    A = cvx.vstack([top, bottom])
    fc = cvx.Variable(8)
    b1 = cvx.Variable(num_facets)
    b2 = cvx.Variable(num_facets)
    objective = cvx.Minimize(cvx.norm(grasp_map @ fc - desired_wrench))

    constraints = [A @ b1 == fc[0], A @ b1 == fc[2], A @ b2 == fc[4], A @ b2 == fc[6]]
    constraints += [fc[1] >= 0, fc[5] >= 0]
    
    constraints += [cvx.abs(fc[3]) <= gamma * fc[1], cvx.abs(fc[7]) <= gamma * fc[5]]

    constraints += [b1 >= 0, b2 >= 0]
    constraints += [cvx.sum(b1) <= mu * fc[1], cvx.sum(b2) <= mu * fc[5]]

    problem = cvx.Problem(objective, constraints)
    problem.solve()

    if (problem.status == cvx.OPTIMAL) and (problem.value < 2):
        return True, problem.value
    return False, 0.0

def compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Gravity produces some wrench on your object. Computes whether the grasp can 
    produce an equal and opposite wrench.

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
        torsional friction coefficient
    object_mass (float): mass of the object

    Returns
    -------
    (float): quality of the grasp
    """
    # YOUR CODE HERE
    wrench = np.array([0, 0, object_mass*9.81, 0, 0, 0])
    is_resist, resist_val = contact_forces_exist(vertices, normals, num_facets, mu, gamma, wrench)
    return resist_val
    
"""
you're encouraged to implement a version of this method, 
def sample_around_vertices(delta, vertices, object_mesh=None):
    raise NotImplementedError
"""

def compute_robust_force_closure(vertices, normals, num_facets, mu, gamma, object_mass,desired_wrench, num_trials = 500):
    """
    Should return a score for the grasp according to the robust force closure metric.

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
        torsional friction coefficient
    object_mass (float): mass of the object

    Returns
    -------
    (float): quality of the grasp
    """
    # YOUR CODE HERE
    g = 9.81
    success = 0
    for _ in range(num_trials):
        sample_vertices = vertices + np.random.randn(*vertices.shape)*0.3
        sample_normals = normals + np.random.randn(*normals.shape)*0.3
        resist, value = contact_forces_exist(sample_vertices, sample_normals,num_facets,mu,gamma,desired_wrench)
        if resist:
            success +=1                                                       ## Implementing Monte Carlo Simulation
    return success/num_trials * value


def compute_ferrari_canny(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Should return a score for the grasp according to the Ferrari Canny metric.
    Use your favourite python convex optimization package. We suggest cvxpy.

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
        torsional friction coefficient
    object_mass (float): mass of the object

    Returns
    -------
    (float): quality of the grasp
    """
    # YOUR CODE HERE
    def get_friction_cone(num_facets, mu, gamma, object_mass):
        #Returns discretized friction cone with the desired number of facets
        F_1 = []
        F_2 = []
        f3 = object_mass * 9.8 / mu #force required so that force due to friction overcomes gravity
        for i in range(num_facets):
            F_1.append(np.cos(i*2*np.pi/num_facets) * mu * f3)
            F_2.append(np.sin(i*2*np.pi*num_facets) * mu * f3)
        pass
        F_3 = [f3 for _ in range(num_facets)]
        F_4 = [gamma * f3 for _ in range(num_facets)]

        return cvx.vstack([F_1, F_2, F_3, F_4])
    
    def minimize_f_norm_squared(vertices, normals, num_facets, mu, gamma, object_mass, wrench):
        F = get_friction_cone(num_facets, mu, gamma, object_mass)
        G = get_grasp_map(vertices, normals, num_facets, mu, gamma)

        f1 = cvx.Variable(4)
        f2 = cvx.Variable(4)
        alpha1 = cvx.Variable(num_facets)
        alpha2 = cvx.Variable(num_facets)

        f = cvx.hstack([f1, f2])
        objective = cvx.Minimize(cvx.norm(f)**2)

        constraints = [f1 == F@alpha1,
                       f2 == F@alpha2,
                       G@f == wrench,
                       alpha1 >= 0,
                       alpha2 >= 0
                       ]

        problem = cvx.Problem(objective, constraints)
        problem.solve()
        return problem.value

    num_samples = 10
    q_optimal = None
    for i in range(num_samples):
        random_vector = np.random.normal(size=6)
        test_wrench = random_vector / np.linalg.norm(random_vector)
        lq_hat = minimize_f_norm_squared(vertices, normals, num_facets, mu, gamma, object_mass, test_wrench)
        q_test = 1/np.sqrt(lq_hat)
        if q_optimal is None or q_test < q_optimal:
            q_optimal = q_test
            
    return q_optimal


def custom_grasp_planner(object_mesh, vertices, metric_name):
    """
    Write your own grasp planning algorithm! You will take as input the mesh
    of an object, and a pair of contact points from the surface of the mesh.
    You should return a 4x4 ridig transform specifying the desired pose of the
    end-effector (the gripper tip) that you would like the gripper to be at
    before closing in order to execute your grasp.

    You should be prepared to handle malformed grasps. Return None if no
    good grasp is possible with the provided pair of contact points.
    Keep in mind the constraints of the gripper (length, minimum and maximum
    distance between fingers, etc) when picking a good pose, and also keep in
    mind limitations of the robot (can the robot approach a grasp from the inside
    of the mesh? How about from below?). You should also make sure that the robot
    can successfully make contact with the given contact points without colliding
    with the mesh.

    The trimesh package has several useful functions that allow you to check for
    collisions between meshes and rays, between meshes and other meshes, etc, which
    you may want to use to make sure your grasp is not in collision with the mesh.

    Take a look at the functions find_intersections, find_grasp_vertices, 
    normal_at_point in utils.py for examples of how you might use these trimesh 
    utilities. Be wary of using these functions directly. While they will probably 
    work, they don't do excessive edge-case handling. You should spend some time
    reading the documentation of these packages to find other useful utilities.
    You may also find the collision, proximity, and intersections modules of trimesh
    useful.

    Feel free to change the signature of this function to add more arguments
    if you believe they will be useful to your planner.

    Parameters
    ----------
    object_mesh (trimesh.base.Trimesh): A triangular mesh of the object, as loaded in with trimesh.
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed

    Returns
    -------
    (4x4 np.ndarray): The rigid transform for the desired pose of the gripper, in the object's reference frame.
    """
    # constants -- you may or may not want to use these variables below
    num_facets = 64
    mu = 0.5
    gamma = 0.1
    object_mass = 0.25
    g = 9.8
    desired_wrench = np.array([0, 0, g * object_mass, 0, 0, 0]).T

    trials = 100
    delta = 0.04
    gravity_th = 3
    ferrari_th = 1
    force_c_th = 0.0
    
    # YOUR CODE HERE
    best_quality = None
    best_pose = None
    best_vertices = None
    for _ in range(trials):
        quality=0
        vertices_samples, normals_samples = sample_vertices(vertices, object_mesh, delta)
        if metric_name == "gravity":
            quality = compute_gravity_resistance(vertices_samples, normals_samples, num_facets, mu, gamma, object_mass)
            if best_quality == None or quality > best_quality:
                best_pose = get_gripper_pose(vertices, object_mesh)
                best_quality = quality
                best_vertices = vertices_samples
        elif metric_name =="robust":
            quality = compute_robust_force_closure(vertices_samples, normals_samples, num_facets, mu, gamma, object_mass, desired_wrench, num_trials = trials)
            if best_quality == None or quality > best_quality:
                best_pose = get_gripper_pose(vertices, object_mesh)
                best_quality = quality
                best_vertices = vertices_samples
        elif metric_name == "ferrari":
            quality = compute_ferrari_canny(vertices_samples, normals_samples, num_facets, mu, gamma, object_mass, desired_wrench, num_trials = trials)
            if best_quality == None or quality > best_quality:
                best_pose = get_gripper_pose(vertices, object_mesh)
                best_quality = quality
                best_vertices = vertices_samples
    return best_pose, best_vertices, best_quality


def sample_vertices(vertices, mesh, delta):
    potential_vertices = []
    potential_normals = []
    for v in vertices:
        potential_vertices.append(v)
        potential_normals.append(normal_at_point(mesh, v))
    return np.array(potential_vertices), np.array(potential_normals)

def get_gripper_pose(vertices, object_mesh): # you may or may not need this method 
    """
    Creates a 3D Rotation Matrix at the origin such that the y axis is the same
    as the direction specified.  There are infinitely many of such matrices,
    but we choose the one where the z axis is as vertical as possible.
    z -> y
    x -> x
    y -> z

    Parameters
    ----------
    origin : 3x1 :obj:`numpy.ndarray`
    x : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    4x4 :obj:`numpy.ndarray`
    """
    origin = np.mean(vertices, axis=0)
    direction = vertices[0] - vertices[1]

    up = np.array([0, 0, 1])
    y = normalize(direction)
    x = normalize(np.cross(up, y))
    z = np.cross(x, y)

    gripper_top = origin + GRIPPER_LENGTH * z
    gripper_double = origin + 2 * GRIPPER_LENGTH * z
    if len(find_intersections(object_mesh, gripper_top, gripper_double)[0]) > 0:
        z = normalize(np.cross(up, y))
        x = np.cross(y, x)
    result = np.eye(4)
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z
    result[0:3,3] = origin
    return result

def visualize_grasp(mesh, vertices, pose):
    """Visualizes a grasp on an object. Object specified by a mesh, as
    loaded by trimesh. vertices is a pair of (x, y, z) contact points.
    pose is the pose of the gripper tip.

    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    vertices (np.ndarray): 2x3 matrix, coordinates of the 2 contact points
    pose (np.ndarray): 4x4 homogenous transform matrix
    """
    p1, p2 = vertices
    center = (p1 + p2) / 2
    approach = pose[:3, 2]
    tail = center - GRIPPER_LENGTH * approach

    contact_points = []
    for v in vertices:
        contact_points.append(vedo.Point(pos=v, r=30))

    vec = (p1 - p2) / np.linalg.norm(p1 - p2)
    line = vedo.shapes.Tube([center + 0.5 * MAX_GRIPPER_DIST * vec,
                                   center - 0.5 * MAX_GRIPPER_DIST * vec], r=0.001, c='g')
    approach = vedo.shapes.Tube([center, tail], r=0.001, c='g')
    vedo.show([mesh, line, approach] + contact_points, new=True)

def randomly_sample_from_mesh(mesh, n):
    """Example of sampling points from the surface of a mesh.
    Returns n (x, y, z) points sampled from the surface of the input mesh
    uniformly at random. Also returns the corresponding surface normals.

    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    n (int): number of desired sample points

    Returns
    -------
    vertices (np.ndarray): nx3 matrix, coordinates of the n surface points
    normals (np.ndarray): nx3 matrix, normals of the n surface points
    """
    """"TODO: change verticies, face_ind """# you may want to check out the trimesh mehtods here:)
    vertices, face_ind = mesh.sample(n, return_index=True)
    normals = mesh.face_normals[face_ind]
    return vertices, normals

def load_grasp_data(object_name):
    """Loads grasp data from the provided NPZ files. It returns three arrays:

    Parameters
    ----------
    object_name (String): type of object

    Returns
    -------
    vertices (np.ndarray): nx3 matrix, coordinates of the n surface points
    normals (np.ndarray): nx3 matrix, normals of the n surface points

    grasp_vertices (np.ndarray): 5x2x3 matrix. For each of the 5 grasps,
            this stores a pair of (x, y, z) locations that are the contact points
            of the grasp.
    normals (np.ndarray): 5x2x3 matrix. For each grasp, stores the normal
            vector to the mesh at the two contact points. Remember that the normal
            vectors to a closed mesh always point OUTWARD from the mesh.
    tip_poses (np.ndarray): 5x4x4 matrix. For each of the five grasps, this
            stores the 4x4 rigid transform of the reference frame of the gripper
            tip before the gripper is closed in order to grasp the object.
    results (np.ndarray): 5x5 matrix. Stores the result of five trials for
            each of the five grasps. Entry (i, j) is a 1 if the jth trial of the
            ith grasp was successful, 0 otherwise.
    """
    data = np.load('grasp_data/{}.npz'.format(object_name))
    return data['grasp_vertices'], data['normals'], data['tip_poses'], data['results']

def load_mesh(object_name):
    mesh = trimesh.load_mesh('objects/{}.obj'.format(object_name))
    mesh.fix_normals()
    return mesh

def run(obj_name, metric_name, index):
    vertices, normals, poses, results = load_grasp_data(obj_name)
    mesh = load_mesh(obj_name)
    info = custom_grasp_planner(mesh, vertices[index], metric_name)
    return info

def print_metric_values():
    objs = ["pawn", "nozzle"]
    metrics = ["robust", "gravity"]
    for obj in objs:
        vertices, normals, poses, results = load_grasp_data(obj)
        mesh = load_mesh(obj)
        for metric in metrics:
            print(f"Metric: {metric} for object {obj}")  
            grasps = [custom_grasp_planner(mesh, v, "robust") for v in vertices]
            values = [grasp[2] if not isinstance(grasp, type(None)) else None for grasp in grasps]
            print(values)

def example():
    obj = 'nozzle' # should be 'pawn' or 'nozzle'.
    vertices, normals, poses, results = load_grasp_data(obj)
    mesh = load_mesh(obj)
    # for v, p in zip(vertices, poses):
    #     visualize_grasp(mesh, v, p)

    # you may or may not find the code below helpful:) i was supposed to delete this from the starter code but i didn't so enjoy

    v_and_p = [custom_grasp_planner(mesh, v, "robust") for v in vertices]
    idx = 0
    for grasp in v_and_p:
        if grasp is not None:
            p = np.array(grasp[0])
            v = np.array(grasp[1])
            if p.any() and v.any():
                print(idx)
                visualize_grasp(mesh, v, p)
            else:
                print("None")
        idx += 1
    vert1, norm1 = randomly_sample_from_mesh(mesh, 3)
    while len(vert1) < 3:
        vert1, norm1 = randomly_sample_from_mesh(mesh, 3)
    vert2 = np.array([find_intersections(mesh, vert1[i], vert1[i] - 5 * norm1[i])[0][-1] for i in range(len(vert1))]) #randomly_sample_from_mesh(mesh, 3)
    vertices = list(zip(vert1, vert2))

def main():
    """ Example for interacting with the codebase. Loads data and
    visualizes each grasp against the corresponding mesh of the given
    object.
    """
    # example()
    print_metric_values()

if __name__ == '__main__':
    main()

