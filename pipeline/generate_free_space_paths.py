#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import cvxpy as cp
import fastpathplanning as fpp
import fastpathplanning.polygonal
import fastpathplanning.smooth
import h5py
import numpy as np
import os
import scipy.sparse.csgraph
import scipy.special
import signal
import spear
import threading
import time


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
args = parser.parse_args()

np.random.seed(0)

# Factor to convert the scene's Unreal centimeters to meters. We plan in meters rather than centimeters: the
# polygonal phase repeatedly solves a continuous min-distance cone program and detects kinks with absolute
# tolerances, and the smooth phase solves a jerk-minimizing QP, and both are better conditioned (and the polygonal
# phase's fixed tolerances far better matched) when coordinates are O(1-10) instead of O(100-1000). We scale the box
# corners to meters up front and scale the resulting paths back to centimeters before saving.
centimeters_to_meters = 0.01

# Number of smooth paths to plan.
num_paths = 2

# Number of waypoints each path passes through, including its begin and end points. We plan a single joint smooth
# trajectory through all of them (see plan_through_waypoints), not a sequence of independent paths.
num_waypoints = 10

# Cost weights passed to the smooth phase. Entry i weights the integral of the squared (i + 1)-th derivative of the
# path along its duration, so [0, 0, 1] penalizes only the third derivative (jerk), yielding minimum-jerk paths.
path_cost_weights = [0.0, 0.0, 1.0]

# Number of points at which we sample each planned path when saving it.
num_path_samples = 1000

# Number of mutually-reachable waypoint sequences to sample as candidates. We plan paths through the most
# complicated of these first (see process_scene), so this should be comfortably larger than num_paths.
num_candidate_paths = 100

# Time limit (in seconds) on each smooth-phase solve, so a pathological waypoint sequence cannot hang the solver. A
# solve that exceeds it returns a non-optimal status, which we treat like any other planning failure and skip.
solver_time_limit = 30.0

# Wall-clock time limit (in seconds) for planning a single path, covering its polygonal phase as well as its smooth
# solve. fastpathplanning's polygonal phase (iterative_planner) occasionally fails to converge quickly for a given
# geometry, looping while it re-solves and grows a continuous min-distance program; this limit (enforced with
# SIGALRM, which can interrupt that Python-level loop) abandons such a waypoint sequence and moves on.
planning_time_limit = 60

# Interval (in seconds) at which a background thread prints which planning step is currently running, so a slow step
# is visible rather than appearing to hang.
heartbeat_seconds = 5.0

# The planning step currently in progress, updated by the main thread and printed by the heartbeat thread.
planning_step = {"description": None, "start_time": 0.0}


def process_scene():

    # Read the candidate free-space bounding boxes and the indices of the boxes selected to form the connected
    # network (the phase-1 covering boxes plus the phase-2 overlapping boxes) computed by
    # generate_free_space_bounding_boxes.py.
    free_space_bounding_boxes_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_bounding_boxes", "free_space_bounding_boxes.h5"))
    assert os.path.exists(free_space_bounding_boxes_file)
    spear.log("Reading free-space bounding boxes file: ", free_space_bounding_boxes_file)
    with h5py.File(free_space_bounding_boxes_file, "r") as f:
        box_mins = f["box_mins"][:]
        box_maxs = f["box_maxs"][:]
        covering_indices = f["box_covering_indices"][:]
        overlapping_indices = f["box_overlapping_indices"][:]

    # Plan in meters (see centimeters_to_meters). Everything downstream -- the network, the reachability graph, the
    # sampled waypoints, and the planned paths -- is therefore in meters, until we scale the paths back when saving.
    box_mins = box_mins*centimeters_to_meters
    box_maxs = box_maxs*centimeters_to_meters

    # The path-planning network is the union of the phase-1 and phase-2 selected boxes. fastpathplanning treats each
    # box as a safe region and plans through their intersection graph, so we pass the boxes' lower and upper corners
    # directly as its L and U.
    network_indices = np.union1d(covering_indices, overlapping_indices)
    assert network_indices.shape[0] > 0
    network_box_mins = box_mins[network_indices]
    network_box_maxs = box_maxs[network_indices]

    # fastpathplanning plans through the graph of box intersections, and cannot plan within a box that intersects no
    # other box, because such a box has no node in its line graph (the graph whose nodes are box intersections). So
    # that an isolated box stays reachable from itself, we add a duplicate of each isolated box before planning: a box
    # and its duplicate fully overlap, which gives the box a line-graph node to plan within. Two boxes are connected
    # in this graph if they intersect (closed intersection, matching fastpathplanning's Box.intersects).
    network_box_intersects = np.logical_and(
        np.all(network_box_mins[:,np.newaxis,:] <= network_box_maxs[np.newaxis,:,:], axis=2),
        np.all(network_box_mins[np.newaxis,:,:] <= network_box_maxs[:,np.newaxis,:], axis=2))
    np.fill_diagonal(network_box_intersects, False)
    isolated = np.logical_not(network_box_intersects.any(axis=1))
    L = np.concatenate([network_box_mins, network_box_mins[isolated]])
    U = np.concatenate([network_box_maxs, network_box_maxs[isolated]])
    safe_set = fpp.SafeSet(L=L, U=U, verbose=False)

    # Two points are mutually reachable exactly when the boxes containing them lie in the same connected component of
    # the box intersection graph. We compute the components of the (duplicate-augmented) box set once, so we can
    # reject sampled waypoint sequences that are not mutually reachable without invoking the (much more expensive)
    # planner, which does not itself fail gracefully when the points are unreachable. Every box now intersects at
    # least one other box, so no box is isolated and every in-free-space point is plannable.
    box_intersects = np.logical_and(
        np.all(L[:,np.newaxis,:] <= U[np.newaxis,:,:], axis=2),
        np.all(L[np.newaxis,:,:] <= U[:,np.newaxis,:], axis=2))
    num_components, box_component = scipy.sparse.csgraph.connected_components(box_intersects, directed=False)

    # Precompute per-box volumes and per-component total box volumes, used to sample waypoints (see
    # sample_reachable_waypoints). Sampling all of a sequence's waypoints within one connected component guarantees
    # they are in free space and mutually reachable, and scales to many waypoints (unlike rejection-sampling points
    # from the whole AABB, whose acceptance decays geometrically in the number of waypoints).
    box_volumes = np.prod(U - L, axis=1)
    component_volumes = np.array([ box_volumes[box_component == component].sum() for component in range(num_components) ])

    # Sample a collection of mutually-reachable waypoint sequences, then plan paths through the most complicated
    # sequences first. We measure complexity by the total number of hops between consecutive waypoints' boxes in the
    # box intersection graph (how many boxes the whole path must traverse), breaking ties by the total Euclidean
    # length of the waypoint polyline. This favors long paths that wind through many boxes over trivial ones whose
    # waypoints all lie in the same box.
    box_distances = scipy.sparse.csgraph.shortest_path(box_intersects, directed=False, unweighted=True)
    candidate_paths = []
    for _ in range(num_candidate_paths):
        waypoints = sample_reachable_waypoints(L=L, U=U, box_component=box_component, box_volumes=box_volumes, component_volumes=component_volumes)
        connectivity = sum(get_box_connectivity(p_init=waypoints[i], p_term=waypoints[i + 1], L=L, U=U, box_distances=box_distances) for i in range(num_waypoints - 1))
        distance = sum(np.linalg.norm(waypoints[i + 1] - waypoints[i]) for i in range(num_waypoints - 1))
        candidate_paths.append((connectivity, distance, waypoints))
    candidate_paths.sort(key=lambda candidate: (candidate[0], candidate[1]), reverse=True)

    # Plan num_paths paths, taking candidate waypoint sequences in order of decreasing complexity. Two things can go
    # wrong on an individual candidate: the smooth-phase solver can fail numerically (raising an exception), and
    # fastpathplanning's polygonal phase can fail to converge quickly for some geometry and run for a very long time.
    # We bound each candidate with a wall-clock timeout (SIGALRM, which interrupts the polygonal phase's Python-level
    # loop) and skip any candidate that times out or raises, moving on to the next one. A background thread prints
    # which planning step is in progress every heartbeat_seconds so a slow step is visible rather than looking hung.
    signal.signal(signal.SIGALRM, raise_planning_timeout)
    heartbeat_stop = threading.Event()
    heartbeat_thread = threading.Thread(target=print_planning_heartbeats, kwargs={"stop_event": heartbeat_stop}, daemon=True)
    heartbeat_thread.start()

    path_points = []
    path_point_times = []
    path_waypoints = []
    path_waypoint_times = []
    candidate_index = 0
    try:
        while len(path_points) < num_paths:
            assert candidate_index < len(candidate_paths)
            connectivity, distance, waypoints = candidate_paths[candidate_index]
            candidate_index += 1
            spear.log(f"Planning path {len(path_points)} through {num_waypoints} waypoints (total box hops {int(connectivity)}, total distance {distance:.1f}).")
            signal.alarm(planning_time_limit)
            try:
                path, waypoint_times = plan_through_waypoints(safe_set=safe_set, waypoints=waypoints, cost_weights=path_cost_weights)
            except Exception as e:
                spear.log(f"Planning failed for this waypoint sequence ({type(e).__name__}: {e}), so skipping it.")
                continue
            finally:
                signal.alarm(0)
            sample_times = np.linspace(path.a, path.b, num_path_samples)
            path_points.append(np.array([ path(sample_time) for sample_time in sample_times ]))
            path_point_times.append((sample_times - path.a)/(path.b - path.a))
            path_waypoints.append(waypoints)
            path_waypoint_times.append(waypoint_times)
    finally:
        heartbeat_stop.set()

    # Scale the planned paths and waypoints back from meters to centimeters to match the rest of the pipeline. The
    # path-point and waypoint times share one parametrization, normalized to [0, 1], and so are unitless (used to
    # color the path and its waypoints consistently).
    path_points = np.array(path_points)/centimeters_to_meters
    path_point_times = np.array(path_point_times)
    path_waypoints = np.array(path_waypoints)/centimeters_to_meters
    path_waypoint_times = np.array(path_waypoint_times)

    # Save each path and its waypoints in the same format: a set of points and the normalized time in [0, 1] at which
    # the path passes through each. The path points are sampled uniformly in time (so their times are uniform, and
    # the first and last are the begin and end points); the waypoints are the points the path was planned through (so
    # their times are generally not uniform). The times let the path and its waypoints be colored consistently.
    free_space_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_paths"))
    free_space_paths_file = os.path.realpath(os.path.join(free_space_dir, "free_space_paths.h5"))
    spear.log("Writing free-space paths file: ", free_space_paths_file)
    os.makedirs(free_space_dir, exist_ok=True)
    with h5py.File(free_space_paths_file, "w") as f:
        f.create_dataset("path_points", data=path_points)
        f.create_dataset("path_point_times", data=path_point_times)
        f.create_dataset("path_waypoints", data=path_waypoints)
        f.create_dataset("path_waypoint_times", data=path_waypoint_times)

    spear.log("Done.")

def set_planning_step(description):

    # Record the planning step currently in progress (for the heartbeat thread to report).
    planning_step["description"] = description
    planning_step["start_time"] = time.time()

def print_planning_heartbeats(stop_event):

    # Print the planning step currently in progress every heartbeat_seconds, until stop_event is set, so that a slow
    # step (notably fastpathplanning's polygonal phase) is visible rather than appearing to hang.
    while not stop_event.wait(heartbeat_seconds):
        description = planning_step["description"]
        if description is not None:
            spear.log(f"    ...still running: {description} ({time.time() - planning_step['start_time']:.0f}s elapsed)")

def raise_planning_timeout(signum, frame):

    # SIGALRM handler that aborts a path that has exceeded planning_time_limit (see process_scene).
    raise TimeoutError(f"planning exceeded planning_time_limit ({planning_time_limit}s)")

def sample_reachable_waypoints(L, U, box_component, box_volumes, component_volumes):

    # Sample num_waypoints points that are all in free space and mutually reachable, by first choosing a connected
    # component of the box intersection graph (weighted by its total box volume) and then sampling every waypoint
    # uniformly from the union of that component's boxes. Because all the waypoints come from one component, the whole
    # sequence is reachable by construction. This scales to many waypoints, unlike rejection-sampling points from the
    # whole scene AABB, whose acceptance rate decays geometrically in the number of waypoints.
    component = np.random.choice(component_volumes.shape[0], p=component_volumes/component_volumes.sum())
    box_indices = np.flatnonzero(box_component == component)
    return np.array([ sample_point_in_boxes(L=L, U=U, box_indices=box_indices, box_volumes=box_volumes) for _ in range(num_waypoints) ])

def sample_point_in_boxes(L, U, box_indices, box_volumes):

    # Sample a point uniformly from the union of the given boxes: pick one of them with probability proportional to
    # its volume, sample uniformly inside it, and accept with probability one over the number of those boxes that
    # contain the point. The acceptance step corrects for the boxes overlapping (a point lying in k of them would
    # otherwise be k times as likely), so the returned point is distributed uniformly over the union.
    box_probabilities = box_volumes[box_indices]/box_volumes[box_indices].sum()
    while True:
        box_index = np.random.choice(box_indices, p=box_probabilities)
        point = np.random.uniform(L[box_index], U[box_index])
        num_containing_boxes = np.count_nonzero(np.all(np.logical_and(L[box_indices] <= point, point <= U[box_indices]), axis=1))
        if np.random.uniform() < 1.0/num_containing_boxes:
            return point

def get_box_connectivity(p_init, p_term, L, U, box_distances):

    # The number of hops in the box intersection graph between the begin and end boxes (the boxes containing the
    # points), minimized over the boxes containing each point. Zero when both points lie in the same box, and larger
    # when a path between them must traverse more boxes. The points are assumed reachable, so this is finite.
    init_boxes = np.flatnonzero(np.all(np.logical_and(L <= p_init, p_init <= U), axis=1))
    term_boxes = np.flatnonzero(np.all(np.logical_and(L <= p_term, p_term <= U), axis=1))
    return box_distances[np.ix_(init_boxes, term_boxes)].min()

def plan_through_waypoints(safe_set, waypoints, cost_weights):

    # Plan a single smooth minimum-cost trajectory through the waypoints jointly. fastpathplanning's smooth phase
    # already optimizes one composite Bezier curve through a sequence of boxes, enforcing box containment per segment
    # and full continuity (position and every derivative up to the cost order) at each box transition. We extend it to
    # multiple waypoints by concatenating the box sequences fastpathplanning's polygonal phase finds for each leg
    # (between consecutive waypoints) into one long sequence, and pinning the knot at the end of each leg to the
    # corresponding waypoint. The waypoints' derivatives stay free and are optimized jointly, so the trajectory is
    # smooth through the waypoints rather than stopping at them.
    box_sequence = []
    durations = []
    waypoint_box_indices = {}
    for waypoint_index in range(waypoints.shape[0] - 1):
        p_init = waypoints[waypoint_index]
        p_term = waypoints[waypoint_index + 1]

        # fastpathplanning's polygonal phase: find a sequence of boxes connecting the two waypoints and a polygonal
        # trajectory through them, whose segment lengths we use to initialize the smooth phase's per-box durations.
        set_planning_step(description=f"fastpathplanning polygonal phase for leg {waypoint_index}")
        discrete_planner, _ = safe_set.G.shortest_path(goal=p_term)
        leg_box_sequence, _, _ = discrete_planner(start=p_init)

        # This fix-up logic is required to work around a bug in fastpathplanning. Specifically, discrete_planner can
        # in some cases append an extra bounding box to leg_box_sequence that doesn't contain p_term. This is a bug.
        # The smooth phase pins p_term to the end of the last box in leg_box_sequence, so a trailing box that doesn't
        # contain p_term would make the optimization problem infeasible. So we must detect and remove this extra
        # bounding box if it is present.
        if not (np.all(safe_set.B.boxes[leg_box_sequence[-1]].l <= p_term) and np.all(p_term <= safe_set.B.boxes[leg_box_sequence[-1]].u)):
            leg_box_sequence = leg_box_sequence[:-1]

        assert np.all(safe_set.B.boxes[leg_box_sequence[0]].l <= p_init) and np.all(p_init <= safe_set.B.boxes[leg_box_sequence[0]].u)
        assert np.all(safe_set.B.boxes[leg_box_sequence[-1]].l <= p_term) and np.all(p_term <= safe_set.B.boxes[leg_box_sequence[-1]].u)

        # iterative_planner's solve_min_distance cannot handle a single-box sequence: zip(boxes[:-1], boxes[1:])
        # produces no intersection constraints, leaving l and u as shape-(0,) arrays, which CVXPY can't broadcast
        # against shape-(0,3) trajectory variables. Since both p_init and p_term are in the single box (guaranteed
        # by the asserts above), the straight-line trajectory is valid and optimal.
        if len(leg_box_sequence) == 1:
            leg_trajectory = np.array([p_init, p_term])
        else:
            leg_box_sequence, leg_trajectory, _, _ = fpp.polygonal.iterative_planner(B=safe_set.B, start=p_init, goal=p_term, box_seq=leg_box_sequence, verbose=True)

        box_sequence.extend(leg_box_sequence)
        durations.extend(np.linalg.norm(leg_trajectory[1:] - leg_trajectory[:-1], axis=1))

        # Pin the knot at the end of this leg to the waypoint between this leg and the next. The first and last
        # waypoints are pinned by the begin/end boundary conditions instead.
        if waypoint_index < waypoints.shape[0] - 2:
            waypoint_box_indices[len(box_sequence) - 1] = p_term

    # Use the polygonal segment lengths directly as the per-box durations, so the total duration is the polygonal
    # path length (in meters). The optimal spatial path is invariant to the overall time scale here (the box
    # constraints are on position only and we set no derivative boundary conditions), so matching the time scale to
    # the spatial scale keeps the jerk-cost quadratic form well-conditioned rather than letting a unit total duration
    # inflate the cost by many orders of magnitude.
    durations = np.array(durations)
    L = np.array([ safe_set.B.boxes[box_index].l for box_index in box_sequence ])
    U = np.array([ safe_set.B.boxes[box_index].u for box_index in box_sequence ])

    # cost_weights[i] weights the integral of the squared (i + 1)-th derivative, matching fastpathplanning's plan.
    alpha = { i + 1: cost_weight for i, cost_weight in enumerate(cost_weights) }
    initial = { 0: waypoints[0] }
    final = { 0: waypoints[-1] }
    set_planning_step(description=f"smooth optimization over {len(box_sequence)} boxes")
    path = optimize_bezier(L=L, U=U, durations=durations, alpha=alpha, initial=initial, final=final, waypoints=waypoint_box_indices)

    # The normalized time (in [0, 1]) at which the path passes through each waypoint. A waypoint pinned at the end of
    # box k is reached at the cumulative duration up to box k, and the first and last waypoints are at 0 and 1. The
    # waypoints are generally not evenly spaced in time, so a visualizer needs these to color each waypoint to match
    # the path's color there (the path is colored by this same normalized time) rather than spacing them evenly.
    cumulative_durations = np.cumsum(durations)
    waypoint_times = np.array([0.0] + [cumulative_durations[box_index] for box_index in waypoint_box_indices] + [cumulative_durations[-1]])
    waypoint_times = waypoint_times/cumulative_durations[-1]

    return path, waypoint_times

def optimize_bezier(L, U, durations, alpha, initial, final, waypoints):

    # A faithful re-implementation of fastpathplanning.smooth.optimize_bezier (a single solve, without its optional
    # retiming refinement), with one addition: the waypoints argument pins the end control point of a given box in the
    # sequence to a given position, which is how we constrain the trajectory to pass through an interior waypoint. The
    # control points of each box's Bezier curve and of its derivatives are decision variables; we constrain box
    # containment, the Bezier derivative relations, continuity across box transitions, the begin/end boundary
    # conditions, and the interior waypoints, and minimize the weighted sum of squared-derivative integrals.
    num_boxes, dimension = L.shape
    max_derivative = max(alpha)
    num_points = (max_derivative + 1)*2

    points = {}
    for box_index in range(num_boxes):
        points[box_index] = { derivative: cp.Variable((num_points - derivative, dimension)) for derivative in range(max_derivative + 1) }

    constraints = []

    # Begin and end boundary conditions (the position, and any specified derivatives, at the start of the first box
    # and the end of the last box).
    for derivative, value in initial.items():
        constraints.append(points[0][derivative][0] == value)
    for derivative, value in final.items():
        constraints.append(points[num_boxes - 1][derivative][-1] == value)

    # Interior waypoints: pin the end control point of the given box's position curve to the waypoint.
    for box_index, value in waypoints.items():
        constraints.append(points[box_index][0][-1] == value)

    cost = 0
    for box_index in range(num_boxes):

        # Box containment of the position control points.
        constraints.append(points[box_index][0] >= np.array([L[box_index]]*num_points))
        constraints.append(points[box_index][0] <= np.array([U[box_index]]*num_points))

        # Bezier derivative relations: each derivative's control points are the scaled finite differences of the
        # previous derivative's control points.
        for derivative in range(max_derivative):
            h = num_points - derivative - 1
            constraints.append(points[box_index][derivative][1:] - points[box_index][derivative][:-1] == durations[box_index]/h*points[box_index][derivative + 1])

        # Continuity of the position and every derivative across the transition to the next box.
        if box_index < num_boxes - 1:
            for derivative in range(max_derivative + 1):
                constraints.append(points[box_index][derivative][-1] == points[box_index + 1][derivative][0])

        # Cost: the weighted sum of the integrals of the squared derivatives along this box's curve. The Gram matrix
        # gives the integral of the squared Bezier curve as a quadratic form in its control points.
        for derivative, weight in alpha.items():
            h = num_points - 1 - derivative
            gram = np.zeros((h + 1, h + 1))
            for m in range(h + 1):
                for n in range(h + 1):
                    gram[m, n] = scipy.special.binom(h, m)*scipy.special.binom(h, n)/scipy.special.binom(2*h, m + n)
            gram = np.kron(gram*durations[box_index]/(2*h + 1), np.eye(dimension))
            cost += weight*cp.quad_form(cp.vec(points[box_index][derivative], order="C"), gram)

    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver="CLARABEL", time_limit=solver_time_limit)
    if problem.status not in ("optimal", "optimal_inaccurate"):
        raise RuntimeError(f"Bezier optimization did not solve (status: {problem.status}).")

    # Reconstruct the composite Bezier curve from the optimized control points, with each box's curve spanning its
    # duration.
    beziers = []
    start_time = 0.0
    for box_index in range(num_boxes):
        end_time = start_time + durations[box_index]
        beziers.append(fpp.smooth.BezierCurve(points=points[box_index][0].value, a=start_time, b=end_time))
        start_time = end_time
    return fpp.smooth.CompositeBezierCurve(beziers=beziers)


if __name__ == "__main__":
    process_scene()
