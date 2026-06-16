#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import json
import numpy as np
import os
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
args = parser.parse_args()

np.random.seed(0)

# Component classes that export the local-space bounds we use to build oriented bounding boxes.
static_mesh_component_classes = ["StaticMeshComponent"]

# Distance in Unreal world units (cm) by which every obstacle is expanded so the camera frustum's near face is
# guaranteed to lie in free space. See generate_free_space_points.py for the derivation.
expand_bounding_box_eps = 20.0

# Number of candidate free-space bounding boxes to generate. Each is an axis-aligned box grown greedily outward
# from a free-space point until it collides with a component bounding box.
num_candidate_bounding_boxes = 500

# Cross-product separating axes shorter than this (in world units) arise from near-parallel box edges and are
# skipped when computing how far a candidate free-space bounding box's face can advance before contact.
cross_axis_tolerance = 0.000001

# Each round of growth advances one face by at most this fraction of the box's current extent along that face's
# axis, rather than all the way to contact. Capping the advance lets perpendicular faces grow before any single
# face commits the box to a long thin shape, which produces substantially larger boxes in cluttered regions.
bounding_box_growth_step_fraction = 0.25

# A face stops growing once its remaining free distance falls below this many world units. This also keeps
# floating-point residuals near contact from generating arbitrarily many vanishingly-small growth steps.
bounding_box_growth_tolerance = 0.001

# Having generated the candidate boxes above, we greedily select a small subset of them that covers the free-space
# points as thoroughly as possible. A free-space point counts as covered by a candidate box if it lies inside the
# box, expanded by this many world units so points lying exactly on a box face still count as covered.
coverage_tolerance = 0.01

# Stop selecting boxes once we have selected this many (the budget constraint)...
max_num_covering_bounding_boxes = 100

# ...or once the selected boxes cover this fraction of all coverable points (the quota constraint), whichever comes
# first. A point is coverable if at least one candidate box covers it, and covering every coverable point counts as
# 100% coverage. Hitting the budget first versus the quota first gives a different lazy-greedy approximation
# guarantee, but either is fine for our purposes.
coverage_quota_fraction = 0.95

# Having selected the phase-1 boxes above, we greedily select additional boxes to cover the phase-1 boxes'
# footprints (forming a connected network). A candidate box counts as overlapping a phase-1 box, and so as
# contributing to its footprint, only if the two boxes' intersection is at least this fraction of the phase-1 box's
# length along every axis, so that the shared region has room for a trajectory to pass through.
overlap_fraction = 0.1

# Stop selecting footprint boxes once we have selected this many (the budget constraint)...
max_num_overlapping_bounding_boxes = 100

# ...or once the selected boxes cover this fraction of all coverable footprint (the quota constraint), whichever
# comes first, analogously to coverage_quota_fraction above.
overlapping_quota_fraction = 0.95


def process_scene():

    bounding_boxes = get_component_bounding_boxes_for_scene()
    assert len(bounding_boxes) > 0

    # Read the free-space points and the mutual-visibility graph computed by generate_free_space_points.py and
    # generate_free_space_visibility_graph.py.
    free_space_points_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_points", "free_space_points.h5"))
    visibility_graph_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_visibility_graph", "free_space_visibility_graph.h5"))
    assert os.path.exists(free_space_points_file)
    assert os.path.exists(visibility_graph_file)
    spear.log("Reading free-space points file: ", free_space_points_file)
    spear.log("Reading visibility graph file: ", visibility_graph_file)
    with h5py.File(free_space_points_file, "r") as f:
        free_space_points = f["scene_points"][:]
    with h5py.File(visibility_graph_file, "r") as f:
        edges = f["visibility_graph_edges"][:]

    # Grow a collection of overlapping free-space bounding boxes, each seeded from a maximal clique in the
    # visibility graph and grown until it collides with a component bounding box. We clamp growth to the scene's
    # overall AABB so boxes never extend beyond the geometry.
    corners = np.array([ bounding_box["corners_world"] for bounding_box in bounding_boxes ]).reshape(-1, 3)
    scene_min = corners.min(axis=0)
    scene_max = corners.max(axis=0)
    free_space_box_mins, free_space_box_maxs = compute_free_space_bounding_boxes(
        free_space_points=free_space_points, edges=edges, bounding_boxes=bounding_boxes, scene_min=scene_min, scene_max=scene_max)
    spear.log(f"Generated {free_space_box_mins.shape[0]} candidate free-space bounding boxes.")

    # Phase 1: greedily select a small subset of the candidate boxes that covers the free-space points as thoroughly
    # as possible, by building the point-by-box coverage indicator matrix and running lazy greedy submodular
    # maximization.
    coverage_indicator_matrix = compute_coverage_indicator_matrix(free_space_points=free_space_points, box_mins=free_space_box_mins, box_maxs=free_space_box_maxs)
    covering_indices = select_covering_bounding_boxes(coverage_indicator_matrix=coverage_indicator_matrix)

    # Phase 2: greedily select additional candidate boxes to form a connected network, by maximizing the covered
    # footprint of the phase-1 boxes (the space outside each phase-1 box that belongs to candidate boxes overlapping
    # it). This is again a positive, monotone, submodular objective, so we use the same lazy greedy algorithm. It
    # reuses the phase-1 coverage indicator matrix plus a graph of which candidate boxes overlap which.
    overlap_graph = compute_overlap_graph(box_mins=free_space_box_mins, box_maxs=free_space_box_maxs)
    overlapping_indices = select_overlapping_bounding_boxes(coverage_indicator_matrix=coverage_indicator_matrix, overlap_graph=overlap_graph, anchor_indices=covering_indices)

    # Save the candidate free-space bounding boxes as axis-aligned min and max corners, along with the indices of
    # the subset selected to cover the free-space points (phase 1) and the additional subset selected to cover the
    # phase-1 boxes' footprints (phase 2).
    free_space_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_bounding_boxes"))
    free_space_bounding_boxes_file = os.path.realpath(os.path.join(free_space_dir, "free_space_bounding_boxes.h5"))
    spear.log("Writing free-space bounding boxes file: ", free_space_bounding_boxes_file)
    os.makedirs(free_space_dir, exist_ok=True)
    with h5py.File(free_space_bounding_boxes_file, "w") as f:
        f.create_dataset("box_mins", data=free_space_box_mins)
        f.create_dataset("box_maxs", data=free_space_box_maxs)
        f.create_dataset("box_covering_indices", data=covering_indices)
        f.create_dataset("box_overlapping_indices", data=overlapping_indices)

    spear.log("Done.")

def get_component_bounding_boxes_for_scene():

    unreal_metadata_dir = os.path.realpath(os.path.join(args.pipeline_dir, "unreal_metadata"))
    actors_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "scene.json"))
    spear.log("Reading JSON file: ", actors_json_file)
    assert os.path.exists(unreal_metadata_dir)
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    # We only export geometry for actors for which relevant_for_level_bounds is True, so we apply the same
    # filter here, and ignore actors with no root component.
    bounding_boxes = []
    for actor_name, actor_desc in actors_json.items():
        if actor_desc["root_component"] is None or not actor_desc["editor_properties"]["relevant_for_level_bounds"]:
            continue
        spear.log("Processing actor: ", actor_name)
        actor_bounding_boxes = get_component_bounding_boxes(component_desc=actor_desc["root_component"], log_prefix_str="    ")
        bounding_boxes.extend(actor_bounding_boxes)

    return bounding_boxes

def get_component_bounding_boxes(component_desc, log_prefix_str):

    spear.log(f"{log_prefix_str}Processing component: {component_desc['stable_name']}")

    # We only build a bounding box for StaticMeshComponents that have a static mesh assigned.
    bounding_boxes = []
    if component_desc["class"] in static_mesh_component_classes and component_desc["editor_properties"]["static_mesh"] is not None:
        bounding_box = spear.pipeline.get_bounding_box(component_desc=component_desc)
        bounding_box = spear.pipeline.expand_bounding_box(bounding_box=bounding_box, eps=expand_bounding_box_eps)
        bounding_boxes.append(bounding_box)

    # Recurse for each child component.
    if component_desc["children_components"] is not None:
        for child_component_desc in component_desc["children_components"].values():
            child_bounding_boxes = get_component_bounding_boxes(component_desc=child_component_desc, log_prefix_str=f"{log_prefix_str}    ")
            bounding_boxes.extend(child_bounding_boxes)

    return bounding_boxes

def compute_free_space_bounding_boxes(free_space_points, edges, bounding_boxes, scene_min, scene_max):

    num_points = free_space_points.shape[0]

    if num_points == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))

    # Build an undirected adjacency matrix for the visibility graph.
    adjacency = np.zeros((num_points, num_points), dtype=bool)
    adjacency[edges[:,0], edges[:,1]] = True
    adjacency[edges[:,1], edges[:,0]] = True

    box_mins = []
    box_maxs = []
    for _ in range(num_candidate_bounding_boxes):

        # Choose a random starting node and grow a maximal clique around it in the visibility graph.
        start_node = np.random.randint(num_points)
        clique = find_maximal_clique(adjacency=adjacency, start_node=start_node)

        # Find the clique node closest to the clique's centroid, then grow an axis-aligned box outward from it.
        clique_points = free_space_points[clique]
        centroid = clique_points.mean(axis=0)
        center_node = clique[np.argmin(np.linalg.norm(clique_points - centroid, axis=1))]

        box_min, box_max = grow_axis_aligned_bounding_box(center=free_space_points[center_node], scene_min=scene_min, scene_max=scene_max, bounding_boxes=bounding_boxes)
        box_mins.append(box_min)
        box_maxs.append(box_max)

    return np.array(box_mins), np.array(box_maxs)

def find_maximal_clique(adjacency, start_node):

    # Greedily grow a clique from the starting node: repeatedly add a random node adjacent to every node already
    # in the clique, intersecting the candidate set with each added node's neighbors, until no candidates remain.
    clique = [start_node]
    candidates = adjacency[start_node].copy()
    while candidates.any():
        node = np.random.choice(np.flatnonzero(candidates))
        clique.append(node)
        candidates = np.logical_and(candidates, adjacency[node])
    return clique

def grow_axis_aligned_bounding_box(center, scene_min, scene_max, bounding_boxes):

    # Stack the component bounding boxes' precomputed world-space centers, axes, and half-extents once here (they
    # are all already computed in get_bounding_box), so the face-advance computations in the growth loop below can
    # reuse them without restacking on every call.
    obb_centers = np.array([ bounding_box["center_world"] for bounding_box in bounding_boxes ])
    obb_axes = np.array([ bounding_box["axes_world"] for bounding_box in bounding_boxes ])
    obb_half_extents = np.array([ bounding_box["half_extent"] for bounding_box in bounding_boxes ])

    # Seed the box from a non-degenerate cube rather than a degenerate point, so that a single uniform
    # face-growing routine suffices (growing from a point would otherwise need special-case point-growing then
    # edge-growing then face-growing logic). The largest collision-free sphere centered at the seed point has a
    # radius equal to the distance to the nearest component bounding box; the axis-aligned cube inscribed in that
    # sphere (half-extent radius/sqrt(3)) is therefore a non-degenerate, collision-free starting box.
    radius = distance_to_nearest_bounding_box(point=center, obb_centers=obb_centers, obb_axes=obb_axes, obb_half_extents=obb_half_extents)
    box_min = np.maximum(center - radius/np.sqrt(3.0), scene_min)
    box_max = np.minimum(center + radius/np.sqrt(3.0), scene_max)

    # Greedily expand the box one face at a time. Each round we compute, in closed form, how far every face can
    # advance before it first contacts a component bounding box or the scene bound, and we advance the face whose
    # advance adds the most volume (the advance distance times the box's cross-section perpendicular to its axis).
    # Rather than advancing the chosen face all the way to contact, we cap the advance at a fraction of the box's
    # current extent along that axis: committing a single face fully to contact stretches the box into a corridor
    # whose flanks are then against obstacles, freezing every perpendicular face, so we instead interleave growth
    # across faces. A face whose free distance has fallen below bounding_box_growth_tolerance can no longer grow
    # (growing the box only ever reduces a face's remaining free distance), so growth terminates once no face can
    # advance. Each face is an (axis, direction) pair; direction -1 advances box_min outward, +1 advances box_max.
    faces = [ (0, -1), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1) ]

    box_is_growing = True
    while box_is_growing:
        extent = box_max - box_min

        best_face = None
        best_advance = 0.0
        best_volume_gain = 0.0
        for axis, direction in faces:
            distance = face_free_distance(box_min=box_min, box_max=box_max, axis=axis, direction=direction, scene_min=scene_min, scene_max=scene_max, obb_centers=obb_centers, obb_axes=obb_axes, obb_half_extents=obb_half_extents)
            if distance <= bounding_box_growth_tolerance:
                continue
            advance = min(distance, bounding_box_growth_step_fraction*extent[axis])
            volume_gain = advance*np.prod(np.delete(extent, axis))
            if volume_gain > best_volume_gain:
                best_face = (axis, direction)
                best_advance = advance
                best_volume_gain = volume_gain

        # No face can advance any further, so the box has reached its final size.
        box_is_growing = best_face is not None
        if box_is_growing:
            axis, direction = best_face
            if direction < 0:
                box_min[axis] -= best_advance
            else:
                box_max[axis] += best_advance

    return box_min, box_max

def distance_to_nearest_bounding_box(point, obb_centers, obb_axes, obb_half_extents):

    # Distance from the point to the nearest component bounding box. We project the point into each box's frame
    # (its coordinate along box axis j is the dot product of (point - center) with that axis), clamp it to the
    # box's extent, and measure the residual distance back to the unclamped coordinate. The distance is zero when
    # the point lies inside a box, but seed points are sampled to lie outside every box.
    center_deltas = point - obb_centers

    # Project each (point - center) onto box axis j by summing its world components weighted by axis j's
    # components: center_deltas is (num_boxes, 3) and obb_axes is (num_boxes, 3, 3), so we broadcast the delta
    # across the axes (adding a trailing axis-index dimension) and sum over the world-component dimension.
    points_component = np.sum(center_deltas[:,:,np.newaxis]*obb_axes, axis=1)
    clamped = np.clip(points_component, -obb_half_extents, obb_half_extents)
    return np.linalg.norm(points_component - clamped, axis=1).min()

def face_free_distance(box_min, box_max, axis, direction, scene_min, scene_max, obb_centers, obb_axes, obb_half_extents):

    # Closed-form distance the given face can advance before the box first contacts a component bounding box or the
    # scene bound. The face is identified by (axis, direction): direction -1 advances box_min outward along the
    # axis, +1 advances box_max. Advancing this face by a distance t makes the box's center and half-extent affine
    # in t along the moving axis, center[axis](t) = center0 + direction*t/2 and half_extent[axis](t) = half0 + t/2,
    # and leaves the other two axes unchanged.
    #
    # We use the separating axis theorem: two boxes are disjoint exactly when their projections onto some axis do
    # not overlap, where the candidate axes are the 3 world axes, each OBB's 3 axes, and the 9 cross products of a
    # world axis with an OBB axis. On a candidate axis a, the overlap condition |a.(obb_center - center(t))| <=
    # aabb_radius(t) + obb_radius is linear in t, and because the box only grows, each OBB's overlap region is an
    # upper half-line t >= t_contact. We solve for each OBB's first-contact t (the largest of its per-axis lower
    # bounds) and return the nearest contact over all OBBs, clamped to the scene bound.
    center0 = (box_min + box_max)/2.0
    half_extent0 = (box_max - box_min)/2.0
    center_deltas = obb_centers - center0

    world_axes = np.identity(3)

    # Gather the 15 candidate separating axes as (num_boxes, 3) arrays (the world axes are shared across all boxes):
    # the 3 world axes (the AABB's face normals), each OBB's 3 axes (its face normals), and the 9 cross products of
    # a world axis with an OBB axis (the edge-edge axes).
    candidate_axes = [
        np.broadcast_to(world_axes[0], obb_centers.shape),
        np.broadcast_to(world_axes[1], obb_centers.shape),
        np.broadcast_to(world_axes[2], obb_centers.shape),
        obb_axes[:,:,0],
        obb_axes[:,:,1],
        obb_axes[:,:,2],
        np.cross(world_axes[0], obb_axes[:,:,0]),
        np.cross(world_axes[0], obb_axes[:,:,1]),
        np.cross(world_axes[0], obb_axes[:,:,2]),
        np.cross(world_axes[1], obb_axes[:,:,0]),
        np.cross(world_axes[1], obb_axes[:,:,1]),
        np.cross(world_axes[1], obb_axes[:,:,2]),
        np.cross(world_axes[2], obb_axes[:,:,0]),
        np.cross(world_axes[2], obb_axes[:,:,1]),
        np.cross(world_axes[2], obb_axes[:,:,2])]

    # For each OBB accumulate its first-contact t (the max of its per-axis lower bounds), and whether it can be
    # contacted at all (every axis must admit some t, otherwise growth moves the box away from the OBB forever).
    contact_distance = np.zeros(obb_centers.shape[0])
    contactable = np.ones(obb_centers.shape[0], dtype=bool)

    for a in candidate_axes:
        a_along_axis = a[:,axis]
        growth_rate = np.abs(a_along_axis)/2.0               # rate the AABB's projected radius grows with t
        center_separation = np.sum(a*center_deltas, axis=1)  # a.(obb_center - center0)
        aabb_radius = np.sum(np.abs(a)*half_extent0, axis=1) # AABB projected radius at t = 0

        # OBB projected radius: project a onto each OBB axis j (summing a's components weighted by axis j's), then
        # weight by the OBB's half-extents.
        a_in_obb_frame = np.sum(a[:,:,np.newaxis]*obb_axes, axis=1)
        obb_radius = np.sum(np.abs(a_in_obb_frame)*obb_half_extents, axis=1)
        radius_sum = aabb_radius + obb_radius

        # A near-zero cross product comes from near-parallel edges and is not a valid separating axis, so skip it.
        valid = np.linalg.norm(a, axis=1) > cross_axis_tolerance

        # Axes whose direction is perpendicular to the moving axis (growth_rate == 0) do not change with t, so they
        # impose a constant condition: the projections must already overlap, else the OBB is never contacted. Axes
        # that do change with t give a lower bound on t and a feasibility condition (the OBB must lie on the side
        # the face is advancing toward). q is the sign of the moving axis component relative to the growth direction.
        moving = valid & (growth_rate > 0.0)
        perpendicular = valid & (growth_rate == 0.0)

        q = np.sign(a_along_axis*direction)

        # growth_rate is zero for axes perpendicular to the moving axis, so we intentionally let this division
        # produce inf/nan for those entries rather than branching, and suppress the expected divide-by-zero and 0/0
        # warnings; the np.where below discards exactly those entries.
        with np.errstate(divide="ignore", invalid="ignore"):
            lower_bound = (q*center_separation - radius_sum)/(2.0*growth_rate)
        lower_bound = np.where(moving, np.maximum(lower_bound, 0.0), 0.0)
        contact_distance = np.maximum(contact_distance, lower_bound)

        feasible = np.where(moving, radius_sum + q*center_separation >= 0.0, np.where(perpendicular, np.abs(center_separation) <= radius_sum, True))
        contactable = np.logical_and(contactable, feasible)

    # Each contactable OBB blocks the face at its first-contact distance; unreachable OBBs never block it.
    obb_distance = np.where(contactable, contact_distance, np.inf).min(initial=np.inf)

    if direction < 0:
        scene_distance = box_min[axis] - scene_min[axis]
    else:
        scene_distance = scene_max[axis] - box_max[axis]

    return min(obb_distance, scene_distance)

def compute_coverage_indicator_matrix(free_space_points, box_mins, box_maxs):

    # coverage_indicator_matrix[p, b] is True if candidate bounding box b covers free-space point p. We broadcast the
    # (P, 1, 3) points against the (1, B, 3) box bounds so the containment test is vectorized over every point-box
    # pair, then require containment on all three axes. Each box is expanded by coverage_tolerance so a point lying
    # exactly on a box face still counts as covered.
    points = free_space_points[:,np.newaxis,:]
    inside_min = points >= box_mins[np.newaxis,:,:] - coverage_tolerance
    inside_max = points <= box_maxs[np.newaxis,:,:] + coverage_tolerance
    return np.all(np.logical_and(inside_min, inside_max), axis=2)

def select_covering_bounding_boxes(coverage_indicator_matrix):

    num_points, num_boxes = coverage_indicator_matrix.shape

    # The most points any selection can cover: a point is coverable if at least one candidate box covers it. We
    # count covering every coverable point as 100% coverage, and stop once we reach coverage_quota_fraction of it.
    num_coverable_points = np.count_nonzero(coverage_indicator_matrix.any(axis=1))
    if num_coverable_points == 0:
        spear.log("No coverable free-space points, so selecting no bounding boxes.")
        return np.zeros(0, dtype=int)
    coverage_quota = coverage_quota_fraction*num_coverable_points

    # Lazy (CELF) greedy maximization of coverage, which is a positive, monotone, submodular objective. The marginal
    # reward of adding a box is the number of currently-uncovered points it covers. Because the objective is
    # submodular, a box's marginal reward can only decrease as we select more boxes, so a marginal reward computed in
    # an earlier iteration is an upper bound on its current value. We keep these (possibly stale) marginal rewards in
    # an array, and each iteration we repeatedly recompute the current top box's reward until the top box's reward is
    # known to be current: it is then the true best box, because every other box's stale reward is an upper bound
    # that does not exceed it. This recomputes far fewer rewards than the naive greedy algorithm, which would
    # recompute the reward of every box on every iteration.
    marginal_rewards = coverage_indicator_matrix.sum(axis=0)
    selected = np.zeros(num_boxes, dtype=bool)
    covered = np.zeros(num_points, dtype=bool)

    covering_indices = []
    num_covered_points = 0
    while len(covering_indices) < max_num_covering_bounding_boxes and num_covered_points < coverage_quota:

        # Find the unselected box with the largest marginal reward, recomputing stale rewards until the top box's
        # reward is current. is_current marks the boxes recomputed against the current covered set this iteration,
        # and the -1 sentinel keeps already-selected boxes from ever being chosen.
        is_current = np.zeros(num_boxes, dtype=bool)
        while True:
            box = np.argmax(np.where(selected, -1, marginal_rewards))
            if is_current[box]:
                break
            marginal_rewards[box] = np.count_nonzero(np.logical_and(coverage_indicator_matrix[:,box], np.logical_not(covered)))
            is_current[box] = True

        # Every remaining box covers no new points, so no further selection can improve coverage.
        if marginal_rewards[box] == 0:
            break

        selected[box] = True
        covered = np.logical_or(covered, coverage_indicator_matrix[:,box])
        num_covered_points = np.count_nonzero(covered)
        covering_indices.append(box)

    spear.log(f"Selected {len(covering_indices)} of {num_boxes} candidate bounding boxes covering {num_covered_points} of {num_coverable_points} coverable points ({100.0*num_covered_points/num_coverable_points:.1f}%).")

    return np.array(covering_indices, dtype=int)

def compute_overlap_graph(box_mins, box_maxs):

    # overlap_graph[a, c] is True if candidate box c overlaps candidate box a by at least overlap_fraction
    # of box a's length along every axis. Along each axis the overlap length is the length of the intersection of the
    # two boxes' [min, max] intervals (clamped at zero), and we require it to be at least that fraction of box a's
    # extent along that axis. The condition is relative to box a, so the graph is not symmetric. We broadcast the
    # (B, 1, 3) box-a bounds against the (1, B, 3) box-c bounds so the test is vectorized over every (a, c) pair.
    overlap_min = np.maximum(box_mins[:,np.newaxis,:], box_mins[np.newaxis,:,:])
    overlap_max = np.minimum(box_maxs[:,np.newaxis,:], box_maxs[np.newaxis,:,:])
    overlap_length = np.maximum(overlap_max - overlap_min, 0.0)
    box_length = box_maxs - box_mins
    return np.all(overlap_length >= overlap_fraction*box_length[:,np.newaxis,:], axis=2)

def select_overlapping_bounding_boxes(coverage_indicator_matrix, overlap_graph, anchor_indices):

    num_points, num_boxes = coverage_indicator_matrix.shape
    num_anchors = anchor_indices.shape[0]
    if num_anchors == 0:
        spear.log("No anchor bounding boxes, so selecting no footprint bounding boxes.")
        return np.zeros(0, dtype=int)

    # The footprint of an anchor box (a phase-1 box) is the space outside it that belongs to candidate boxes
    # overlapping it. We discretize each anchor's footprint by the free-space points: footprint element (anchor a,
    # point p) exists when point p lies outside anchor a but inside some candidate box that overlaps anchor a, and a
    # candidate box c covers that element when c overlaps anchor a and contains p. Maximizing the number of covered
    # footprint elements summed over all anchors is a sum of coverage objectives, hence positive, monotone, and
    # submodular, so we select with the same lazy greedy algorithm as the phase-1 coverage problem. Rather than
    # materializing the (num_anchors*num_points, num_boxes) footprint coverage matrix, we factor it through the
    # phase-1 coverage indicator matrix and the overlap graph.
    #
    # Every array stays indexed by candidate box index, the same as coverage_indicator_matrix, so a column always
    # refers to a bounding box: covered_footprint[p, a] and coverable_footprint[p, a] hold footprint state for the
    # anchor in column a (only anchor columns are populated). Note coverage_indicator_matrix[p, a] already tells us
    # whether anchor a contains point p, which is exactly the "outside the anchor" test (we negate it), so we need no
    # separate array for that. is_anchor marks the anchor columns.
    is_anchor = np.zeros(num_boxes, dtype=bool)
    is_anchor[anchor_indices] = True

    # covered_footprint[p, a] is True if footprint element (anchor a, point p) is covered by the current selection.
    # The anchors are already part of the network, so we seed it with what the anchors themselves cover: anchor a's
    # footprint point p is covered if some anchor that overlaps anchor a contains p (and p lies outside anchor a).
    # overlap_to_anchor[a, a'] keeps only the contributing boxes a' that are anchors, and the matrix product counts,
    # for each (point p, anchor a), the overlapping anchors that contain p.
    overlap_to_anchor = np.logical_and(overlap_graph, is_anchor[np.newaxis,:])
    coverage_indicator_as_matrix = np.matrix(coverage_indicator_matrix.astype(int))
    overlap_to_anchor_as_matrix = np.matrix(overlap_to_anchor.astype(int))
    overlapping_anchor_counts = (coverage_indicator_as_matrix*overlap_to_anchor_as_matrix.T).A
    covered_footprint = np.logical_and(overlapping_anchor_counts > 0, np.logical_and(np.logical_not(coverage_indicator_matrix), is_anchor[np.newaxis,:]))

    # The most footprint elements any selection can cover: footprint element (anchor a, point p) is coverable if some
    # candidate box overlapping anchor a contains p (and p lies outside anchor a). We count covering every coverable
    # element as 100%, and stop once we reach overlapping_quota_fraction of it.
    overlap_graph_as_matrix = np.matrix(overlap_graph.astype(int))
    overlapping_box_counts = (coverage_indicator_as_matrix*overlap_graph_as_matrix.T).A
    coverable_footprint = np.logical_and(overlapping_box_counts > 0, np.logical_and(np.logical_not(coverage_indicator_matrix), is_anchor[np.newaxis,:]))
    num_coverable_footprint = np.count_nonzero(coverable_footprint)
    if num_coverable_footprint == 0:
        spear.log("No coverable footprint, so selecting no footprint bounding boxes.")
        return np.zeros(0, dtype=int)
    footprint_quota = overlapping_quota_fraction*num_coverable_footprint

    # Lazy (CELF) greedy maximization, exactly as in select_covering_bounding_boxes: marginal rewards are upper bounds
    # that can only decrease as we select more boxes, so we keep them in an array and recompute the current top box's
    # reward until it is current. The marginal reward of a candidate box c is the number of not-yet-covered footprint
    # elements it covers: summed over the anchors c overlaps, the points c contains that lie outside the anchor and
    # are not already covered. The matrix product below evaluates this for every candidate box at once to initialize
    # the rewards (column c of overlap_graph is the set of anchors c overlaps). The anchors are already in the
    # network, so we mark them selected and only add non-anchor boxes.
    available_footprint = np.logical_and(np.logical_and(np.logical_not(coverage_indicator_matrix), np.logical_not(covered_footprint)), is_anchor[np.newaxis,:])
    available_footprint_as_matrix = np.matrix(available_footprint.astype(int))
    available_footprint_counts = (coverage_indicator_as_matrix.T*available_footprint_as_matrix).A
    marginal_rewards = (overlap_graph.T*available_footprint_counts).sum(axis=1)
    selected = np.zeros(num_boxes, dtype=bool)
    selected[anchor_indices] = True

    overlapping_indices = []
    num_covered_footprint = np.count_nonzero(covered_footprint)
    while len(overlapping_indices) < max_num_overlapping_bounding_boxes and num_covered_footprint < footprint_quota:

        # Find the unselected box with the largest marginal reward, recomputing stale rewards until the top box's
        # reward is current. is_current marks the boxes recomputed against the current covered footprint this
        # iteration, and the -1 sentinel keeps already-selected boxes (including the anchors) from being chosen.
        is_current = np.zeros(num_boxes, dtype=bool)
        while True:
            box = np.argmax(np.where(selected, -1, marginal_rewards))
            if is_current[box]:
                break
            anchor_columns = np.logical_and(overlap_graph[:,box], is_anchor)
            newly_covered = np.logical_and(coverage_indicator_matrix[:,box,np.newaxis], np.logical_and(np.logical_not(coverage_indicator_matrix[:,anchor_columns]), np.logical_not(covered_footprint[:,anchor_columns])))
            marginal_rewards[box] = np.count_nonzero(newly_covered)
            is_current[box] = True

        # Every remaining box covers no new footprint, so no further selection can improve the footprint coverage.
        if marginal_rewards[box] == 0:
            break

        anchor_columns = np.logical_and(overlap_graph[:,box], is_anchor)
        newly_covered = np.logical_and(coverage_indicator_matrix[:,box,np.newaxis], np.logical_and(np.logical_not(coverage_indicator_matrix[:,anchor_columns]), np.logical_not(covered_footprint[:,anchor_columns])))
        covered_footprint[:,anchor_columns] = np.logical_or(covered_footprint[:,anchor_columns], newly_covered)
        num_covered_footprint = num_covered_footprint + np.count_nonzero(newly_covered)
        selected[box] = True
        overlapping_indices.append(box)

    spear.log(f"Selected {len(overlapping_indices)} of {num_boxes} candidate bounding boxes covering {num_covered_footprint} of {num_coverable_footprint} coverable footprint elements ({100.0*num_covered_footprint/num_coverable_footprint:.1f}%).")

    return np.array(overlapping_indices, dtype=int)


if __name__ == "__main__":
    process_scene()
