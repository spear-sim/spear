#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#


class NavMesh:
    def __init__(self, instance):
        self._instance = instance

    def get_random_points(self, num_poses):
        instance.engine_service.begin_tick()
        points = instance.legacy_service.get_random_points(num_poses)
        instance.engine_service.tick()
        instance.engine_service.end_tick()
        return points

    def get_random_reachable_points_in_radius(self, initial_points, radius):
        instance.engine_service.begin_tick()
        reachable_points = instance.legacy_service.get_random_reachable_points_in_radius(initial_points, radius)
        instance.engine_service.tick()
        instance.engine_service.end_tick()
        return reachable_points

    def get_paths(self, initial_points, goal_points):
        instance.engine_service.begin_tick()
        paths = instance.legacy_service.get_paths(initial_points, goal_points)
        instance.engine_service.tick()
        instance.engine_service.end_tick()
        return paths
