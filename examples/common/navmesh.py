#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

class NavMesh:
    def __init__(self, instance):
        self._instance = instance

    def get_random_points(self, num_poses):
        with self._instance.begin_frame():
            points = self._instance.legacy_service.get_random_points(num_poses)
        with self._instance.end_frame():
            pass
        return points

    def get_random_reachable_points_in_radius(self, initial_points, radius):
        with self._instance.begin_frame():
            reachable_points = self._instance.legacy_service.get_random_reachable_points_in_radius(initial_points, radius)
        with self._instance.end_frame():
            pass
        return reachable_points

    def get_paths(self, initial_points, goal_points):
        with self._instance.begin_frame():
            paths = self._instance.legacy_service.get_paths(initial_points, goal_points)
        with self._instance.end_frame():
            pass
        return paths
