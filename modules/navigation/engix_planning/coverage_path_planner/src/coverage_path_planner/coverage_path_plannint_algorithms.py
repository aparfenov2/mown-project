import numpy as np
from shapely.geometry import Polygon


from coverage_path_planner.area_polygon import AreaPolygon


from engix_msgs.msg import CoverageTask


class CoveragePathPlanningAlgorithm:
    def __init__(
        self, offset_distance: float, offset_resolution: float, step_size: float
    ) -> None:
        self.__offset_distance = offset_distance
        self.__offset_resolution = offset_resolution
        self.__step_size = step_size

    def plan(
        self,
        robot_position: list,
        listed_polygon: list,
        approximate: bool = False,
        auto_angle: bool = False,
        angle: float = 0.0,
    ) -> list:
        # listed_polygon = [(point.x, point.y) for point in message.target_polygon.points]
        listed_polygon = self.__create_offset(listed_polygon)

        if approximate:
            temp_polygon = listed_polygon
            listed_polygon = list()

            for i in range(1, len(temp_polygon)):
                listed_polygon.append(temp_polygon[i - 1])
                point_count = np.linalg.norm(
                    np.array(temp_polygon[i]) - np.array(temp_polygon[i - 1])
                )
                point_count = max(1, int(point_count))
                extended_segment = np.linspace(
                    temp_polygon[i - 1], temp_polygon[i], point_count
                ).tolist()
                listed_polygon += extended_segment
        if auto_angle:
            path_creator = AreaPolygon(
                listed_polygon, robot_position, ft=self.__step_size
            )
        else:
            path_creator = AreaPolygon(
                listed_polygon,
                robot_position,
                ft=self.__step_size,
                angle=angle,
            )

        coverage_route_points = path_creator.get_area_coverage()

        return [(point[0], point[1]) for point in coverage_route_points]

    def __create_offset(self, polygon):
        shapely_polygon = Polygon(polygon)
        poly_line_offset = shapely_polygon.buffer(
            self.__offset_distance,
            resolution=self.__offset_resolution,
            join_style=2,
            mitre_limit=1,
        ).exterior
        return poly_line_offset.coords
