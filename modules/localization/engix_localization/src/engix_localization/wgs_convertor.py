import utm
from typing import Tuple


def convert_to_utm(latitude: float, longitude: float) -> Tuple[float, float]:
    easting, northing, _, _ = utm.from_latlon(latitude, longitude)
    return (northing, easting)
