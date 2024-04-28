import utm
from typing import Tuple


def convert_to_utm(latitude: float, longitude: float) -> Tuple[float, float]:
    """
    Translate tuple (latitude, longitude) to (x, y).
    
    Returns:
        tuple: x, y.
    """
    easting, northing, _, _ = utm.from_latlon(latitude, longitude)
    return (northing, easting)
