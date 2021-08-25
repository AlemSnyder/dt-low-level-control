from typing import Optional

from duckietown_world.structure.duckietown_map import DuckietownMap
from duckietown_world.structure.map_factory import MapFactory
import os

DT_WORLD = None


def get_dt_world(map_name=None) -> DuckietownMap:
    global DT_WORLD
    if not DT_WORLD:
        if map_name:
            DT_WORLD = MapFactory.load_map(os.path.abspath(map_name))
        else:
            DT_WORLD = MapFactory.load_map(os.path.abspath("maps/empty"))
    return DT_WORLD


def get_new_dt_world(map_name: Optional[str] = None) -> Optional[DuckietownMap]:
    if map_name:
        return MapFactory.load_map(os.path.abspath(map_name))
