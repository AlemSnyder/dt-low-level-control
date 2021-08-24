from typing import Dict, Tuple, Optional, List

from duckietown_world.structure.duckietown_map import DuckietownMap


class ManagerDuckietownMaps:
    def __init__(self):
        self._maps: Dict[str, DuckietownMap] = {}
        self.is_active: str = ""
        self.main_relative: str = ""  # TODO: save main relative in map

    def add_map(self, new_map_name: str, map_obj: DuckietownMap) -> None:
        if new_map_name not in self._maps:
            self._maps[new_map_name] = map_obj

    def get_map(self, name: str) -> Optional[DuckietownMap]:
        if name in self._maps:
            self.is_active = name
            return self._maps[name]

    def get_maps_name(self) -> List[str]:
        return list(self._maps.keys())
