from enum import Enum


class LayerType(Enum):
    TILES = '0-tiles'
    TRAFFIC_SIGNS = '1-signs'
    GROUND_APRILTAG = '2-groundtags'
    WATCHTOWERS = '3-watchtowers'
    REGIONS = '4-regions'
    ITEMS = 'objects'

    def __str__(self):
        return self.value
