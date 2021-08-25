from layers.layer_type import LayerType
from classes.mapObjects import MapBaseObject, SignObject, CityObject, WatchTowerObject, GroundAprilTagObject


LAYER_TYPE_WITH_OBJECTS = (LayerType.TRAFFIC_SIGNS, LayerType.GROUND_APRILTAG, LayerType.WATCHTOWERS, LayerType.ITEMS)

LAYER_OBJECTS = {
    LayerType.TRAFFIC_SIGNS: {
        'type': 'sign',
        'class': SignObject
    },
    LayerType.GROUND_APRILTAG: {
        'type': 'apriltag',
        'class': GroundAprilTagObject
    },
    LayerType.WATCHTOWERS: {
        'type': 'watchtower',
        'class': WatchTowerObject
    },
    LayerType.ITEMS: {
        'type': 'objects',
        'class': CityObject
    },
    'default': {
        'type': '',
        'class': MapBaseObject
    }
}


def get_class_by_object_type(object_type):
    """
    Get map object class from classes.mapObjects for object_type
    If object_type doesn't exist, return MapBaseObject
    :param object_type: type of object
    :return: class of map object
    """
    for layer_type, info in LAYER_OBJECTS.items():
        if object_type == info['type']:
            return info['class']
    return LAYER_OBJECTS['default']['class']


def get_class_by_layer_type(layer_type):
    """
    Get map object class from classes.mapObjects for layer_type
    If layer_type doesn't exist, return MapBaseObject
    :param layer_type: LayerType
    :return: class of map object
    """
    if layer_type in LAYER_OBJECTS:
        return LAYER_OBJECTS[layer_type]['class']
    else:
        return LAYER_OBJECTS['default']['class']


def get_layer_type_by_object_type(object_type):
    """
    Get layer type by object type (help know what layer type need for this object_type)
    If object_type doesn't exist, return LayerType.ITEMS
    :param object_type: type of object
    :return: LayerType
    """
    for layer_type, info in LAYER_OBJECTS.items():
        if object_type == info['type']:
            return layer_type
    else:
        return LayerType.ITEMS


def get_layer_type_by_value(type_value):
    for layer_type in LayerType: 
        if type_value == layer_type.value:
            return layer_type
