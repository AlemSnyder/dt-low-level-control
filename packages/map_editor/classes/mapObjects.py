# -*- coding: utf-8 -*-
from .baseClass import BaseEditorClass


class MapBaseObject(BaseEditorClass):
    def __init__(self, init_info):
        BaseEditorClass.__init__(self, init_info)
        self.position = list(init_info['pos'])
        self.height = init_info['height']
        self.optional = init_info['optional']
        self.static = init_info['static']

    def __iter__(self):
        yield from {
            'kind': self.kind,
            'height': self.height,
            'pos': self.position,
            'rotate': self.rotation,
            'optional': self.optional,
            'static': self.static
        }.items()

    def get_editable_attrs(self):
        return {
            'height': self.height,
            'pos': self.position,
            'rotate': self.rotation,
            'optional': self.optional,
            'static': self.static
        }


class SignObject(MapBaseObject):

    def __init__(self, init_info):
        MapBaseObject.__init__(self, init_info)
        self.tag_id = init_info["tag_id"] if "tag_id" in init_info else 0

    def get_editable_attrs(self):
        return {
            'pos': self.position,
            'rotate': self.rotation,
            "tag_id": self.tag_id,
        }
    
    def __iter__(self):
        yield from {
            'kind': self.kind,
            'height': self.height,
            'pos': self.position,
            'rotate': self.rotation,
            'optional': self.optional,
            'static': self.static,
            "tag_id": self.tag_id,
        }.items()
    

class CityObject(MapBaseObject):

    def __init__(self, init_info):
        MapBaseObject.__init__(self, init_info)


class WatchTowerObject(MapBaseObject):

    def __init__(self, init_info):
        MapBaseObject.__init__(self, init_info)
        self.hostname = init_info["hostname"] if "hostname" in init_info else "watchtower00" # TODO: How to init hostname?

    def __iter__(self):
        yield from {
            'kind': self.kind,
            'height': self.height,
            'pos': self.position,
            'rotate': self.rotation,
            'optional': self.optional,
            'static': self.static,
            'hostname': self.hostname
        }.items()

    def get_editable_attrs(self):
        return {
            'height': self.height,
            'pos': self.position,
            'rotate': self.rotation,
            'hostname': self.hostname
        }


class GroundAprilTagObject(MapBaseObject):

    def __init__(self, init_info):
        MapBaseObject.__init__(self, init_info)
        self.tag_id = init_info["tag_id"] if "tag_id" in init_info else 0
        self.tag_type = init_info["tag_type"] if "tag_type" in init_info else ""


    def get_editable_attrs(self):
        return {
            'pos': self.position,
            'rotate': self.rotation,
            "tag_id": self.tag_id,
            "tag_type": self.tag_type
        }
    
    def __iter__(self):
        yield from {
            'kind': self.kind,
            'height': self.height,
            'pos': self.position,
            'rotate': self.rotation,
            'optional': self.optional,
            'static': self.static,
            "tag_id": self.tag_id,
            "tag_type": self.tag_type
        }.items()
