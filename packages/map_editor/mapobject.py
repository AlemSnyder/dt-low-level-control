# -*- coding: utf-8 -*-
class MapObject:

    def __init__(self, kind, position=(0.0, 0.0), rotation=0, height=1, optional=False, static=True):
        self.kind = kind
        self.position = list(init_info['pos'])
        self.rotation = rotation
        self.height = height
        self.optional = optional
        self.static = static
