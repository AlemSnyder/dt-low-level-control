# -*- coding: utf-8 -*-

from .baseClass import BaseEditorClass


class MapTile(BaseEditorClass):
    rotation_val = {0: 'E', 90: 'S', 180: 'W', 270: 'N'}
    no_rotation_tile = ('empty', 'asphalt', 'grass', 'floor', '4way')

    def __init__(self, kind, rotation=0):
        BaseEditorClass.__init__(self, dict(kind=kind, rotate=rotation))

    def __iter__(self):
        yield from {
            'kind': self.kind,
            'rotation': self.rotation
        }.items()

    def __str__(self):
        return '{}{}'.format(self.kind, self.rotation_to_str())
    
    def rotation_to_str(self):
        if self.kind not in self.no_rotation_tile:
            return '/{}'.format(self.rotation_val[self.rotation])
        else:
            return ''
