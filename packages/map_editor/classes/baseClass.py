# -*- coding: utf-8 -*-

class BaseEditorClass:
    def __init__(self, init_info):
        self.kind = init_info['kind']
        self.rotation = init_info['rotate']

    def __iter__(self):
        raise NotImplementedError("Subclasses should implement __iter__")
