# -*- coding: utf-8 -*-
from map import DuckietownMap
from mapviewer import MapViewer
from collections import deque
import copy


# Don't forget call setMap on map change
class MapEditor:
    memento = None
    map = None
    viewer = None

    def __init__(self, map: DuckietownMap, viewer: MapViewer):
        self.map = map
        self.viewer = viewer
        self.memento = deque([], maxlen=150)

    @DeprecationWarning
    def setMap(self, newMap: DuckietownMap):
        self.map = newMap

    def __createBuffer(self, selection):
        buffer = []
        tile_layer = self.map.get_tile_layer().data
        for i in range(selection[3] - selection[1]):
            buffer.append([])
            for j in range(selection[2] - selection[0]):
                buffer[i].append(copy.copy(
                    tile_layer[selection[1] + i][selection[0] + j]))
                # buffer[i][j] = copy.copy(
                #     self.map.tiles[selection[1] + i][selection[0] + j])
        return buffer

    def __extendAndPasteBuffer(self, selection, buffer, destX, destY, backgroundTile):
        self.extendToFit(selection, destX, destY, backgroundTile)
        tile_layer = self.map.get_tile_layer().data
        for i in range(len(buffer)):
            for j in range(len(buffer[i])):
                # print('paste to ['+str(destX + j)+' ; ' + str(destY + i)+ ']')
                tile_layer[destY + i][destX + j] = buffer[i][j]

    def extendToFit(self, selection, destX, destY, backgroundTile):
        tile_layer = self.map.get_tile_layer().data
        u, r, d, l = 0, 0, 0, 0
        if destX < 0:
            l = -destX
            destX = 0
        elif destX + (selection[2] - selection[0]) > len(tile_layer[0]):
            r = destX + (selection[2] - selection[0]) - len(tile_layer[0])
        if destY < 0:
            u = -destY
            destY = 0
        elif destY + (selection[3] - selection[1]) > len(tile_layer):
            d = destY + (selection[3] - selection[1]) - len(tile_layer)
        self.addBorderLines(u, r, d, l, backgroundTile)

    # TO TEST
    def copySelection(self, selection, destX, destY, backgroundTile):
        buffer = self.__createBuffer(selection)
        self.__extendAndPasteBuffer(selection, buffer, destX, destY, backgroundTile)
        return

    # TO TEST
    def moveSelection(self, selection, destX, destY, backgroundTile):
        buffer = self.__createBuffer(selection)
        self.deleteSelection(selection, backgroundTile)
        self.__extendAndPasteBuffer(selection, buffer, destX, destY, backgroundTile)
        return

    # TO TEST
    def deleteSelection(self, selection, backgroundTile):
        tile_layer = self.map.get_tile_layer().data
        for y in range(max(0, selection[1]), min(selection[3], len(tile_layer))):
            for x in range(max(0, selection[0]), min(selection[2], len(tile_layer[0]))):
                tile_layer[y][x] = copy.copy(backgroundTile)
        return

    # TO TEST
    def addBorderLines(self, linesUp: int, linesRight: int, linesDown: int, linesLeft: int, backgroundTile):
        tile_layer = self.map.get_tile_layer().data
        for i in range(linesUp):
            l = len(tile_layer[0])
            tile_layer.insert(0, [])
            for j in range(l):
                tile_layer[0].append(copy.copy(backgroundTile))
        for i in range(linesDown):
            emptyLine = []
            l = len(tile_layer[0])
            for j in range(l):
                emptyLine.append(copy.copy(backgroundTile))
            if len(tile_layer[len(tile_layer)-1]) == 0:
                tile_layer[len(tile_layer)-1] = emptyLine
            else:
                tile_layer.append(emptyLine)
        for j in range(len(tile_layer)):
            for i in range(linesLeft):
                tile_layer[j].insert(0, copy.copy(backgroundTile))
            for i in range(linesRight):
                tile_layer[j].append(copy.copy(backgroundTile))
        return

        #  Removes lines at the edges if all tiles in the row are background
        # TO TEST

    def trimBorders(self, trimUp: bool, trimRight: bool, trimDown: bool, trimLeft: bool, backgroundTile):
        isLineEmpty = True
        tile_layer = self.map.get_tile_layer().data
        if trimDown:
            while isLineEmpty and tile_layer:
                for i in range(len(tile_layer[len(tile_layer) - 1])):
                    if tile_layer[len(tile_layer) - 1][i].kind != backgroundTile.kind:
                        isLineEmpty = False
                        break
                if isLineEmpty:
                    tile_layer.pop()
        isLineEmpty = True
        if trimUp:
            while isLineEmpty and tile_layer:
                for i in range(len(tile_layer[0])):
                    if tile_layer[0][i].kind != backgroundTile.kind:
                        isLineEmpty = False
                        break
                if isLineEmpty:
                    tile_layer.pop(0)
        isLineEmpty = True
        if trimRight:
            while isLineEmpty and tile_layer:
                for i in range(len(tile_layer)):
                    if tile_layer[i][len(tile_layer[i]) - 1].kind != backgroundTile.kind:
                        isLineEmpty = False
                        break
                if isLineEmpty:
                    for i in range(len(tile_layer)):
                        tile_layer[i].pop()
        isLineEmpty = True
        if trimLeft:
            while isLineEmpty and tile_layer:
                for i in range(len(tile_layer)):
                    if tile_layer[i][0].kind != backgroundTile.kind:
                        isLineEmpty = False
                        break
                if isLineEmpty:
                    for i in range(len(tile_layer)):
                        tile_layer[i].pop(0)
        if not tile_layer:
            self.map.set_tile_layer([[]])
        return

    def save(self, map_to_save):
        # deep copy
        backup = DuckietownMap(empty=True)
        # Fill backup layers
        for layer in map_to_save.layers:
            backup.add_layer(copy.deepcopy(layer))
        if len(self.memento) == self.memento.maxlen - 1:
            self.memento.popleft()
        self.memento.append(backup)

    def undo(self):
        if len(self.memento) == 0:
            return
        backup = self.memento.pop()
        self.map.layers.clear()
        # Fill layers from backup
        for layer in backup.layers:
            self.map.add_layer(layer)
