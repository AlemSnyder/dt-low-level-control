# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog


def get_lab_code(parent): return QtWidgets.QInputDialog.getText(parent, "Lab code", "Enter Lab's code for map:")


def save_map_as(parent: QtWidgets.QWidget):
    if parent.map.layers:
        output_map_dir = QFileDialog.getExistingDirectory(parent, 'Save map to directory', '.',
                                                          QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        return output_map_dir

