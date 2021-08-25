from os import path
# -*- coding: utf-8 -*-
import codecs
import copy
import functools
import json
import numpy as np
import os

from PyQt5.QtWidgets import QMessageBox, QDesktopWidget, QFormLayout, QVBoxLayout, QLineEdit, QGroupBox, \
    QLabel, QComboBox, QFrame, QGridLayout, QPushButton, QHBoxLayout

from duckietown_world.structure.bases import _Frame
from duckietown_world.structure.duckietown_map import DuckietownMap
from duckietown_world.structure.objects import Watchtower, Citizen, Tile, TrafficSign, GroundTag, Vehicle, Camera, \
    _Camera, _Group
from duckietown_world.structure.old_format.convert import convert_new_format, dump
import map
import mapviewer
import utils
from DTWorld import get_dt_world
from DTWorld import get_new_dt_world
from IOManager import *
import logging
from classes.mapObjects import GroundAprilTagObject
from duckietown_world.structure.utils import get_degree_for_orientation, get_orientation_for_degree, \
    get_canonical_sign_name

from classes.mapTile import MapTile
from forms.default_forms import question_form_yes_no
from forms.env import EnvForm
from forms.new_region import NewGroupForm
from forms.new_tag_object import NewTagForm
from forms.start_info import StartInfoForm
from infowindow import info_window
from layers.layer_type import LayerType
from layers.relations import get_layer_type_by_object_type
from main_design import *
from managerduckietownmaps import ManagerDuckietownMaps
from mapEditor import MapEditor
from tag_config import get_duckietown_types

logger = logging.getLogger('root')
TILE_TYPES = ('block', 'road')
DEFAULT_TILE_SIZE = 0.585

# pyuic5 main_design.ui -o main_design.py

_translate = QtCore.QCoreApplication.translate
EPS = .1  # step for move

# rot_val = {'E': 0, 'S': 90, 'W': 180, 'N': 270, None: 0}
rot_val = {'E': 0, 'S': 270, 'W': 180, 'N': 90, None: 0}


class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class duck_window(QtWidgets.QMainWindow):
    map = None
    mapviewer = None
    info_json = None
    editor = None
    drawState = ''
    copyBuffer = [[]]

    def __init__(self, args, elem_info="doc/info.json"):
        super().__init__()
        # active items in editor
        self.distortion_view_one_string_mode = True
        self.region_create = False
        self.active_items = []
        self.active_group = None
        self.name_of_editable_obj = None
        self.dm = get_dt_world()
        self.tile_size = DEFAULT_TILE_SIZE
        self.duckie_manager = ManagerDuckietownMaps()
        self.duckie_manager.add_map("maps/empty", self.dm)

        #  additional windows for displaying information
        self.author_window = info_window()
        self.param_window = info_window()
        self.mater_window = info_window()

        #  The brush button / override the closeEvent
        self.brush_button = QtWidgets.QToolButton()
        self.closeEvent = functools.partial(self.quit_program_event)

        # Set locale
        self.locale = args.locale

        # Set debug mode
        self.debug_mode = args.debug

        # Load element's info
        self.info_json = json.load(codecs.open(elem_info, "r", "utf-8"))

        # Loads info about types from duckietown
        self.duckietown_types_apriltags = get_duckietown_types()
        #####  Forms   #############
        self.new_tag_class = NewTagForm(self.duckietown_types_apriltags)
        self.init_info_form = StartInfoForm()
        self.new_group_form = NewGroupForm()
        self.env_form = EnvForm()
        ############################
        map_name = "maps/empty"
        self.dm = get_dt_world(map_name)
        logger.debug(self.dm.get_context())
        self.map = map.DuckietownMap()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        viewer = mapviewer.MapViewer()
        self.editor = MapEditor(self.map, self.mapviewer)
        viewer.setMap(self.map)
        self.mapviewer = viewer
        viewer.setMinimumSize(540, 540)
        self.ui.horizontalLayout.addWidget(viewer)
        viewer.repaint()
        self.initUi()

        self.update_layer_tree()

    def get_translation(self, elem):
        """Gets info about the element based on self.locale
        If local doesn't exist, return locale='en'

        :param self
        :param elem: dict, information about elements (or category), that contains translation
        :return: dict(). Dict with translation based on the self.locale
        """
        if self.locale in elem['lang']:
            return elem['lang'][self.locale]
        else:
            logger.debug("duck_window.get_translation. No such locale: {}".format(self.locale))
            return elem['lang']['en']

    def initUi(self):
        self.center()
        self.show()

        #  Initialize button objects
        create_map = self.ui.create_new
        open_map = self.ui.open_map
        save_map = self.ui.save_map
        save_map_as = self.ui.save_map_as
        calc_param = self.ui.calc_param
        about_author = self.ui.about_author
        exit = self.ui.exit
        change_blocks = self.ui.change_blocks
        change_info = self.ui.change_info
        change_map = self.ui.change_map
        change_layer = self.ui.change_layer
        distortion_view = self.ui.distortion_view
        create_region = self.ui.region_create
        import_old_format = self.ui.import_old_format
        environment = self.ui.env

        #  Initialize floating blocks
        block_widget = self.ui.block_widget
        info_widget = self.ui.info_widget
        map_info_widget = self.ui.map_info_widget
        layer_info_widget = self.ui.layer_info_widget

        #  Signal from viewer
        self.mapviewer.selectionChanged.connect(self.selectionUpdate)
        self.mapviewer.editObjectChanged.connect(self.create_form)
        self.new_tag_class.apriltag_added.connect(self.add_apriltag)

        #  Assign actions to buttons
        create_map.triggered.connect(self.create_map_triggered)
        open_map.triggered.connect(self.open_map_triggered)
        save_map.triggered.connect(self.save_map_triggered)
        save_map_as.triggered.connect(self.save_map_as_triggered)
        calc_param.triggered.connect(self.calc_param_triggered)
        about_author.triggered.connect(self.about_author_triggered)
        distortion_view.triggered.connect(self.change_distortion_view_triggered)
        create_region.triggered.connect(self.create_region)
        import_old_format.triggered.connect(self.import_old_format)
        environment.triggered.connect(self.change_env)
        exit.triggered.connect(self.exit_triggered)

        change_blocks.toggled.connect(self.change_blocks_toggled)
        change_info.toggled.connect(self.change_info_toggled)
        change_map.toggled.connect(self.change_map_toggled)
        change_layer.toggled.connect(self.toggle_layer_window)

        block_widget.closeEvent = functools.partial(self.blocks_event)
        info_widget.closeEvent = functools.partial(self.info_event)
        map_info_widget.closeEvent = functools.partial(self.map_event)
        layer_info_widget.closeEvent = functools.partial(self.close_layer_window_event)

        #  QToolBar setting
        tool_bar = self.ui.tool_bar

        a1 = QtWidgets.QAction(QtGui.QIcon("img/icons/new.png"), _translate("MainWindow", "New map"), self)
        a2 = QtWidgets.QAction(QtGui.QIcon("img/icons/open.png"), _translate("MainWindow", "Open map"), self)
        a3 = QtWidgets.QAction(QtGui.QIcon("img/icons/save.png"), _translate("MainWindow", "Save map"), self)
        a4 = QtWidgets.QAction(QtGui.QIcon("img/icons/save_as.png"), _translate("MainWindow", "Save map as"), self)
        a5 = QtWidgets.QAction(QtGui.QIcon("img/icons/png.png"), _translate("MainWindow", "Export to PNG"), self)

        b1 = QtWidgets.QAction(QtGui.QIcon("img/icons/copy.png"), _translate("MainWindow", "Copy"), self)
        b2 = QtWidgets.QAction(QtGui.QIcon("img/icons/cut.png"), _translate("MainWindow", "Cut"), self)
        b3 = QtWidgets.QAction(QtGui.QIcon("img/icons/insert.png"), _translate("MainWindow", "Paste"), self)
        b4 = QtWidgets.QAction(QtGui.QIcon("img/icons/delete.png"), _translate("MainWindow", "Delete"), self)
        b5 = QtWidgets.QAction(QtGui.QIcon("img/icons/undo.png"), _translate("MainWindow", "Undo"), self)
        b1.setShortcut("Ctrl+C")
        b2.setShortcut("Ctrl+X")
        b3.setShortcut("Ctrl+V")
        b4.setShortcut("Delete")
        b5.setShortcut("Ctrl+Z")

        c1 = QtWidgets.QAction(QtGui.QIcon("img/icons/rotate.png"), _translate("MainWindow", "Rotate"), self)
        c2 = QtWidgets.QAction(QtGui.QIcon("img/icons/trim.png"),
                               _translate("MainWindow", "Delete extreme empty blocks"), self)
        c1.setShortcut("Ctrl+R")
        c2.setShortcut("Ctrl+F")

        self.brush_button.setIcon(QtGui.QIcon("img/icons/brush.png"))
        self.brush_button.setCheckable(True)
        self.brush_button.setToolTip("Brush tool")
        self.brush_button.setShortcut("Ctrl+B")

        a1.triggered.connect(self.create_map_triggered)
        a2.triggered.connect(self.open_map_triggered)
        a3.triggered.connect(self.save_map_triggered)
        a4.triggered.connect(self.save_map_as_triggered)

        b1.triggered.connect(self.copy_button_clicked)
        b2.triggered.connect(self.cut_button_clicked)
        b3.triggered.connect(self.insert_button_clicked)
        b4.triggered.connect(self.delete_button_clicked)
        b5.triggered.connect(self.undo_button_clicked)

        c1.triggered.connect(self.rotateSelectedTiles)
        c2.triggered.connect(self.trimClicked)

        self.brush_button.clicked.connect(self.brush_mode)

        for elem in [[a1, a2, a3, a4, a5], [b1, b2, b3, b4, b5]]:
            for act in elem:
                tool_bar.addAction(act)
            tool_bar.addSeparator()
        tool_bar.addWidget(self.brush_button)
        tool_bar.addAction(c1)
        tool_bar.addAction(c2)

        # Setup Layer Tree menu
        self.ui.layer_tree.setModel(QtGui.QStandardItemModel())  # set item model for tree

        #  Customize the Blocks menu
        block_list_widget = self.ui.block_list
        block_list_widget.itemClicked.connect(self.item_list_clicked)
        block_list_widget.itemDoubleClicked.connect(self.item_list_double_clicked)

        #  Customize the Map Editor menu
        default_fill = self.ui.default_fill
        delete_fill = self.ui.delete_fill

        #  Fill out the list
        categories = self.info_json['categories']
        information = self.info_json['info']
        for group in categories:
            # add separator
            icon = "img/icons/galka.png"
            widget = QtWidgets.QListWidgetItem(QtGui.QIcon(icon), self.get_translation(group)['name'])
            widget.setData(0x0100, "separator")
            widget.setData(0x0101, group['id'])
            widget.setBackground(QtGui.QColor(169, 169, 169))
            block_list_widget.addItem(widget)
            # add elements
            for elem_id in group['elements']:
                widget = QtWidgets.QListWidgetItem(QtGui.QIcon(information[elem_id]['icon']),
                                                   self.get_translation(information[elem_id])['name'])
                widget.setData(0x0100, elem_id)
                widget.setData(0x0101, group['id'])
                block_list_widget.addItem(widget)
                # add tiles to fill menu
                if group['id'] in ("road", "block"):
                    default_fill.addItem(QtGui.QIcon(information[elem_id]['icon']),
                                         self.get_translation(information[elem_id])['name'], elem_id)
                    delete_fill.addItem(QtGui.QIcon(information[elem_id]['icon']),
                                        self.get_translation(information[elem_id])['name'], elem_id)

        default_fill.setCurrentText(self.get_translation(information["grass"])['name'])
        delete_fill.setCurrentText(self.get_translation(information["empty"])['name'])

        set_fill = self.ui.set_fill
        set_fill.clicked.connect(self.set_default_fill)

    def change_env(self):
        self.env_form.show()

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    #  Create a new map
    def open_map_triggered(self):
        logger.debug("Creating a new map")
        new_map_dir = QFileDialog.getExistingDirectory(self, 'Open new map', '.',
                                                       QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        if new_map_dir:
            dm2_test = get_new_dt_world(new_map_dir)
            print(dm2_test)
            self.duckie_manager.add_map(new_map_dir.split('/')[-1], dm2_test)
            self.reset_duckietown_map(dm2_test)
            print(new_map_dir.split('/')[-1])
            self.mapviewer.offsetX = self.mapviewer.offsetY = 0
            self.mapviewer.scene().update()
            self.update_layer_tree()

    def import_old_format(self):
        old_format_map = QFileDialog.getOpenFileName(self, 'Open map(old format)', '.')
        path, _ = old_format_map
        print(path)
        with open(path) as file:
            new_format_map = convert_new_format(file.read())
        print(new_format_map)
        map_path = os.getcwd()+"/output"
        try:
            os.makedirs(map_path)
        except:
            pass
        dump(new_format_map)
        dm = get_new_dt_world(map_path)
        self.duckie_manager.add_map(map_path.split('/')[-1], dm)
        self.reset_duckietown_map(dm)
        self.mapviewer.offsetX = self.mapviewer.offsetY = 0
        self.mapviewer.scene().update()
        self.update_layer_tree()

    #  Open map
    def create_map_triggered(self):
        logger.debug(2)
        self.editor.save(self.map)

        def init_info(info):
            i, j = int(info['x']), int(info['y'])
            self.tile_size = float(info['tile_size'])
            self.create_empty_map(i, j)
            self.dm.tile_maps['map_1'].x = self.tile_size
            self.dm.tile_maps['map_1'].y = self.tile_size
            self.mapviewer.tile_size = self.tile_size
            self.mapviewer.scene().update()
            self.update_layer_tree()

        self.init_info_form.send_info.connect(init_info)
        self.init_info_form.show()

        self.mapviewer.offsetX = self.mapviewer.offsetY = 0
        self.mapviewer.scene().update()
        self.update_layer_tree()

    def create_region(self):
        self.region_create = True
        self.new_group_form.show()
        print('Create REGION ', self.region_create)

    def change_distortion_view_triggered(self):
        self.distortion_view_one_string_mode = not self.distortion_view_one_string_mode

    #  Save map
    def save_map_triggered(self):
        self.save_map_as_triggered()

    #  Save map as
    def save_map_as_triggered(self):
        path_folder = save_map_as(self)
        if path_folder:
            map_final = self.dm.dump(self.dm)

            for layer_name in map_final:
                with open(path_folder + f'/{layer_name}.yaml', 'w+') as file:
                    file.write(map_final[layer_name])
            print('FINAL PATH, ', path_folder)

    #  Calculate map characteristics
    def calc_param_triggered(self):
        text = ""
        for info, obj in self.dm.tiles:
            i, j = obj.i, obj.j
            type = obj.type
            orientation = obj.type
            text += f"{i}-{j}: {type}/{orientation}\n"
        self.show_info(self.param_window, _translate("MainWindow", "Map characteristics"), text)

    #  Help: About
    def about_author_triggered(self):
        text = '''
        - Select an object using the left mouse button\n
        - when object is selected you can change pos, using mouse\n
        - add apriltag using key `R`\n
        - Edit an object, click on it using the right mouse button\n
        - Authors:\n alskaa;\n dihindee;\n ovc-serega;\n HadronCollider;\n light5551;\n snush.\n\n Contact us on github!
        '''
        self.show_info(self.author_window, "About", text)

    #  Exit
    def exit_triggered(self):
        self.save_before_exit()
        QtCore.QCoreApplication.instance().quit()

    # Save map before exit
    def save_before_exit(self):
        if not self.debug_mode:
            ret = self.quit_MessageBox()
            if ret == QMessageBox.Cancel:
                return
            if ret == QMessageBox.Save:
                self.save_map_as_triggered()

    #  Hide Block menu
    def change_blocks_toggled(self):
        block = self.ui.block_widget
        if self.ui.change_blocks.isChecked():
            block.show()
            block.setFloating(False)
        else:
            block.close()

    #  Change button state
    def blocks_event(self, event):
        self.ui.change_blocks.setChecked(False)
        event.accept()

    #  Hide information menu
    def change_info_toggled(self):
        block = self.ui.info_widget
        if self.ui.change_info.isChecked():
            block.show()
            block.setFloating(False)
        else:
            block.close()

    #  Change button state
    def info_event(self, event):
        self.ui.change_info.setChecked(False)
        event.accept()

    #  Hide the menu about map properties
    def change_map_toggled(self):
        block = self.ui.map_info_widget
        if self.ui.change_map.isChecked():
            block.show()
            block.setFloating(False)
        else:
            block.close()

    #  Change button state
    def map_event(self, event):
        self.ui.change_map.setChecked(False)
        event.accept()

    # Layer window

    def toggle_layer_window(self):
        """
        Toggle layers window by `View -> Layers`
        :return: -
        """
        block = self.ui.layer_info_widget
        if self.ui.change_layer.isChecked():
            block.show()
            block.setFloating(False)
        else:
            block.close()

    def close_layer_window_event(self, event):
        """
        Reset flag `View -> Layers` when closing layers window
        :param event: closeEvent
        :return: -
        """
        self.ui.change_layer.setChecked(False)
        event.accept()

    def layer_tree_clicked(self):
        pass

    def layer_tree_double_clicked(self):
        pass

    def update_layer_tree(self):
        """
        Update layer tree.
        Show layer's elements as children in hierarchy (except tile layer)
        :return: -
        """

        def signal_check_state(item):
            """
            update visible state of layer.
            :return: -
            """

            dm = self.duckie_manager.get_map(item.text())
            self.reset_duckietown_map(dm)
            self.mapviewer.scene().update()
            layer_tree_view.clearSelection()
            item_model.clear()
            item_model.setHorizontalHeaderLabels(['Maps'])
            self.show_maps_menu(layer_tree_view.model().invisibleRootItem())

        layer_tree_view = self.ui.layer_tree
        item_model = layer_tree_view.model()
        item_model.clear()
        print('waws clear')
        try:
            item_model.itemChanged.disconnect()
        except TypeError:
            pass  # only 1st time in update_layer_tree
        item_model.itemChanged.connect(signal_check_state)
        item_model.setHorizontalHeaderLabels(['Maps'])
        root_item = layer_tree_view.model().invisibleRootItem()

        self.show_maps_menu(root_item)
        layer_tree_view.expandAll()

    def show_maps_menu(self, root_item):
        for map_name in self.duckie_manager.get_maps_name():
            layer_item = QtGui.QStandardItem(str(map_name))
            layer_item.setCheckable(True)
            layer_item.setCheckState(QtCore.Qt.Unchecked)
            layer_item.setCheckState(
                QtCore.Qt.Checked if self.duckie_manager.is_active == map_name else QtCore.Qt.Unchecked)
            root_item.appendRow(layer_item)
            layer_item.sortChildren(0)

    #  MessageBox to exit
    def quit_MessageBox(self):
        reply = QMessageBox(self)
        reply.setIcon(QMessageBox.Question)
        reply.setWindowTitle(_translate("MainWindow", "Exit"))
        reply.setText(_translate("MainWindow", "Exit"))
        reply.setInformativeText(_translate("MainWindow", "Save and exit?"))
        reply.setStandardButtons(QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
        reply.setDefaultButton(QMessageBox.Save)
        ret = reply.exec()
        return ret

    #  Program exit event
    def quit_program_event(self, event):
        self.save_before_exit()

        #  Close additional dialog boxes
        self.author_window.exit()
        self.param_window.exit()
        self.mater_window.exit()

        event.accept()

    def create_empty_map(self, i_size: int, j_size: int) -> None:
        self.mapviewer.i_tile, self.mapviewer.j_tile = i_size, j_size
        for i in range(i_size):
            for j in range(j_size):
                tile = Tile("{}/tile_{}_{}".format(self.dm.get_context(), i, j))
                tile.obj.i = i
                tile.obj.j = j
                tile.frame.pose.x = int(i) * 0.585 + 0.585 / 2
                tile.frame.pose.y = int(j) * 0.585 + 0.585 / 2
                tile.obj.orientation = 'E'
                tile.frame.relative_to = self.dm.get_context()
                tile.frame.dm = self.dm
                self.dm.add(tile)

    #  Handle a click on an item from a list to a list
    def item_list_clicked(self):
        list = self.ui.block_list
        name = list.currentItem().data(0x0100)
        type = list.currentItem().data(0x0101)

        if name == "separator":
            list.currentItem().setSelected(False)

            for i in range(list.count()):
                elem = list.item(i)
                if type == elem.data(0x0101):
                    if name == elem.data(0x0100):
                        icon = QtGui.QIcon("img/icons/galka.png") if list.item(i + 1).isHidden() else \
                            QtGui.QIcon("img/icons/galka_r.png")
                        elem.setIcon(icon)
                    else:
                        elem.setHidden(not elem.isHidden())
        else:
            elem = self.info_json['info'][name]
            info_browser = self.ui.info_browser
            info_browser.clear()
            text = "{}:\n {}\n{}:\n{}".format(_translate("MainWindow", "Name"), list.currentItem().text(),
                                              _translate("MainWindow", "Description"),
                                              self.get_translation(elem)['info'])
            if elem["type"] == "block":
                text += "\n\n{}: {} {}".format(_translate("MainWindow", "Road len"), elem["length"],
                                               _translate("MainWindow", "sm"))
                text += " Tape:\n"
                text += " {}: {} {}\n".format(_translate("MainWindow", "Red"), elem["red"],
                                              _translate("MainWindow", "sm"))
                text += " {}: {} {}\n".format(_translate("MainWindow", "Yellow"), elem["yellow"],
                                              _translate("MainWindow", "sm"))
                text += " {}: {} {}\n".format(_translate("MainWindow", "White"), elem["white"],
                                              _translate("MainWindow", "sm"))
            info_browser.setText(text)

    #  Double click initiates as single click action
    def item_list_double_clicked(self):
        item_ui_list = self.ui.block_list
        item_name = item_ui_list.currentItem().data(0x0100)
        item_type = item_ui_list.currentItem().data(0x0101)

        if item_name == "separator":
            item_ui_list.currentItem().setSelected(False)
        else:
            if item_type in TILE_TYPES:
                self.ui.default_fill.setCurrentText(self.get_translation(self.info_json['info'][item_name])['name'])
                logger.debug("Set {} for brush".format(item_name))
            else:
                # save map before adding object
                self.editor.save(self.map)
                # adding object
                print(item_name)
                type_of_element = self.info_json['info'][item_name]['type']
                obj = None
                if item_name == "duckie":
                    obj = Citizen(f"{self.dm.get_context()}/duckie_{len(self.dm.citizens.dict())}", x=1, y=1)
                elif item_name == "watchtower":
                    name = f"{self.dm.get_context()}/watchtower_{len(self.dm.watchtowers.dict())}"
                    obj = Watchtower(name, x=1, y=1)
                    self.dm.add(Camera(f"{name}/camera"))
                elif type_of_element == "sign":
                    name = f"{self.dm.get_context()}/{get_canonical_sign_name(item_name)}_{len(self.dm.trafficsigns.dict())}"
                    obj = TrafficSign(name, x=1, y=1)
                    obj.obj.type = get_canonical_sign_name(item_name)
                    obj.obj.id = utils.get_id_by_type(item_name)
                elif item_name == "apriltag":
                    name = f"{self.dm.get_context()}/groundtag_{len(self.dm.groundtags.dict())}"
                    obj = GroundTag(name, x=1, y=1)
                elif item_name == "duckiebot":
                    name = f"{self.dm.get_context()}/vehicle_{len(self.dm.vehicles.dict())}"
                    obj = Vehicle(name, x=1, y=1)
                    self.dm.add(Camera(f"{name}/camera"))
                if obj:
                    obj.frame.relative_to = self.dm.get_context()
                    self.dm.add(obj)

                # TODO: need to understand what's the type and create desired class, not general
                # also https://github.com/moevm/mse_visual_map_editor_for_duckietown/issues/122
                # (for args, that can be edited and be different between classes)
                self.mapviewer.scene().update()
                logger.debug("Add {} to map".format(item_name))
            self.update_layer_tree()

    #  Reset to default values
    def set_default_fill(self):
        default_fill = self.ui.default_fill.currentData()
        delete_fill = self.ui.delete_fill.currentData()
        # TODO установка занчений по умолчанию
        logger.debug("{}; {}".format(default_fill, delete_fill))

    #  Copy
    def copy_button_clicked(self):
        if self.brush_button.isChecked():
            self.brush_button.click()
        self.drawState = 'copy'
        self.copyBuffer = copy.copy(self.mapviewer.tileSelection)
        logger.debug("Copy")

    #  Cut
    def cut_button_clicked(self):
        if self.brush_button.isChecked():
            self.brush_button.click()
        self.drawState = 'cut'
        self.copyBuffer = copy.copy(self.mapviewer.tileSelection)
        logger.debug("Cut")

    #  Paste
    def insert_button_clicked(self):
        if len(self.copyBuffer) == 0:
            return
        self.editor.save(self.map)
        if self.drawState == 'copy':
            self.editor.copySelection(self.copyBuffer, self.mapviewer.tileSelection[0], self.mapviewer.tileSelection[1],
                                      MapTile(self.ui.delete_fill.currentData()))
        elif self.drawState == 'cut':
            self.editor.moveSelection(self.copyBuffer, self.mapviewer.tileSelection[0], self.mapviewer.tileSelection[1],
                                      MapTile(self.ui.delete_fill.currentData()))
        self.mapviewer.scene().update()
        self.update_layer_tree()

    #  Delete
    def delete_button_clicked(self):
        if not self.map.get_tile_layer().visible:
            return
        self.mapviewer.remove_last_obj()
        self.mapviewer.scene().update()
        self.update_layer_tree()

    #  Undo
    def undo_button_clicked(self):
        self.editor.undo()
        self.mapviewer.scene().update()
        self.update_layer_tree()

    #  Brush mode
    def brush_mode(self):
        if self.brush_button.isChecked():
            self.drawState = 'brush'
        else:
            self.drawState = ''

    def keyPressEvent(self, e):
        selection = self.mapviewer.raw_selection
        item_layer = self.map.get_objects_from_layers()  # TODO: add self.current_layer for editing only it's objects?
        new_selected_obj = False
        for layer in self.dm:
            print(layer)
        print(selection)
        if self.region_create:
            self.region_create = False

        for item in item_layer:
            x, y = item.position
            if x > selection[0] and x < selection[2] and y > selection[1] and y < selection[3]:
                if item not in self.active_items:
                    self.active_items.append(item)
                    new_selected_obj = True
        if new_selected_obj:
            # save map if new objects are selected
            self.editor.save(self.map)
        key = e.key()
        print('KEY ', key,  " ", QtCore.Qt.ALT, " ", e.modifiers())
        if key == QtCore.Qt.Key_Q:
            # clear object buffer
            self.active_items = []
            self.mapviewer.raw_selection = [0] * 4
        elif key == QtCore.Qt.Key_R:
            self.new_tag_class.create_form()
        elif key == QtCore.Qt.Key_H:
            # print(self.duckie_manager.get_maps_name())
            for map_name in self.duckie_manager.get_maps_name():
                if map_name == "maps/test":
                    self.reset_duckietown_map(self.duckie_manager.get_map(map_name))

        if self.active_items:
            if key == QtCore.Qt.Key_Backspace:
                # delete object
                if question_form_yes_no(self, "Deleting objects", "Delete objects from map?") == QMessageBox.Yes:
                    # save map before deleting objects
                    self.editor.save(self.map)
                    for item in self.active_items:
                        object_type = self.info_json['info'][item.kind]['type']
                        layer = self.map.get_layer_by_type(get_layer_type_by_object_type(object_type))
                        layer.remove_object_from_layer(item)
                    self.active_items = []
                    self.mapviewer.scene().update()
                    self.update_layer_tree()
                return
            for item in self.active_items:
                logger.debug("Name of item: {}; X - {}; Y - {};".format(item.kind, item.position[0], item.position[1]))
                if key == QtCore.Qt.Key_W:
                    item.position[1] -= EPS
                elif key == QtCore.Qt.Key_S:
                    item.position[1] += EPS
                elif key == QtCore.Qt.Key_A:
                    item.position[0] -= EPS
                elif key == QtCore.Qt.Key_D:
                    item.position[0] += EPS
                elif key == QtCore.Qt.Key_E:
                    if len(self.active_items) == 1:
                        self.create_form(self.active_items[0])
                    else:
                        logger.debug("I can't edit more than one object!")
        self.mapviewer.scene().update()

    def create_form(self, active_object_data: tuple):

        active_object, (name, tp) = active_object_data
        print(name, 'GFDGFDGFGFGFGF')
        self.name_of_editable_obj = name
        assert tp is _Frame

        def accept():
            active_object.pose.x = float(edit_obj['x'].text())
            active_object.pose.y = float(edit_obj['y'].text())
            active_object.pose.yaw = float(np.deg2rad(float(edit_obj['yaw'].text())))
            new_type = None
            print(f"ACCEPT: {cam_obj}")
            for key in editable_values:
                try:
                    print("Key - ", key)
                    if key in ["width", "height", "framerate", "distortion_parameters", "camera_matrix"]:  # cam obj
                        if key in ["width", "height", "framerate"]:
                            cam_obj[key] = int(edit_obj[key].text().split()[0])
                        elif key == "distortion_parameters":
                            if not cam_obj.distortion_parameters:
                                cam_obj["distortion_parameters"] = [] #[0 for _ in range(5)]

                            if self.distortion_view_one_string_mode:
                                dk = edit_obj[f"distortion_parameters"].text().replace(" ", "").split(",")
                                for idx, val in enumerate(dk):
                                    val = float(val)
                                    if len(cam_obj.distortion_parameters) < 5:
                                        cam_obj.distortion_parameters.append(val)
                                    else:
                                        cam_obj.distortion_parameters[idx] = val
                            else:
                                for idx in range(5):
                                    val = float(edit_obj[f"distortion_parameters_{idx}"].text().split()[0])
                                    if len(cam_obj.distortion_parameters) < 5:
                                        cam_obj.distortion_parameters.append(val)#.insert(idx, val)
                                    else:
                                        cam_obj.distortion_parameters[idx] = val
                        elif key == "camera_matrix":
                            print('MATRIX1   ', cam_obj.camera_matrix)
                            if not cam_obj.camera_matrix:
                                cam_obj["camera_matrix"] = []
                            print('MATRIX2   ', cam_obj.camera_matrix)
                            if not cam_obj.camera_matrix:
                                for _ in range(3):
                                    cam_obj.camera_matrix.append([])
                            print('MATRIX3   ', cam_obj.camera_matrix)
                            for row in range(3):
                                for col in range(3):
                                    val = float(edit_obj[f"camera_matrix_{row}_{col}"].text().split()[0])
                                    if len(cam_obj.camera_matrix[row]) < 3:
                                        cam_obj.camera_matrix[row].append(val)
                                    else:
                                        cam_obj.camera_matrix[row][col] = val
                            print('MATRIX1   ', cam_obj.camera_matrix)
                    else:
                        new_value = edit_obj[key].text().split()[0]
                        if key == 'id' and len(edit_obj[key].text().split()) > 1:
                            new_type = edit_obj[key].text().split()[1][1:-1]
                        if key == 'type' and new_type:
                            obj[key] = get_canonical_sign_name(new_type)  # new_type
                            continue
                        if new_value.isdigit():
                            new_value = int(new_value)
                        try:
                            if isinstance(new_value, str):
                                new_value = float(new_value)
                        except ValueError:
                            pass
                        print(f"New value {new_value} for key {key}")
                        obj[key] = new_value
                except Exception as e:
                    print(e)
            dialog.close()
            self.mapviewer.scene().update()
            self.update_layer_tree()

        def reject():
            dialog.close()

        # work version
        info_object = self.dm.get_objects_by_name(name)
        del info_object[(name, tp)]
        _, type_object = list(info_object.keys())[0]
        obj = info_object[(name, type_object)]
        cam_obj = None

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('Change attribute of object')
        # buttonbox
        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttonBox.accepted.connect(accept)
        buttonBox.rejected.connect(reject)
        # form
        formGroupBox = QGroupBox("Change attribute's object: {}".format(""))

        layout = QFormLayout()
        # editable_attrs = active_object.get_editable_attrs()
        edit_obj = {}
        combo_id = QComboBox(self)

        def change_combo_id(value):
            combo_id.clear()
            combo_id.addItems(["{}/{}".format(i.id, i.type) for i in self.duckietown_types_apriltags[value]])
            combo_id.setEditText(str(self.duckietown_types_apriltags[value][0]))

        def change_type_from_combo(value: str):
            if 'type' in edit_obj:
                edit_obj['type'].setText(value.split()[1][1:-1])

        x_edit = QLineEdit(str(active_object.pose.x))
        y_edit = QLineEdit(str(active_object.pose.y))
        yaw_edit = QLineEdit(str(np.rad2deg(active_object.pose.yaw)))
        edit_obj['x'] = x_edit
        edit_obj['y'] = y_edit
        edit_obj['yaw'] = yaw_edit
        layout.addRow(QLabel("{}.X".format("pose")), x_edit)
        layout.addRow(QLabel("{}.Y".format("pose")), y_edit)
        layout.addRow(QLabel("{}.yaw".format("pose")), yaw_edit)

        editable_values = obj.dict()
        print('ATTR SHOW: ', editable_values, name)

        for attr_name in sorted(editable_values.keys()):
            attr = editable_values[attr_name]
            new_edit = QLineEdit(str(attr))

            edit_obj[attr_name] = new_edit

            if "vehicle" not in name and attr_name == 'id':
                type_id = list(self.duckietown_types_apriltags.keys())[0]
                for type_sign in self.duckietown_types_apriltags.keys():
                    try:
                        if int(attr) in self.duckietown_types_apriltags[type_sign]:
                            type_id = type_sign
                            break
                    except:
                        pass
                print(type_id)
                combo_id.addItems(["{} ({})".format(i.id, i.type) for i in self.duckietown_types_apriltags[type_id]])
                combo_id.setLineEdit(new_edit)
                combo_id.setEditText(str(attr))
                combo_id.currentTextChanged.connect(change_type_from_combo)
                layout.addRow(QLabel(attr_name), combo_id)
            elif attr_name == 'id':
                new_edit.setReadOnly(True)
                layout.addRow(QLabel("{}".format(attr_name)), new_edit)
            elif attr_name == "type":
                new_edit.setReadOnly(True)
                layout.addRow(QLabel("{}".format(attr_name)), new_edit)
            else:
                layout.addRow(QLabel("{}".format(attr_name)), new_edit)

        if "vehicle" in name:
            cam = self.dm.get_objects_by_name(name + "/camera")
            cam_obj: _Camera = cam[list(cam.keys())[0]]
            editable_values.update(cam_obj.dict())
            print("Camera info ", type(cam), cam.keys(), list(cam.keys())[0], cam)
            print(cam_obj)
            layout.addRow(QHLine())


            width_camera_edit = QLineEdit(str(cam_obj.width))
            height_camera_edit = QLineEdit(str(cam_obj.height))
            framerate_camera_edit = QLineEdit(str(cam_obj.framerate))
            edit_obj.update({
                "width": width_camera_edit,
                "height": height_camera_edit,
                "framerate": framerate_camera_edit
            })
            layout.addRow(QLabel("{}.width".format("camera")), width_camera_edit)
            layout.addRow(QLabel("{}.height".format("camera")), height_camera_edit)
            layout.addRow(QLabel("{}.framerate".format("camera")), framerate_camera_edit)

            layout.addRow(QLabel("Camera Matrix"))
            grid_matrix = QGridLayout()
            grid_matrix.setColumnStretch(1, 4)
            grid_matrix.setColumnStretch(2, 4)
            for row in range(3):
                for col in range(3):
                    if cam_obj.camera_matrix:
                        # TODO: NEED TO TEST
                        grid_line_edit = QLineEdit(str(cam_obj.camera_matrix[row][col]))
                    else:
                        grid_line_edit = QLineEdit("0")
                    edit_obj[f"camera_matrix_{row}_{col}"] = grid_line_edit
                    grid_matrix.addWidget(grid_line_edit, row, col)

            layout.addRow(grid_matrix)
            #layout.addRow(QLabel("Camera Distortion"))
            grid_distortion = QGridLayout()
            grid_distortion.setColumnStretch(1, 4)
            grid_distortion.setColumnStretch(2, 4)
            if self.distortion_view_one_string_mode:
                dist_k = []
                for idx in range(5):
                    if cam_obj.distortion_parameters:
                        str_k = str(cam_obj.distortion_parameters[idx]) + (", " if idx != 4 else "")
                    else:
                        str_k = "0" + (", " if idx != 4 else "")
                    dist_k.append(str_k)
                #for kfc in dist_k:
                edit_str_dist = QLineEdit("".join(dist_k))
                edit_obj["distortion_parameters"] = edit_str_dist
                layout.addRow(QLabel("Camera Distortion"), edit_str_dist)
            else:
                layout.addRow(QLabel("Camera Distortion"))
                for idx in range(5):
                    if cam_obj.distortion_parameters:
                        grid_line_edit = QLineEdit(str(cam_obj.distortion_parameters[idx]))
                    else:
                        grid_line_edit = QLineEdit("0")
                    edit_obj[f"distortion_parameters_{idx}"] = grid_line_edit
                    grid_distortion.addWidget(grid_line_edit, 0, idx)

                layout.addRow(grid_distortion)
        layout.addRow(QHLine())
        combo_groups = QComboBox(self)
        ##### DEV FOR GROUP ####
        groups = ["No chosen"]
        for ((nm, _), group) in self.dm.groups:
            groups.append(f"{nm} [{group.description}]")
            print(nm, group.description)
        combo_groups.addItems([i for i in groups])

        combo_groups.setEditText("Choose group")
        combo_groups.currentTextChanged.connect(self.change_active_group)
        layout.addRow(QLabel("Choose group"), combo_groups)
        add_group = QPushButton("Add in group", self)
        add_group.clicked.connect(self.add_group_triggered)
        del_group = QPushButton("Del in group", self)
        del_group.clicked.connect(self.del_group_triggered)
        hl = QHBoxLayout()
        hl.addWidget(add_group)
        hl.addWidget(del_group)
        layout.addRow(hl)
        ########################
        formGroupBox.setLayout(layout)
        # layout
        mainLayout = QVBoxLayout()
        mainLayout.addWidget(formGroupBox)
        mainLayout.addWidget(buttonBox)
        dialog.setLayout(mainLayout)
        dialog.exec_()

    def add_group_triggered(self):
        print(self.active_group)
        print(self.name_of_editable_obj)
        members = self.active_group.members
        if self.name_of_editable_obj not in members:
            members.append(self.name_of_editable_obj)

    def del_group_triggered(self):
        if self.name_of_editable_obj in self.active_group.members:
            self.active_group.members.remove(self.name_of_editable_obj)

    def change_active_group(self, value: str):
        print(value)
        print(self.dm.get_object(value.split()[0], _Group))
        self.active_group = self.dm.get_object(value.split()[0], _Group)

    def rotateSelectedTiles(self):
        self.editor.save(self.map)
        is_selected_tile = self.mapviewer.is_selected_tile
        for ((nm, _), tile) in self.dm.tiles:
            if is_selected_tile(tile):
                frame: _Frame = self.dm.frames[nm]
                orien_val = get_degree_for_orientation(tile.orientation) - 90  # (rot_val[tile.orientation] + 90) % 360
                tile.orientation = get_orientation_for_degree(orien_val)
                frame.pose.yaw = {'E': np.pi * 1.5, 'N': 0, 'W': np.pi, 'S': np.pi * 0.5, None: 0}[tile.orientation]
        self.mapviewer.scene().update()

    def add_apriltag(self, apriltag: GroundAprilTagObject):
        layer = self.map.get_layer_by_type(LayerType.GROUND_APRILTAG)
        if layer is None:
            self.map.add_layer_from_data(LayerType.GROUND_APRILTAG, [apriltag])
        else:
            self.map.add_elem_to_layer_by_type(LayerType.GROUND_APRILTAG, apriltag)
        self.update_layer_tree()
        self.mapviewer.scene().update()

    def trimClicked(self):
        self.editor.save(self.map)
        self.editor.trimBorders(True, True, True, True, MapTile(self.ui.delete_fill.currentData()))
        self.mapviewer.scene().update()
        self.update_layer_tree()

    def selectionUpdate(self):
        is_selected_tile = self.mapviewer.is_selected_tile
        if self.drawState == 'brush':
            self.editor.save(self.map)  # TODO: CTRL+Z need to fix because dt-world
            tiles = self.dm.tiles.only_tiles()
            for i in range(len(tiles)):
                for j in range(len(tiles[0])):
                    tile = tiles[i][j]
                    if is_selected_tile(tile):
                        tile.type = self.ui.default_fill.currentData()
                        tile.orientation = 'E'
        self.update_layer_tree()
        self.mapviewer.scene().update()

    def reset_duckietown_map(self, new_dm: DuckietownMap):
        self.dm = new_dm
        self.mapviewer.dm = new_dm
        # self.update_layer_tree()
        self.mapviewer.scene().update()

    def get_random_name(self, begin):
        return "{}_{}".format(
            begin,
            np.random.randint(1000)
        )

    def show_info(self, name, title, text):
        name.set_window_name(title)
        name.set_text(text)
        name.show()
