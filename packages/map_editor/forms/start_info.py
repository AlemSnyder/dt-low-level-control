from PyQt5 import QtCore
from PyQt5.QtGui import QIntValidator, QDoubleValidator
from PyQt5.QtWidgets import QDialog, QGroupBox, QDialogButtonBox, QFormLayout, QVBoxLayout, \
    QLineEdit, QLabel


class StartInfoForm(QDialog):
    send_info = QtCore.pyqtSignal(object)

    def __init__(self):
        super(StartInfoForm, self).__init__()
        self.setWindowTitle("Info for initialization of map")
        self.formGroupBox = QGroupBox("Init info")
        self.nameXEdit = QLineEdit(self)
        self.nameXEdit.setText("5")
        self.nameXEdit.setValidator(QIntValidator())
        self.nameYEdit = QLineEdit(self)
        self.nameYEdit.setText("5")
        self.nameYEdit.setValidator(QIntValidator())
        self.nameTileSizeEdit = QLineEdit(self)
        self.nameTileSizeEdit.setText("0.585")
        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttonBox.accepted.connect(self.get_info)
        self.buttonBox.rejected.connect(self.reject)
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(self.formGroupBox)
        main_layout.addWidget(self.buttonBox)
        self.create_form()
        self.setLayout(main_layout)

    def get_info(self):
        info = {
            'x': self.nameXEdit.text(),
            'y': self.nameYEdit.text(),
            'tile_size': self.nameTileSizeEdit.text()
        }
        self.send_info.emit(info)
        self.close()

    def create_form(self):
        layout = QFormLayout()
        layout.addRow(QLabel("Width"), self.nameXEdit)
        layout.addRow(QLabel("Height"), self.nameYEdit)
        layout.addRow(QLabel("Tile size"), self.nameTileSizeEdit)
        self.formGroupBox.setLayout(layout)
