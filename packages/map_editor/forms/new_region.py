from PyQt5.QtWidgets import (QApplication, QComboBox, QDialog,
                             QDialogButtonBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QLineEdit, QMenu, QMenuBar, QPushButton, QSpinBox, QTextEdit,
                             QVBoxLayout)

from duckietown_world.structure.objects import Group
from DTWorld import get_dt_world
import sys


class NewGroupForm(QDialog):
    NumGridRows = 3
    NumButtons = 4

    def __init__(self):
        super(NewGroupForm, self).__init__()
        self.nameLineEdit = QLineEdit()
        self.descriptionLineEdit = QLineEdit()
        self.createFormGroupBox()


        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttonBox.accepted.connect(self.get_info)
        buttonBox.rejected.connect(self.reject)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.formGroupBox)
        mainLayout.addWidget(buttonBox)
        self.setLayout(mainLayout)

        self.setWindowTitle("Create New Group")

    def createFormGroupBox(self):
        self.formGroupBox = QGroupBox("Info about group")
        layout = QFormLayout()
        layout.addRow(QLabel("Name:"), self.nameLineEdit)
        layout.addRow(QLabel("Description:"), self.descriptionLineEdit)
        self.formGroupBox.setLayout(layout)

    def get_info(self):
        name_group = self.nameLineEdit.text()
        description_group = self.descriptionLineEdit.text()
        dm = get_dt_world()
        group = Group(name_group)
        group.obj.description = description_group
        dm.add(group)
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = NewGroupForm()
    sys.exit(dialog.exec_())
