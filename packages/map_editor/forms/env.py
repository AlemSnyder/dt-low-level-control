import sys
from PyQt5.QtWidgets import (QApplication, QDialog,
                             QDialogButtonBox, QFormLayout, QGroupBox, QLabel, QLineEdit, QVBoxLayout)

from DTWorld import get_dt_world
from duckietown_world.structure.objects import Group, Environment


class EnvForm(QDialog):
    NumGridRows = 3
    NumButtons = 4

    def __init__(self):
        super(EnvForm, self).__init__()
        self.dateLineEdit = QLineEdit()
        self.locationLineEdit = QLineEdit()
        self.weatherLineEdit = QLineEdit()
        self.createFormGroupBox()


        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttonBox.accepted.connect(self.get_info)
        buttonBox.rejected.connect(self.reject)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.formGroupBox)
        mainLayout.addWidget(buttonBox)
        self.setLayout(mainLayout)

        self.setWindowTitle("Environment")

    def createFormGroupBox(self):
        self.formGroupBox = QGroupBox("Environment")
        layout = QFormLayout()
        layout.addRow(QLabel("date:"), self.dateLineEdit)
        layout.addRow(QLabel("location:"), self.locationLineEdit)
        layout.addRow(QLabel("weather:"), self.weatherLineEdit)
        self.formGroupBox.setLayout(layout)

    def get_info(self):
        env = None
        date = self.dateLineEdit.text()
        location = self.locationLineEdit.text()
        weather = self.weatherLineEdit.text()
        dm = get_dt_world()

        for ((nm, _), _env) in dm.environment:
            env = _env
            env.weather = weather
            env.location = location
            env.date = date

        if not env:
            env = Environment("environment")
            dm.add(env)
            env.obj.weather = weather
            env.obj.location = location
            env.obj.date = date

        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = EnvForm()
    sys.exit(dialog.exec_())
