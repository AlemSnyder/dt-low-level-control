# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_info_widget(object):
    def setupUi(self, info_widget):
        info_widget.setObjectName("info_widget")
        info_widget.resize(469, 355)
        self.verticalLayout = QtWidgets.QVBoxLayout(info_widget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.text_browser = QtWidgets.QTextBrowser(info_widget)
        self.text_browser.setMinimumSize(QtCore.QSize(451, 337))
        self.text_browser.setObjectName("text_browser")
        self.verticalLayout.addWidget(self.text_browser)

        self.retranslateUi(info_widget)
        QtCore.QMetaObject.connectSlotsByName(info_widget)

    def retranslateUi(self, info_widget):
        _translate = QtCore.QCoreApplication.translate
        info_widget.setWindowTitle(_translate("MainWindow", "Reference"))


#  Class for info_window
class info_window(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(info_window, self).__init__(parent)
        self.ui = Ui_info_widget()
        self.ui.setupUi(self)

    def set_window_name(self, title):
        self.setWindowTitle(title)

    def set_text(self, text):
        self.ui.text_browser.setText(text)

    def exit(self):
        self.close()
