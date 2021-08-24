import argparse
import os
import cv2
import sys
import rospy
import numpy as np
from cv_bridge import CvBridge
from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QLabel, QApplication, QComboBox, QVBoxLayout, QHBoxLayout, \
    QPushButton, QSplitter, QGridLayout
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPixmap, QIcon
from datetime import datetime
from sensor_msgs.msg import CompressedImage


PICTURES_DIR = "/data/pictures"


class ROSSpin(QThread):
    changeROSImage = pyqtSignal(np.ndarray)

    def __init__(self, robot_name: str):
        super().__init__()
        rospy.init_node('ImageCreator', anonymous=False)
        self.bridge = CvBridge()
        self.image_compressed = rospy.Subscriber(f"/{robot_name}/camera_node/image/compressed",
                                                 CompressedImage, self.callback, queue_size=1)

    def callback(self, picture: CompressedImage):
        image = self.bridge.compressed_imgmsg_to_cv2(picture, "bgra8")
        self.changeROSImage.emit(image)


class App(QWidget):

    def __init__(self, robot_name: str):
        super().__init__()
        self.w, self.h = 640, 480
        self.filter_image = None
        self.mask_image = None
        script_path = os.path.dirname(__file__)
        self.script_path = (script_path + "/") if script_path else ""
        self.save_image = None
        # gui components
        self.top_layout = None
        self.middle_layout = None
        self.label_filter = None
        self.label_mask = None
        self.bot_layout = None
        self.ros = None
        # ---
        self.initUI()
        self.init_ROS(robot_name)

    @pyqtSlot(np.ndarray)
    def setImage(self, image: np.ndarray):
        self.callback(image)

    def callback(self, bot_image: np.ndarray):
        bot_image = cv2.resize(bot_image, (self.w, self.h))
        if self.mask_image is not None:
            mask_image = self.convert_cv_qt(self.add_mask(bot_image))
        else:
            mask_image = self.convert_cv_qt(bot_image)
        self.save_image = bot_image
        if self.filter_image is not None:
            self.save_image = self.add_filter(bot_image)
            filter_image = self.convert_cv_qt(self.save_image)
        else:
            filter_image = self.convert_cv_qt(bot_image)
        self.label_mask.setPixmap(mask_image)
        self.label_filter.setPixmap(filter_image)

    def add_mask(self, img):
        return cv2.addWeighted(img, 1, self.mask_image, 1, 0)

    def add_filter(self, img1):
        img2 = self.filter_image
        rows,cols,channels = img2.shape
        roi = img1[0:rows, 0:cols ]
        
        img2gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(img2gray, 0, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)

        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
        
        dst = cv2.add(img1_bg,img2_fg)
        img1[0:rows, 0:cols ] = dst
        return img1

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2RGBA)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGBX8888)
        p = convert_to_Qt_format.scaled(self.w, self.h, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
    
    def initUI(self):
        self.setWindowTitle("Duckietown Social App")
        self.setWindowIcon(QIcon(os.path.join(self.script_path, 'images', 'icon.png')))
        self.setGeometry(300, 300, self.w + 300, self.h + 200)
        self.resize(2 * self.w, int(1.2 * self.h))
        self.setFixedSize(self.size())
        windowLayout = QVBoxLayout()
        self.create_top_layout()
        self.create_middle_layout()
        self.create_bot_layout()
        windowLayout.addLayout(self.top_layout)
        windowLayout.addLayout(self.middle_layout)
        windowLayout.addLayout(self.bot_layout)
        self.setLayout(windowLayout)

    def createComboBox(self):
        combo = QComboBox(self)
        filters = []
        for filter in os.listdir(self.script_path + "filters"):
            if os.path.isdir(self.script_path + 'filters/' + filter):
                filters.append(filter.replace('_', ' ').title())
        combo.addItems(filters)
        combo.setCurrentIndex(filters.index('None'))
        combo.activated[str].connect(self.changed_filter)
        return combo

    def changed_filter(self, text):
        path = self.script_path + 'filters/' + text.lower().replace(' ', '') + "/"
        mask, filter = path + "mask.png", path + "filter.png"
        self.mask_image = cv2.imread(mask, cv2.IMREAD_UNCHANGED)
        self.mask_image = cv2.resize(self.mask_image, (self.w, self.h))
        self.filter_image = cv2.imread(filter, cv2.IMREAD_UNCHANGED)
        self.filter_image = cv2.resize(self.filter_image, (self.w, self.h))

    def create_top_layout(self):
        self.top_layout = QGridLayout()
        self.top_layout.addWidget(QLabel('Filter:', self), 0, 0, 1, 1, Qt.AlignRight)
        self.top_layout.addWidget(self.createComboBox(), 0, 1, 1, 19)
        self.top_layout.addWidget(QLabel('Mask:', self), 1, 0)
        self.top_layout.addWidget(QLabel('Result:', self), 1, 10)

    def create_middle_layout(self):
        self.middle_layout = QHBoxLayout()
        # filter label
        self.label_filter = QLabel(self)
        self.label_filter.resize(self.w, self.h)
        # mask label
        self.label_mask = QLabel(self)
        self.label_mask.resize(self.w, self.h)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.label_mask)
        splitter.addWidget(self.label_filter)
        self.middle_layout.addWidget(splitter)

    def create_bot_layout(self):
        self.bot_layout = QHBoxLayout()
        button_save = QPushButton('Take picture', self)
        button_save.clicked.connect(self.save_picture)
        self.bot_layout.addWidget(button_save)

    def save_picture(self):
        if self.save_image is not None:
            filename = datetime.now().strftime('%m_%d_%Y_%H_%M_%S')
            path = os.path.join(PICTURES_DIR, f"dt-social-{filename}.jpg")
            cv2.imwrite(path, self.save_image)
            os.chmod(path, 0o777)
        print('SAVE PICTURE')

    def init_ROS(self, robot_name):
        self.ros = ROSSpin(robot_name)
        self.ros.changeROSImage.connect(self.setImage)
        self.ros.start()

    @staticmethod
    def overlay_transparent(bg_img, fg_img):
        bg_img = cv2.cvtColor(bg_img, cv2.COLOR_BGRA2BGR)
        b, g, r, mask = cv2.split(fg_img)
        overlay = cv2.merge((b, g, r))
        img1_bg = cv2.bitwise_and(bg_img, bg_img, mask=cv2.bitwise_not(mask))
        img2_fg = cv2.bitwise_and(overlay, overlay, mask=mask)
        return cv2.add(img1_bg, img2_fg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", type=str, help="Robot's name")
    parsed = parser.parse_args()
    # ---
    app = QApplication(sys.argv)
    app.setApplicationName("Duckietown Social App")
    window = App(parsed.robot)
    window.show()
    app.exec_()
    window.ros.terminate()
    window.ros.wait()


if __name__ == '__main__':
    os.makedirs(PICTURES_DIR, exist_ok=True)
    main()