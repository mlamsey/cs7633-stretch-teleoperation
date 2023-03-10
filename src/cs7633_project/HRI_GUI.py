# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'HRI_GUI.ui'
#
# Created by: Sophie Yang

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1086, 489)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.forward_drive = QtWidgets.QPushButton(self.centralwidget)
        self.forward_drive.setGeometry(QtCore.QRect(130, 80, 91, 91))
        self.forward_drive.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.forward_drive.setObjectName("forward_drive")
        self.forward_drive.setStyleSheet(
        "QPushButton"
        "{"
        "background:lightblue;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightblue;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.backward_drive = QtWidgets.QPushButton(self.centralwidget)
        self.backward_drive.setGeometry(QtCore.QRect(130, 280, 91, 91))
        self.backward_drive.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.backward_drive.setObjectName("backward_drive")
        self.backward_drive.setStyleSheet(
         "QPushButton"
        "{"
        "background:lightblue;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightblue;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.left_drive = QtWidgets.QPushButton(self.centralwidget)
        self.left_drive.setGeometry(QtCore.QRect(30, 180, 91, 91))
        self.left_drive.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.left_drive.setObjectName("left_drive")
        self.left_drive.setStyleSheet(
        "QPushButton"
        "{"
        "background:lightblue;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightblue;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.right_drive = QtWidgets.QPushButton(self.centralwidget)
        self.right_drive.setGeometry(QtCore.QRect(230, 180, 91, 91))
        self.right_drive.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.right_drive.setObjectName("right_drive")
        self.right_drive.setStyleSheet(
        "QPushButton"
        "{"
        "background:lightblue;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightblue;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.grasp = QtWidgets.QPushButton(self.centralwidget)
        self.grasp.setGeometry(QtCore.QRect(910, 140, 131, 91))
        self.grasp.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.grasp.setObjectName("grasp")
        self.grasp.setStyleSheet(
           "QPushButton"
        "{"
        "background:#FFE4E1	;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px #FFE4E1;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.release = QtWidgets.QPushButton(self.centralwidget)
        self.release.setGeometry(QtCore.QRect(910, 250, 131, 91))
        self.release.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.release.setObjectName("release")
        self.release.setStyleSheet(
       "QPushButton"
        "{"
        "background:#FFE4E1;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px #FFE4E1;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.drive_label = QtWidgets.QLabel(self.centralwidget)
        self.drive_label.setGeometry(QtCore.QRect(120, 10, 171, 50))
        self.drive_label.setObjectName("drive_label")
        self.drive_label.setStyleSheet("font-size:20px;")
        self.manipulate_label = QtWidgets.QLabel(self.centralwidget)
        self.manipulate_label.setGeometry(QtCore.QRect(670, 10, 171, 50))
        self.manipulate_label.setObjectName("manipulate_label")
        self.manipulate_label.setStyleSheet("font-size:20px;")
        self.backward_manipulate = QtWidgets.QPushButton(self.centralwidget)
        self.backward_manipulate.setGeometry(QtCore.QRect(810, 250, 91, 91))
        self.backward_manipulate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.backward_manipulate.setObjectName("backward_manipulate")
        self.backward_manipulate.setStyleSheet(
        "QPushButton"
        "{"
        "background:lightyellow;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightyellow;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.forward_manipulate = QtWidgets.QPushButton(self.centralwidget)
        self.forward_manipulate.setGeometry(QtCore.QRect(810, 140, 91, 91))
        self.forward_manipulate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.forward_manipulate.setObjectName("forward_manipulate")
        self.forward_manipulate.setStyleSheet(
       "QPushButton"
        "{"
        "background:lightyellow;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightyellow;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.idle_drive = QtWidgets.QPushButton(self.centralwidget)
        self.idle_drive.setGeometry(QtCore.QRect(130, 180, 91, 91))
        self.idle_drive.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.idle_drive.setObjectName("idle_drive")
        self.idle_drive.setStyleSheet(
        "QPushButton"
        "{"
        "background:lightgrey;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightgrey;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        # self.pushButton_14 = QtWidgets.QPushButton(self.centralwidget)
        # self.pushButton_14.setGeometry(QtCore.QRect(630, 590, 141, 91))
        # self.pushButton_14.setObjectName("pushButton_14")
        self.down_manipulate = QtWidgets.QPushButton(self.centralwidget)
        self.down_manipulate.setGeometry(QtCore.QRect(570, 280, 91, 91))
        self.down_manipulate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.down_manipulate.setObjectName("down_manipulate")
        self.down_manipulate.setStyleSheet(
        "QPushButton"
        "{"
        "background:lightyellow;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightyellow;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.right_manipulate = QtWidgets.QPushButton(self.centralwidget)
        self.right_manipulate.setGeometry(QtCore.QRect(670, 180, 91, 91))
        self.right_manipulate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.right_manipulate.setObjectName("right_manipulate")
        self.right_manipulate.setStyleSheet(
         "QPushButton"
        "{"
        "background:lightyellow;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightyellow;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.idle_manipulate = QtWidgets.QPushButton(self.centralwidget)
        self.idle_manipulate.setGeometry(QtCore.QRect(570, 180, 91, 91))
        self.idle_manipulate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.idle_manipulate.setObjectName("idle_manipulate")
        self.idle_manipulate.setStyleSheet(
       "QPushButton"
        "{"
        "background:lightgrey;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightgrey;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        
        self.up_manipulate = QtWidgets.QPushButton(self.centralwidget)
        self.up_manipulate.setGeometry(QtCore.QRect(570, 80, 91, 91))
        self.up_manipulate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.up_manipulate.setObjectName("up_manipulate")
        self.up_manipulate.setStyleSheet(
           "QPushButton"
        "{"
        "background:lightyellow;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightyellow;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        self.left_manipulate = QtWidgets.QPushButton(self.centralwidget)
        self.left_manipulate.setGeometry(QtCore.QRect(470, 180, 91, 91))
        self.left_manipulate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.left_manipulate.setObjectName("left_manipulate")
        self.left_manipulate.setStyleSheet(
         "QPushButton"
        "{"
        "background:lightyellow;\n"
        "color:black;\n"
        "border-radius:25px;\n"
        "font-size:15px;"
        "border: 3px lightyellow;"
        "}"
        "QPushButton::hover"
        "{"
        "border: 3px solid black"
        "}"
        )
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1086, 24))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.forward_drive.setText(_translate("MainWindow", "Forward"))
        self.backward_drive.setText(_translate("MainWindow", "Backward"))
        self.left_drive.setText(_translate("MainWindow", "Turn Left"))
        self.right_drive.setText(_translate("MainWindow", "Turn Right"))
        self.grasp.setText(_translate("MainWindow", "Grasp"))
        self.release.setText(_translate("MainWindow", "Release"))
        self.drive_label.setText(_translate("MainWindow", "Drive Robot"))
        self.manipulate_label.setText(_translate("MainWindow", "Manipulate Robot"))
        self.backward_manipulate.setText(_translate("MainWindow", "Backward"))
        self.forward_manipulate.setText(_translate("MainWindow", "Forward"))
        self.idle_drive.setText(_translate("MainWindow", "Idle/Stop"))
        self.down_manipulate.setText(_translate("MainWindow", "Down"))
        self.right_manipulate.setText(_translate("MainWindow", "Right"))
        self.idle_manipulate.setText(_translate("MainWindow", "Idle/Stop"))
        self.up_manipulate.setText(_translate("MainWindow", "Up"))
        self.left_manipulate.setText(_translate("MainWindow", "Left"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
