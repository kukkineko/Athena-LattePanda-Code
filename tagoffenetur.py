# Form implementation generated from reading ui file 'tagoffenetur.ui'
#
# Created by: PyQt6 UI code generator 6.4.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.



from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1350, 490)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonIconOnly)
        MainWindow.setTabShape(QtWidgets.QTabWidget.TabShape.Triangular)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(10, 10, 1331, 421))
        self.frame.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.frame.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.frame.setLineWidth(2)
        self.frame.setObjectName("frame")
        self.bottom_IO = QtWidgets.QFrame(self.frame)
        self.bottom_IO.setGeometry(QtCore.QRect(1, 1, 1331, 421))
        self.bottom_IO.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.bottom_IO.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.bottom_IO.setObjectName("bottom_IO")
        self.l_lichter = QtWidgets.QFrame(self.bottom_IO)
        self.l_lichter.setGeometry(QtCore.QRect(10, 20, 181, 381))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.l_lichter.sizePolicy().hasHeightForWidth())
        self.l_lichter.setSizePolicy(sizePolicy)
        self.l_lichter.setMinimumSize(QtCore.QSize(180, 360))
        self.l_lichter.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.l_lichter.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.l_lichter.setObjectName("l_lichter")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.l_lichter)
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetNoConstraint)
        self.horizontalLayout.setContentsMargins(-1, -1, -1, 9)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetNoConstraint)
        self.verticalLayout.setSpacing(9)
        self.verticalLayout.setObjectName("verticalLayout")
        self.l_v = QtWidgets.QLabel(self.l_lichter)
        self.l_v.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.l_v.setLineWidth(1)
        self.l_v.setText("")
        self.l_v.setObjectName("l_v")
        self.verticalLayout.addWidget(self.l_v)
        self.l_h = QtWidgets.QLabel(self.l_lichter)
        self.l_h.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.l_h.setLineWidth(1)
        self.l_h.setText("")
        self.l_h.setObjectName("l_h")
        self.verticalLayout.addWidget(self.l_h)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.IO_button = QtWidgets.QFrame(self.bottom_IO)
        self.IO_button.setGeometry(QtCore.QRect(210, 20, 881, 381))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.IO_button.sizePolicy().hasHeightForWidth())
        self.IO_button.setSizePolicy(sizePolicy)
        self.IO_button.setMinimumSize(QtCore.QSize(451, 380))
        self.IO_button.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.IO_button.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.IO_button.setObjectName("IO_button")
        self.widget = QtWidgets.QWidget(self.IO_button)
        self.widget.setGeometry(QtCore.QRect(11, 11, 861, 361))
        self.widget.setObjectName("widget")
        self.blinker_heck = QtWidgets.QVBoxLayout(self.widget)
        self.blinker_heck.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetNoConstraint)
        self.blinker_heck.setContentsMargins(0, 0, 0, 0)
        self.blinker_heck.setObjectName("blinker_heck")
        self.blinker = QtWidgets.QHBoxLayout()
        self.blinker.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetMinimumSize)
        self.blinker.setObjectName("blinker")
        self.l_binker = QtWidgets.QPushButton(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.l_binker.sizePolicy().hasHeightForWidth())
        self.l_binker.setSizePolicy(sizePolicy)
        self.l_binker.setObjectName("l_binker")
        self.blinker.addWidget(self.l_binker)
        self.warnblink = QtWidgets.QPushButton(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.warnblink.sizePolicy().hasHeightForWidth())
        self.warnblink.setSizePolicy(sizePolicy)
        self.warnblink.setObjectName("warnblink")
        self.blinker.addWidget(self.warnblink)
        self.r_blinker = QtWidgets.QPushButton(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.r_blinker.sizePolicy().hasHeightForWidth())
        self.r_blinker.setSizePolicy(sizePolicy)
        self.r_blinker.setObjectName("r_blinker")
        self.blinker.addWidget(self.r_blinker)
        self.blinker_heck.addLayout(self.blinker)
        self.heck = QtWidgets.QHBoxLayout()
        self.heck.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetNoConstraint)
        self.heck.setContentsMargins(-1, 0, -1, -1)
        self.heck.setObjectName("heck")
        self.bremsen = QtWidgets.QPushButton(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.bremsen.sizePolicy().hasHeightForWidth())
        self.bremsen.setSizePolicy(sizePolicy)
        self.bremsen.setObjectName("bremsen")
        self.heck.addWidget(self.bremsen)
        self.ruckwarts = QtWidgets.QPushButton(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ruckwarts.sizePolicy().hasHeightForWidth())
        self.ruckwarts.setSizePolicy(sizePolicy)
        self.ruckwarts.setObjectName("ruckwarts")
        self.heck.addWidget(self.ruckwarts)
        self.blinker_heck.addLayout(self.heck)
        self.blinker_heck.setStretch(0, 10)
        self.blinker_heck.setStretch(1, 4)
        self.r_lichter = QtWidgets.QFrame(self.bottom_IO)
        self.r_lichter.setGeometry(QtCore.QRect(1120, 20, 181, 381))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.r_lichter.sizePolicy().hasHeightForWidth())
        self.r_lichter.setSizePolicy(sizePolicy)
        self.r_lichter.setMinimumSize(QtCore.QSize(180, 360))
        self.r_lichter.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.r_lichter.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.r_lichter.setObjectName("r_lichter")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.r_lichter)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setSpacing(9)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.r_v = QtWidgets.QLabel(self.r_lichter)
        self.r_v.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.r_v.setLineWidth(1)
        self.r_v.setText("")
        self.r_v.setObjectName("r_v")
        self.verticalLayout_2.addWidget(self.r_v)
        self.r_h = QtWidgets.QLabel(self.r_lichter)
        self.r_h.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.r_h.setLineWidth(1)
        self.r_h.setText("")
        self.r_h.setObjectName("r_h")
        self.verticalLayout_2.addWidget(self.r_h)
        self.gridLayout_2.addLayout(self.verticalLayout_2, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1350, 21))
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
        self.l_binker.setText(_translate("MainWindow", "Blinker links"))
        self.warnblink.setText(_translate("MainWindow", "Warnlichter"))
        self.r_blinker.setText(_translate("MainWindow", "Blinker rechts"))
        self.bremsen.setText(_translate("MainWindow", "Bremsen"))
        self.ruckwarts.setText(_translate("MainWindow", "Rückwährts fahren"))
