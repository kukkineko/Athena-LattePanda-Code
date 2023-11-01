import math
from PyQt6 import QtCore, QtGui, QtWidgets



class ColorWheel(QtWidgets.QWidget):
    colorChanged = QtCore.pyqtSignal(QtGui.QColor)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._color = QtGui.QColor(255, 0, 0)
        self.setMouseTracking(True)

    def setColor(self, color):
        if self._color != color:
            self._color = color
            self.colorChanged.emit(self._color)
            self.update()

    def color(self):
        return self._color

    def sizeHint(self):
        return QtCore.QSize(250, 250)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        #painter.setRenderHint(QtGui.QPainter.Antialiasing)

        outerRadius = min(self.width(), self.height()) / 2
        innerRadius = outerRadius - 20

        center = self.rect().center()
        gradient = QtGui.QRadialGradient(QtCore.QPointF(center), outerRadius, QtCore.QPointF(center))
        gradient.setColorAt(0, QtGui.QColor(255, 255, 255))
        gradient.setColorAt(1, QtGui.QColor(0, 0, 0))
        painter.setBrush(QtGui.QBrush(gradient))
        painter.setPen(QtGui.QPen(QtCore.Qt.gray, 1))
        painter.drawEllipse(center, outerRadius, outerRadius)
        painter.setBrush(QtGui.QBrush(QtCore.Qt.black))
        painter.drawEllipse(center, innerRadius, innerRadius)

        gradient = QtGui.QConicalGradient(center, 90)
        gradient.setColorAt(0, QtCore.Qt.red)
        gradient.setColorAt(60 / 360, QtCore.Qt.yellow)
        gradient.setColorAt(120 / 360, QtCore.Qt.green)
        gradient.setColorAt(180 / 360, QtCore.Qt.cyan)
        gradient.setColorAt(240 / 360, QtCore.Qt.blue)
        gradient.setColorAt(300 / 360, QtCore.Qt.magenta)
        gradient.setColorAt(1, QtCore.Qt.red)
        painter.setPen(QtGui.QPen(QtCore.Qt.NoPen))
        painter.setBrush(QtGui.QBrush(gradient))
        painter.drawEllipse(center, innerRadius, innerRadius)

        painter.setPen(QtGui.QPen(QtCore.Qt.black, 1))
        painter.setBrush(QtGui.QBrush(QtCore.Qt.NoBrush))
        angle = self._color.hue() / 360 * math.tau
        painter.drawLine(center, self.color_pos)
        self.colorwheel = ColorWheel()
        self.colorwheel.setGeometry(QtCore.QRect(11, 11, 944, 399))
        self.colorwheel.setObjectName("colorwheel")
        self.colorwheel.setParent(self.bottom_IO)

