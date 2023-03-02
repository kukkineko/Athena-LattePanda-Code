import sys
from PyQt6 import QtWidgets, QtCore, QtGui
from tagoffenetur import Ui_MainWindow
import comms

class GUIMain:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.MainWindow = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.MainWindow)
        self.connections()
        self.esp = comms.ESP32()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.blink_labels)
        self.timer.start(500) # this will call the blink_indicator function every 500ms





    def blinker_function(self, nr):
        match nr:
            case 1:
                self.b_states[1] = not self.b_states[1]
                self.b_states[0] = False
            case 2:
                self.b_states[0] = not self.b_states[0]
                self.b_states[1] = False
            case 5:
                if self.b_states[0] is not self.b_states[1]:
                    self.b_states[0] = True
                    self.b_states[1] = True
                else:
                    self.b_states[0] = not self.b_states[0]
                    self.b_states[1] = not self.b_states[1]

        #print(self.b_states)
        self.update_lights()

        
        

    def heck_function(self, nr):
        match nr:
            case 3:
                self.h_states[0] = not self.h_states[0]
            case 4:
                self.h_states[1] = not self.h_states[1]


        #print(self.h_states)
        self.update_lights()

    def blink_labels(self):
        # blinker
        if self.b_states[0]:
            if self.l_v_label.styleSheet() == "QLabel { background-color : orange; }":
                self.l_v_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")
                if self.h_states[0]:
                    self.l_h_label.setStyleSheet("QLabel { background-color : red; }")
                elif self.h_states[1]:
                    self.l_h_label.setStyleSheet("QLabel { background-color : white; }")
                else:
                    self.l_h_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")

            else:
                self.l_v_label.setStyleSheet("QLabel { background-color : orange; }")
                self.l_h_label.setStyleSheet("QLabel { background-color : orange; }")
        else:
            self.l_v_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")
            self.l_h_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")
            
        if self.b_states[1]:
            if self.r_v_label.styleSheet() == "QLabel { background-color : orange; }":
                self.r_v_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")
                if self.h_states[0]:
                    self.r_h_label.setStyleSheet("QLabel { background-color : red; }")
                elif self.h_states[1]:
                    self.r_h_label.setStyleSheet("QLabel { background-color : white; }")
                else:
                    self.r_h_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")
            else:
                self.r_v_label.setStyleSheet("QLabel { background-color : orange; }")
                self.r_h_label.setStyleSheet("QLabel { background-color : orange; }")
        else:
            self.r_v_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")
            self.r_h_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")

        # breaking
        if self.h_states[1] and not self.b_states[0] and not self.b_states[1]:
            self.l_h_label.setStyleSheet("QLabel { background-color : white; }")
            self.r_h_label.setStyleSheet("QLabel { background-color : white; }")
        elif self.h_states[0] and not self.b_states[0] and not self.b_states[1]:
            self.l_h_label.setStyleSheet("QLabel { background-color : red; }")
            self.r_h_label.setStyleSheet("QLabel { background-color : red; }")
        elif not self.b_states[0] and not self.b_states[1]:
            self.l_h_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")
            self.r_h_label.setStyleSheet("QLabel { background-color : rgb(100, 100, 100); }")

            
        





    def update_lights(self):
        print("b states: ", self.b_states)
        print("h states: ", self.h_states)
        if self.b_states[0] == False and self.b_states[1] == False and self.h_states[0] == False and self.h_states[1] == False:
            print("reset")
            self.esp.send_req(0)
        elif self.b_states[0] == True and self.b_states[1] == False and self.h_states[0] == False and self.h_states[1] == False:
            print("left")
            self.esp.send_req(2)
        elif self.b_states[0] == False and self.b_states[1] == True and self.h_states[0] == False and self.h_states[1] == False:
            print("right")
            self.esp.send_req(1)
        elif self.b_states[0] == True and self.b_states[1] == True and self.h_states[0] == False and self.h_states[1] == False:
            print("hazard")
            self.esp.send_req(5)
        elif self.b_states[0] == False and self.b_states[1] == False and self.h_states[0] == True and self.h_states[1] == False:
            print("braking")
            self.esp.send_req(3)
        elif self.b_states[0] == False and self.b_states[1] == False and self.h_states[0] == False and self.h_states[1] == True:
            print("reverse")
            self.esp.send_req(4)
        elif self.b_states[0] == True and self.b_states[1] == False and self.h_states[0] == True and self.h_states[1] == False:
            print("left + braking")
            self.esp.send_req(7)
        elif self.b_states[0] == False and self.b_states[1] == True and self.h_states[0] == True and self.h_states[1] == False:
            print("right + braking")
            self.esp.send_req(6)
        elif self.b_states[0] == True and self.b_states[1] == True and self.h_states[0] == True and self.h_states[1] == False:
            print("hazard + braking")
            self.esp.send_req(8)
        elif self.b_states[0] == False and self.b_states[1] == False and self.h_states[0] == False and self.h_states[1] == True:
            print("reverse + braking")
            self.esp.send_req(9)
        elif self.b_states[0] == True and self.b_states[1] == False and self.h_states[0] == False and self.h_states[1] == True:
            print("left + reverse")
            self.esp.send_req(11)
        elif self.b_states[0] == False and self.b_states[1] == True and self.h_states[0] == False and self.h_states[1] == True:
            print("right + reverse")
            self.esp.send_req(10)
        elif self.b_states[0] == True and self.b_states[1] == True and self.h_states[0] == False and self.h_states[1] == True:
            print("hazard + reverse")
            self.esp.send_req(12)
        elif self.b_states[0] == True and self.b_states[1] == False and self.h_states[0] == True and self.h_states[1] == True:
            print("left + braking + reverse")
            self.esp.send_req(14)
        elif self.b_states[0] == False and self.b_states[1] == True and self.h_states[0] == True and self.h_states[1] == True:
            print("right + braking + reverse")
            self.esp.send_req(13)
        elif self.b_states[0] == True and self.b_states[1] == True and self.h_states[0] == True and self.h_states[1] == True:
            print("hazard + braking + reverse")
            self.esp.send_req(15)



    def connections(self):
        #---buttons---
        #blinker
        self.b_states = [False, False]  #blinker states, 0 is left / 1 is right

        self.ui.r_blinker.clicked.connect(lambda: self.blinker_function(1))
        self.ui.l_binker.clicked.connect(lambda: self.blinker_function(2))
        self.ui.warnblink.clicked.connect(lambda: self.blinker_function(5))

        #heck
        self.h_states = [False, False]  #heck states, 0 is braking / 1 is reversing mode

        self.ui.bremsen.clicked.connect(lambda: self.heck_function(3))
        self.ui.ruckwarts.clicked.connect(lambda: self.heck_function(4))    

        #---labels---
        self.l_v_label = self.MainWindow.findChild(QtWidgets.QLabel, "l_v")
        self.r_v_label = self.MainWindow.findChild(QtWidgets.QLabel, "r_v")
        self.l_h_label = self.MainWindow.findChild(QtWidgets.QLabel, "l_h")
        self.r_h_label = self.MainWindow.findChild(QtWidgets.QLabel, "r_h")


        
    def main(self):
        self.MainWindow.show()
        sys.exit(self.app.exec())

if __name__ == '__main__':
    gui = GUIMain()
    gui.main()
