#!/usr/bin/env python

# Modified for Team Align by: Rohan Rao

#############################################################################
##
## Copyright (C) 2013 Riverbank Computing Limited.
## Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
## All rights reserved.
##
## This file is part of the examples of PyQt.
##
## $QT_BEGIN_LICENSE:BSD$
## You may use this file under the terms of the BSD license as follows:
##
## "Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above copyright
##     notice, this list of conditions and the following disclaimer in
##     the documentation and/or other materials provided with the
##     distribution.
##   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
##     the names of its contributors may be used to endorse or promote
##     products derived from this software without specific prior written
##     permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
## $QT_END_LICENSE$
##
#############################################################################

from PyQt5.QtCore import QDateTime, Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget)

# ROS related libraries
import rospy
from std_msgs.msg import Float64, Int8
from state_machine.msg import *

class WidgetGallery(QDialog):
    def __init__(self, parent=None):
        super(WidgetGallery, self).__init__(parent)

        self.originalPalette = QApplication.palette()
        self.textEdit = QTextEdit()

        self.createTopLeftGroupBox()
        self.createTopRightGroupBox()
        topLayout = QHBoxLayout()

        mainLayout = QGridLayout()
        mainLayout.addLayout(topLayout, 0, 0, 1, 2)
        mainLayout.addWidget(self.topLeftGroupBox, 1, 0)
        mainLayout.addWidget(self.topRightGroupBox, 1, 1)
        mainLayout.setRowStretch(1, 1)
        # mainLayout.setRowStretch(2, 1)
        mainLayout.setColumnStretch(0, 1)
        mainLayout.setColumnStretch(1, 1)
        self.setLayout(mainLayout)

        self.setWindowTitle("Align: An Autonomous Payload Handling System")
        self.changeStyle('Fusion')

        # ROS topics to publish
        self.cmd_pub = rospy.Publisher('SM_input', StateIn, queue_size=1)
        self.StateUpdate = StateIn()
        self.which_callback = -1

    def changeStyle(self, styleName):
        QApplication.setStyle(QStyleFactory.create(styleName))

    def createTopLeftGroupBox(self):
        self.topLeftGroupBox = QGroupBox("Choose the next action")

        radioButton1 = QRadioButton("Pick up pod given pod ID")
        radioButton2 = QRadioButton("Drop off pod at drop location")
        radioButton3 = QRadioButton("Unlock the pod from the chassis")
        radioButton4 = QRadioButton("Locking successful")
        radioButton5 = QRadioButton("Approach Navigation")
        radioButton1.clicked.connect(self.podpickup)
        radioButton2.clicked.connect(self.poddropoff)
        radioButton3.clicked.connect(self.podunlock)
        radioButton4.clicked.connect(self.lockingsuccess)
        radioButton5.clicked.connect(self.approachnavigation)
        radioButton1.setChecked(True)

        layout = QVBoxLayout()
        layout.addWidget(radioButton1)
        layout.addWidget(radioButton2)
        layout.addWidget(radioButton3)
        layout.addWidget(radioButton4)
        layout.addWidget(radioButton5)
        layout.addStretch(1)
        self.topLeftGroupBox.setLayout(layout)    

    def createTopRightGroupBox(self):
        self.topRightGroupBox = QGroupBox("Provide pod information")

        sendPodID = QPushButton("Send!")
        sendPodID.setDefault(True)
        sendPodID.clicked.connect(self.sendtextdata)

        self.textEdit.setPlainText("(replace with pod ID)")

        layout = QVBoxLayout()
        layout.addWidget(self.textEdit)
        layout.addWidget(sendPodID)
        layout.addStretch(1)
        self.topRightGroupBox.setLayout(layout)

    def podpickup(self):
        self.which_callback = 0
        self.textEdit.setDisabled(False)
        self.textEdit.setPlainText("(replace with pod ID)")

    def poddropoff(self):
        self.which_callback = 1
        self.textEdit.setDisabled(False)
        self.textEdit.setPlainText("(replace with drop off location)")
    
    def podunlock(self):
        self.which_callback = 2
        self.textEdit.setDisabled(True)

    def lockingsuccess(self):
        self.which_callback = 3
        self.textEdit.setDisabled(True)

    def approachnavigation(self):
        self.which_callback = 4
        self.textEdit.setDisabled(False)
        self.textEdit.setPlainText("(replace with pod ID)")

    def sendtextdata(self):
        text = self.textEdit.toPlainText()
        print(text)
        if not text.isnumeric():
            print("Invalid input!")
        elif int(text) > 100:
            print("Invalid input!")
        else:
            # publish this to the pod server or state machine
            if self.which_callback == 0:
                #pickup
                self.StateUpdate.StateTransitionCond = int(text)
                self.StateUpdate.TransState = 0
                self.StateUpdate.OperationMode = 1

            elif self.which_callback == 1:
                #dropoff
                self.StateUpdate.StateTransitionCond = int(text)
                self.StateUpdate.TransState = 0
                self.StateUpdate.OperationMode = 2

            elif self.which_callback == 2:
                #podunlock
                self.StateUpdate.StateTransitionCond = 0
                self.StateUpdate.TransState = 0
                self.StateUpdate.OperationMode = 2

            elif self.which_callback == 3:
                #podlocksuccess
                self.StateUpdate.StateTransitionCond = 1
                self.StateUpdate.TransState = 0
                self.StateUpdate.OperationMode = 0

            elif self.which_callback == 4:
                #approach
                self.StateUpdate.StateTransitionCond = int(text)
                self.StateUpdate.TransState = 0
                self.StateUpdate.OperationMode = 0
                
            self.cmd_pub.publish(self.StateUpdate)
            

if __name__ == '__main__':

    import sys

    rospy.init_node('gui_client_py')
    while not rospy.is_shutdown():
        app = QApplication(sys.argv)
        alignGUI = WidgetGallery()
        alignGUI.show()
        sys.exit(app.exec_()) 