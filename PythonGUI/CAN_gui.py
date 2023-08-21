import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from qml_plot import qml_Chart
from CAN_connect import CAN_Comm
from Steer_Status import Steer_Status
import cantools        
import numpy as np

class CAN_Tap(QWidget):
    axis = pyqtSignal(list, list)

    def __init__(self):
        super().__init__()
        self.status = Steer_Status()
        self.CAN_Comm = CAN_Comm(self.status)
        self.CAN_Comm.params.connect(self.update_can)
        self.widget = QVBoxLayout(self)
        self.widget.addWidget(self.initUI())
        self.widget.addStretch(1)
        self.CAN_name = None
                
    def initUI(self):
        groupbox = QGroupBox('CAN Setting') 
        vbox = QVBoxLayout()
        
        lbl_DBC = QLabel('DBC File')
        self.lbl_DBC_file = QLabel('MDPS.dbc')
        self.lbl_DBC_file.setWordWrap(True)  
        self.db =  cantools.database.load_file(self.lbl_DBC_file.text())

        DBC_select_btn = QPushButton("DBC Select")
        DBC_select_btn.clicked.connect(self.DBC_clicked)

        self.Com_port_cb = QComboBox()

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.comport_refresh)
        self.comport_refresh()

        connect_btn = QPushButton("Connect")
        connect_btn.clicked.connect(self.CANable_connect)

        vbox.addWidget(lbl_DBC)
        vbox.addWidget(self.lbl_DBC_file)
        vbox.addWidget(DBC_select_btn)
        vbox.addWidget(self.set_CAN_tab())
        vbox.addWidget(self.Com_port_cb)
        hbox = QHBoxLayout()
        hbox.addWidget(refresh_btn)
        hbox.addWidget(connect_btn)
        vbox.addLayout(hbox)

        groupbox.setLayout(vbox)

        return groupbox
    
    def comport_refresh(self):
        self.Com_port_cb.clear()
        self.port_dict = self.CAN_Comm.set_comoprt()
        if len(self.port_dict) != 0:
            self.Com_port_cb.addItems(self.port_dict.keys())
    
    def CANable_connect(self):
        if self.Com_port_cb.currentText()=='':
            msg = 'Connect your CAN device and Push the refresh button!'
            QMessageBox.about(self, "Connecting", msg)
        else:
            self.CAN_name = [self.CAN[i].currentText() for i in range(3)]
            port = self.port_dict[self.Com_port_cb.currentText()]
            msg = f'Information Correct?\
                \nCANable Comoprt : {port}\
                \nCAN1 : {self.CAN_name[0]}\
                \nCAN2 : {self.CAN_name[1]}\
                \nCAN3 : {self.CAN_name[2]}'
            reply = QMessageBox.question(self, 'Message', msg,
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                signals = []
                can_axis = []
                title = []
                for i in range(len(self.CAN)):
                    for j in range(len(self.dbc_list)):
                        if self.dbc_list[j][0] == self.CAN[i].currentText():
                            title.append(self.CAN[i].currentText())
                            signals.append(self.dbc_list[j][1])
                            now_axis = [self.dbc_list[j][2],self.dbc_list[j][3]] 
                    can_axis.append(now_axis)
                signals = list(set(signals))
                self.axis.emit(can_axis, title)
                self.CAN_Comm.start(port, signals, self.db)
    
    def set_CAN_tab(self):
        tab = QTabWidget()
        self.CAN = []
        for i in range(3):
            self.CAN.append(QComboBox())
            tab.addTab(self.set_CAN(i), "CAN" + str(i + 1))

        self.set_CAN_SigName()
        return tab
    
    def set_CAN(self, i):
        CAN_name = 'CAN' + str(i + 1)
        groupbox = QGroupBox(CAN_name) 
        hbox = QHBoxLayout()

        hbox.addWidget(QLabel("Signal Name"+str(i+1)+" : "))
        hbox.addWidget(self.CAN[i])

        groupbox.setLayout(hbox)

        return groupbox

    def DBC_clicked(self):
        fname = QFileDialog.getOpenFileName(self, 'Open file', './')
        if fname[0]:
            self.lbl_DBC_file.setText(fname[0])
        self.set_CAN_SigName()
    
    def set_CAN_SigName(self):
        self.db =  cantools.database.load_file(self.lbl_DBC_file.text())
        self.dbc_list = []
        for messages in self.db.messages:
            for signals in messages.signals:
                # ex) 'CR_Mdps_StrTq', '593', '-8', '7.9921875'
                sig_list = [signals.name, messages.frame_id, signals.minimum, signals.maximum]
                self.dbc_list.append(sig_list)
        
        for i in range(3):
            self.CAN[i].clear()
            self.CAN[i].addItems([sig[0] for sig in self.dbc_list])
            self.CAN[i].setCurrentIndex(i)
    
    @pyqtSlot(tuple)
    def update_can(self, data):
        # data = [signal_name, signal_value, signal_timestamp]
        for i in range(len(self.CAN)):
            if self.dbc_list[i][0] == data[0] and self.status.MCU_status != self.status.disp_Mode.index("Pause"):
                can_num = i + 2
                self.status.time_buff[can_num] -= self.status.time_diff[i]
                self.status.time_buff[can_num] = np.append(self.status.time_buff[can_num][1:], 5)
                self.status.can_buff[i].append(data[1])
                self.status.can_num[i] += 1
                # print(data)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        vbox = QVBoxLayout()
        self.plot = []
        for i in range(3):
            self.plot.append(qml_Chart("CAN" + str(i+1), i))
            vbox.addWidget(self.plot[i].widget)
        
        self.CAN = CAN_Tap()
        self.CAN.axis.connect(self.update_can_axis)

        grid = QGridLayout(self)
        grid.addWidget(self.CAN,1,0)
        grid.addLayout(vbox,1,1)
        grid.setColumnStretch(1,1)
    
    @pyqtSlot(list, list)
    def update_can_axis(self, axis, title):
        for i in range(3):
            self.plot[i].context.setContextProperty("ymin", axis[i][0])
            self.plot[i].context.setContextProperty("ymax", axis[i][1])
            self.plot[i].context.setContextProperty("name", title[i])

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.showMaximized()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()