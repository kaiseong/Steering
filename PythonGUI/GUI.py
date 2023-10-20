# conda remove --name mbs --all
# conda create -n mbs python=3.8.16
# pip install PyQt5
# pip install numpy
# pip install pyserial
# pip install pyqtchart
# pip install can
# pip install cantools


import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import *
import numpy as np
import time
import threading

from Steer_Status import Steer_Status
from Serial_Comm import Serial_Comm
from qml_plot import qml_Chart
from save_data import save_data
from CAN_gui import CAN_Tap

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initstate()
    
    def initUI(self): 
        self.center()
        self.status = Steer_Status()
        self.Serial = Serial_Comm(self.status)
        self.Serial.params.connect(self.update_list)
        self.save_data = save_data(self.status)
        self.setWindowTitle('Steering Robot')
        
        grid = QGridLayout(self)
        grid.addLayout(self.setting_layout(),1,0)
        grid.addLayout(self.plot_layout(),1,1)
        grid.setColumnStretch(1,1)
        self.set_btn_disable()
        self.serial_refresh()
    
    def initstate(self):
        self.now_status = None
        self.init_angle()
    
    def init_angle(self):
        self.Serial.last_angle = 180
        self.Serial.shift_angle = 0
    
    def center(self): # Mack GUI Center
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

###################### Setting Tab Layout ###################### 
    def setting_layout(self): # Setting Layout vbox
        lbl_confirm = QLabel('<Confirmation before Testing>')
        lbl_font = lbl_confirm.font()
        lbl_font.setPointSize(12)
        lbl_font.setBold(True)
        lbl_confirm.setFont(lbl_font)
        
        self.Check1 = QCheckBox('Vehicle Start-Up')
        self.Check2 = QCheckBox('Equipment Installation')

        check_vbox = QVBoxLayout()
        check_vbox.addWidget(lbl_confirm)
        check_vbox.addWidget(self.Check1)
        check_vbox.addWidget(self.Check2)

        check_group = QGroupBox("Check List")
        check_group.setLayout(check_vbox)
        check_group.setStyleSheet("background-color: #ffffff")

        self.folder_btn= QPushButton("Folder Select", self)
        self.folder_btn.clicked.connect(self.directory_select)

        self.lbl_path = QLabel()
        self.lbl_path.setText(self.save_data.path)
        
        test_name_hbox = QVBoxLayout()
        test_name_hbox.addWidget(self.lbl_path)
        test_name_hbox.addWidget(self.folder_btn)
        path_group = QGroupBox("Path")
        path_group.setLayout(test_name_hbox)

        self.CAN = CAN_Tap()
        self.CAN.axis.connect(self.update_can_axis)

        vbox = QVBoxLayout()
        
        vbox.addWidget(check_group)
        vbox.addWidget(self.serial_select())
        vbox.addWidget(path_group)
        vbox.addWidget(self.mtr_mode_select())
        vbox.addWidget(self.CAN)
        vbox.addStretch(1)
        vbox.addLayout(self.set_btn())
        
        return vbox
    
    def serial_select(self):# Select Serial device and baudrate
        self.port_cb = QComboBox(self)
        self.baud_cb = QComboBox(self)

        self.port_dict = self.Serial.set_comoprt()
        
        lbl_Comport = QLabel('Port : ')
        
        port_hbox = QHBoxLayout()
        port_hbox.addWidget(lbl_Comport)
        port_hbox.addWidget(self.port_cb)
                
        lbl_Baud = QLabel('Baudrate : ')
        self.baud_cb.addItems(self.Serial.set_baudrate())
        self.baud_cb.setCurrentText('115200')

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.serial_connect)
        
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.serial_refresh)
        
        baud_hbox = QHBoxLayout()
        baud_hbox.addWidget(lbl_Baud)
        baud_hbox.addWidget(self.baud_cb)
        
        groupbox = QGroupBox('Serial Connect') 
        vbox = QVBoxLayout()
        vbox.addLayout(port_hbox)
        vbox.addLayout(baud_hbox)
        vbox.addWidget(self.btn_refresh)
        vbox.addWidget(self.btn_connect)
        
        groupbox.setLayout(vbox)
        
        return groupbox
    
    def serial_refresh(self):
        self.port_cb.clear()
        self.port_dict = self.Serial.set_comoprt()
        if len(self.port_dict) != 0:
            self.port_cb.addItems(self.port_dict.keys())
    
    def serial_connect(self):
        if self.port_cb.currentText()=='':
            msg = 'Connect your device and Push the refresh button!'
            QMessageBox.about(self, "Connecting", msg)
        else:
            port = self.port_dict[self.port_cb.currentText()]
            baud = self.baud_cb.currentText()            
            msg = f'Device Inform Correct?\
                    \nComoprt : {port}\
                    \nBaudrate : {baud}'
            reply = QMessageBox.question(self, 'Message', msg,
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                self.Serial.start(port, baud)
                self.serial_isconnected()
    
    def serial_isconnected(self):
        t = threading.Thread(target=self.ser_isconnected, args=(), daemon=True)
        t.start()

    def ser_isconnected(self): # Serial connect Select Port & baudrate
        flag = True
        while flag:
            if not self.Serial.q.empty():
                flag = self.Serial.q.get()
                self.update_gui()
                self.serial_refresh()
            time.sleep(1)
        QApplication.processEvents()
    
    # def ble_select(self): # Select BLE device and Open Button
    #     self.BLE_cb = QComboBox()
    #     self.BLE_cb.addItem('COM1')
    #     self.BLE_cb.addItem('COM2')
    #     self.BLE_cb.addItem('COM3')
    #     self.BLE_cb.addItem('COM4')
        
    #     self.btn_BLE= QPushButton("Open", self)
    #     self.btn_BLE.clicked.connect(self.BLE_open)
        
    #     groupbox = QGroupBox('BLE Connect') 
    #     hbox = QHBoxLayout()
    #     hbox.addWidget(self.BLE_cb)
    #     hbox.addWidget(self.btn_BLE)
    #     groupbox.setLayout(hbox)
        
    #     return groupbox
    
    def BLE_open(self): # Connect BLE device
        msg = "BLE is not yet implemented."
        QMessageBox.about(self, "BLE Connect", msg)
    
    def mtr_mode_select(self): # select Motor mode // Setting(L2L), Const Str, Sine Str
        mode_tab = QTabWidget()
        mode_tab.addTab(self.Setting_tab(), "Setting")
        mode_tab.addTab(self.Const_Str_tab(), "Const Str")
        mode_tab.addTab(self.Sine_Str_tab(), "Sine Str")
        
        return mode_tab
    
    def Setting_tab(self): # Setting(L2L) Tab // CSV File location, Check list, Motor On/Off, L2L        
        hbox = QHBoxLayout()
        
        L2L_Torq_lbl = QLabel("Torque : ")

        self.L2L_Torq = QDoubleSpinBox()
        self.L2L_Torq.setRange(0, 10)
        self.L2L_Torq.setSingleStep(0.1)
        self.L2L_Torq.setValue(self.status.init_target_torque)

        L2L_unit_lbl = QLabel(" Nm")
        hbox.addWidget(L2L_Torq_lbl)
        hbox.addWidget(self.L2L_Torq)
        hbox.addWidget(L2L_unit_lbl)

        self.btn_L2L = QPushButton('Measure Lock to Lock')
        self.btn_L2L.clicked.connect(self.L2L)

        self.btn_proceed = QPushButton('Proceed to Test')
        self.btn_proceed.clicked.connect(self.return2zero)
        
        self.btn_L2L_pause = QPushButton('Cancel')
        self.btn_L2L_pause.clicked.connect(self.emergency_stop)
        
        setting_vbox = QVBoxLayout()
        setting_vbox.addLayout(hbox)
        setting_vbox.addWidget(self.btn_L2L)
        setting_vbox.addWidget(self.btn_L2L_pause)
        setting_vbox.addWidget(self.btn_proceed)
        
        tab = QWidget()
        tab.setLayout(setting_vbox)
        
        return tab
    
    def directory_select(self):
        self.directory_path = QFileDialog.getExistingDirectory(self, "select Directory")
        self.lbl_path.setText(self.directory_path + '/')
        self.lbl_path.setWordWrap(True)
        self.save_data.path = self.lbl_path.text()
        
    def mtrOn(self): # Motor On button
        self.Serial.send_mode(self.status.mtrOn) # mode
    
    def mtrOff(self): # Motor Off button
        self.Serial.send_mode(self.status.mtrOff) # mode
        
    def set_btn_disable(self):
        self.btn_goready.setDisabled(True)
        self.btn_proceed.setDisabled(True)

        self.btn_L2L.setDisabled(True)
        self.btn_L2L_pause.setDisabled(True)

        self.btn_ConstStr_start.setDisabled(True)
        self.btn_ConstStr_pause.setDisabled(True)

        self.btn_SineStr_start.setDisabled(True)
        self.btn_SineStr_pause.setDisabled(True)
    
    def Const_Str_tab(self): # Const Str Tab // Velocity, Range, Start, Pause
        vel_lbl = QLabel('Velocity(0.0 ~ 1.8) :')
        self.Const_RPS = QDoubleSpinBox(self)
        self.Const_RPS.setRange(0.0, 1.8)
        self.Const_RPS.setSingleStep(0.1)
        self.Const_RPS.setValue(self.status.const_vel)

        rps_lbl = QLabel('rev/s')
        vel_hbox = QHBoxLayout()
        vel_hbox.addWidget(vel_lbl)
        vel_hbox.addStretch(1)
        vel_hbox.addWidget(self.Const_RPS)
        vel_hbox.addWidget(rps_lbl)
        
        lbl_range = QLabel('Range(0 ~ 50) :')

        self.constStr_range = QDoubleSpinBox(self)
        self.constStr_range.setRange(0, 50)
        self.constStr_range.setValue(30)
        self.constStr_range.setSingleStep(0.1)

        lbl_range2 = QLabel("%")

        range_hbox = QHBoxLayout()
        range_hbox.addWidget(lbl_range)
        range_hbox.addStretch(1)
        range_hbox.addWidget(self.constStr_range)
        range_hbox.addWidget(lbl_range2)

        cycle_lbl = QLabel('Number of Cycle(1 ~ 256) : ')

        self.constStr_cycle = QSpinBox(self)
        self.constStr_cycle.setMaximum(256)
        self.constStr_cycle.setMinimum(1)
        self.constStr_cycle.setValue(self.status.const_cycle)

        cycle_unit_lbl = QLabel('Turn')

        cycle_hbox = QHBoxLayout()
        cycle_hbox.addWidget(cycle_lbl)
        cycle_hbox.addStretch(1)
        cycle_hbox.addWidget(self.constStr_cycle)
        cycle_hbox.addWidget(cycle_unit_lbl)
        
        self.btn_ConstStr_start = QPushButton('Start Steering')
        self.btn_ConstStr_start.clicked.connect(self.constStr)
        
        self.btn_ConstStr_pause = QPushButton('Pause / Restart')
        self.btn_ConstStr_pause.clicked.connect(self.pause)
        
        constStr_vbox = QVBoxLayout()
        constStr_vbox.addLayout(vel_hbox)
        constStr_vbox.addLayout(range_hbox)
        constStr_vbox.addLayout(cycle_hbox)
        constStr_vbox.addWidget(self.btn_ConstStr_start)
        constStr_vbox.addWidget(self.btn_ConstStr_pause)
        
        tab = QWidget()
        tab.setLayout(constStr_vbox)
        
        return tab  
    
    def L2L(self): # send msg L2L to MCU
        self.Serial.last_angle = 180
        self.Serial.shift_angle = 0
        
        torque = int(float(self.L2L_Torq.text()) / self.status.Kt*100 ) # N / Kt * Motor current / gear_ratio
        tor1 = (torque >> 8) & 0xff
        tor2 = torque & 0xff

        self.Serial.send_str(self.status.L2L, [tor2, tor1, self.status.stuck_angle, self.status.stuck_time])

    def constStr(self): # send msg Const Str to MCU 
        mode = self.status.ConstStr

        # rps.value를 0에서 255까지로 매핑
        rps = int(self.Const_RPS.value() * 100 ) 

        # 반복 횟수. zero angle에 한번 도달할때 1회라고 가정.
        cycle = self.constStr_cycle.value()

        # 0 %에서 100%사이로 0.5%단위(1byte, 0 - 200)
        amp = int(self.constStr_range.value() * 2) # int
        
        self.data = [rps, cycle, amp, 0]
        self.Serial.send_str(mode, self.data)
    
    def pause(self): # send msg Pause to MCU
        if self.is_pause:
            self.Serial.send_mode(self.status.Restart) # mode
        else:
            self.Serial.send_mode(self.status.Pause) # mode
    
    def Sine_Str_tab(self): # Sine Str Tab // Initial Angle, Amplitude, Frequency, Cycle, Start, Pause
        Sine_Str_setting = ["Initial Angle(0 ~ 100)", "Sine Amplitude(0 ~ 510)",
                            "Sine Frequency(0 ~ 2.55)","Number of Cycle(1 ~ 256)" ]
        Sine_Str_set_unit = ["%", chr(176), "Hz", "Turn" ]
        Sine_Str_set_unit_2 = chr(177)
        Sine_Str_set_value = [self.status.sine_angle, self.status.sine_amp, self.status.sine_freq, self.status.sine_cycle]
        Sine_Str_set_value_min = [0,0,0,1]
        Sine_Str_set_value_max = [100,510,2.55,256]
        
        SineStr_vbox = QVBoxLayout()
        self.Sine_Str_value = []
        for i in range(len(Sine_Str_setting)):
            if i != 3:
                self.Sine_Str_value.append(QDoubleSpinBox(self))
                self.Sine_Str_value[i].setSingleStep(0.1)
            else:
                self.Sine_Str_value.append(QSpinBox(self))
            self.Sine_Str_value[i].setMaximum(Sine_Str_set_value_max[i])
            self.Sine_Str_value[i].setMinimum(Sine_Str_set_value_min[i])
            self.Sine_Str_value[i].setValue(Sine_Str_set_value[i])
            
            Sine_Str_setting_lbl = QLabel(Sine_Str_setting[i] + " : ")
            Sine_Str_set_unit_lbl = QLabel(Sine_Str_set_unit[i])
            
            SineStr_hbox = QHBoxLayout()
            SineStr_hbox.addWidget(Sine_Str_setting_lbl)
            SineStr_hbox.addStretch(1)
            if i == 1:
                lbl_unit = QLabel(Sine_Str_set_unit_2)
                SineStr_hbox.addWidget(lbl_unit)
            SineStr_hbox.addWidget(self.Sine_Str_value[i])
            SineStr_hbox.addWidget(Sine_Str_set_unit_lbl)
            
            SineStr_vbox.addLayout(SineStr_hbox)

        self.btn_SineStr_start = QPushButton('Start Steering')
        self.btn_SineStr_start.clicked.connect(self.sineStr)
        
        self.btn_SineStr_pause = QPushButton('Pause / Restart')
        self.btn_SineStr_pause.clicked.connect(self.pause)
        
        sineStr_vbox = QVBoxLayout()
        sineStr_vbox.addLayout(SineStr_vbox)
        sineStr_vbox.addWidget(self.btn_SineStr_start)
        sineStr_vbox.addWidget(self.btn_SineStr_pause)
        
        tab = QWidget()
        tab.setLayout(sineStr_vbox)
        
        return tab   
    
    def sineStr(self): # send msg Sine Str to MCU
        mode = self.status.SineStr
        self.data = []
        for value in self.Sine_Str_value:
            self.data.append(value.value())
        self.data[0] = 200 - int(self.data[0] * 2)  # offset
        #self.data[1] = int(self.data[1]/2)  # amp
        self.data[1] = int(self.data[1])  # amp
        self.data[2] = int(self.data[2] * 100)  # hz

        #amp = self.data[1] * 2
        amp = self.data[1]
        freq = self.data[2] / 100
        limit_speed = self.status.motor_speed / 2 / np.pi

        if amp * freq > limit_speed:
            msg = 'Please adjust the range of \"Amplitude\" and \"Frequency\" to be Small.'
            QMessageBox.about(self, "Range Error", msg)

        else:
            offset = self.data[0]
            amp = self.data[1] / 2.55
            # print(offset + amp)

            if offset + amp > 200 or offset - amp < 0:
                msg = 'Please adjust the range of \"Init Angle\" and \"Amplitude\" to be Small.'
                QMessageBox.about(self, "Range Error", msg)
            else:
                self.data = self.data[2:] + self.data[:2]
                self.Serial.send_str(mode, self.data)
    
    def set_btn(self): # make Button Return2zero & Emergency stop
        self.btn_goready = QPushButton("Return to Zero")
        self.btn_goready.clicked.connect(self.return2zero)
        
        self.emergency_btn = QPushButton("Motor OFF")
        self.emergency_btn.clicked.connect(self.emergency_stop)

        self.emergency_btn.setStyleSheet("background-color : red")
        font_stop = self.emergency_btn.font()
        font_stop.setPointSize(12)
        font_stop.setBold(True)
        self.emergency_btn.setFont(font_stop)
        
        vbox = QVBoxLayout()
        vbox.addWidget(self.btn_goready)
        vbox.addWidget(self.emergency_btn)
        
        return vbox
        
    def return2zero(self): # send msg Return2zero to MCU
        self.Serial.send_mode(self.status.GOREADY)
    
    def emergency_stop(self): # send msg Emergency stop to MCU
        self.Serial.send_mode(self.status.mtrOff)
        self.status.l2l_pass = False
        self.update_l2l_lbl(0,0)
        
###################### Plot Layout ###################### 
    def plot_layout(self): # Setting Layout vbox
        self.status_lbl = QLabel()
        font = self.status_lbl.font()
        font.setPointSize(10)
        self.status_lbl.setFont(font)
        self.l2l_lbl = QLabel()
        self.l2l_lbl.setFont(font)
        self.update_l2l_lbl(0,0)
        self.text_para = QLabel()
        self.text_para.setFixedHeight(62)
        self.text_para.setFont(font)
        status_hbox = QHBoxLayout()
        status_hbox.addWidget(self.status_lbl)
        status_hbox.addWidget(self.l2l_lbl)
        status_hbox.addWidget(self.text_para)
        
        # self.update_gui([0,0,-1,0,0])
        self.update_gui()
        self.last_status = -1
        plot_vbox = QVBoxLayout()
        plot_vbox.addLayout(status_hbox)
        plot_title = ["Angle","Torque", "CAN"]
        self.plot = []
        can_hbox = QHBoxLayout()
        for i,name in enumerate(plot_title):
            if i < 2:
                self.plot.append(qml_Chart(name, i))
                plot_vbox.addWidget(self.plot[i].widget)
            else:
                for j in range(3):
                    self.plot.append(qml_Chart(name + str(j + 1), i + j))
                    can_hbox.addWidget(self.plot[i + j].widget)
                plot_vbox.addLayout(can_hbox)
        return plot_vbox

###################### update backend ######################
    @pyqtSlot(list)
    def update_list(self, status):       
        try:
            self.status.motor_status = status[0]     # Motor status
            self.status.BLE_status = status[1]       # BLE status
            self.status.MCU_status = status[2]       # MCU state
            self.cal_value(status)
            self.do_get_serial()

        except Exception as e:
            print(e)
            self.Serial.q.put(True)
            self.Serial.ser.close()
    
    def is_l2l_term(self):
        if self.status.MCU_status == self.status.disp_Mode.index("L2L Term") \
        and self.last_status != self.status.disp_Mode.index("L2L Term"):
            print(self.status.MCU_status, self.last_status)
            msg = f'Are you sure L2L Values?\
                    \nLeft Lock : {min(self.status.ang_buff)}\
                    \nRight Lock : {max(self.status.ang_buff)}'
            reply = QMessageBox.question(self, 'Message', msg,
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                self.status.l2l_pass = True
                self.Serial.send_mode(self.status.GOREADY)

    def update_l2l_lbl(self, cw, ccw):
        self.l2l_lbl.setText(\
            "CW Lock   : %.2f (%.2f)"%(cw, cw - self.status.offset) + "\n"
            "Center      : %.2f (%.2f)"%(self.status.offset, 0)+"\n"
            "CCW Lock : %.2f (%.2f)"%(ccw, ccw - self.status.offset))
    
    def cal_value(self, status):
        if status[2] == self.status.disp_Mode.index("L2L")\
        and self.last_status != self.status.disp_Mode.index("L2L")\
        and self.last_status != self.status.disp_Mode.index("Pause"):
            self.Serial.shift_angle = self.status.offset  - status[3]
        elif self.last_status != self.status.disp_Mode.index("L2L Term")\
        and status[2] == self.status.disp_Mode.index("L2L Term"):
            cw = max(self.status.ang_buff[self.save_data.save_start[0]:])
            ccw = min(self.status.ang_buff[self.save_data.save_start[0]:])
            offset = self.status.offset - ((cw + ccw) / 2 )
            self.Serial.shift_angle += offset 
            self.update_l2l_lbl(cw + offset, ccw + offset)

        # Shift angle when difference is more than 200 deg
        if self.Serial.last_angle - status[3] > 200:
            self.Serial.shift_angle += 360
        elif self.Serial.last_angle - status[3] < -200:
            self.Serial.shift_angle -= 360
        self.Serial.last_angle = status[3]
        
        # if self.status.MCU_status != self.status.disp_Mode.index("Pause"):
        self.status.serial_num += 1
        self.status.ang_buff.append((self.Serial.shift_angle + status[3])) # Angle buff
        self.status.tor_buff.append(status[4])                                                      # Torque buff
    
    def do_get_serial(self):
        self.update_gui()
        self.btn_able()
        # self.update_axis()
        self.save_data.save_data(self.CAN.CAN_name)
    
    def update_gui(self): # Update Status in GUI
        self.status_lbl.setText(\
        "Motor Status        : " + self.status.disp_MS[self.status.motor_status] + "\n"
        "BLE Connection   : " + self.status.disp_BLE[self.status.BLE_status] + "\n"
        "Mode in Progress : " + self.status.disp_Mode[self.status.MCU_status])

        try:
            self.text_para.setText(\
            "<Real-time Parameters> \n"\
            # "Angle  : %.2f "%status[3] + chr(176) + "\n"
            "Angle  : %d "%self.status.ang_buff[-1] + chr(176) + "\n"
            "Torque : %.2f  Nm" %self.status.tor_buff[-1])
        except:
            self.text_para.setText(\
            "<Real-time Parameters> \n"\
            "Angle  : %d "%0 + chr(176) + "\n"
            "Torque : %.2f  Nm" %0)
        self.text_para.setStyleSheet("background-color: #ffffff")
        self.is_pause = self.status.MCU_status == self.status.disp_Mode.index("Pause")
    
    def btn_able(self):
        if self.last_status != self.status.MCU_status:
            self.set_btn_disable()
            for btn in self.status.able_list[self.status.MCU_status]:
                exec("self.btn_" + btn + ".setDisabled(False)")
        self.last_status = self.status.MCU_status
    
    def update_axis(self): # Change Angle plot y-axis limit when L2L mode
        if self.status.MCU_status != self.now_status:
            if self.status.MCU_status == self.status.disp_Mode.index("L2L"):
                self.plot[0].context.setContextProperty("ymin", -600)
                self.plot[0].context.setContextProperty("ymax", 600)
            else:
                self.plot[0].context.setContextProperty("ymin", 0)
                self.plot[0].context.setContextProperty("ymax", 1000)
        self.now_status = self.status.MCU_status
    
    @pyqtSlot(list, list)
    def update_can_axis(self, axis, title):
        for i in range(3):
            self.plot[i + 2].context.setContextProperty("ymin", axis[i][0])
            self.plot[i + 2].context.setContextProperty("ymax", axis[i][1])
            self.plot[i + 2].context.setContextProperty("name", title[i])

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyApp()
    ex.showMaximized()
    app.exec_()