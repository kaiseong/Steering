import can 
import cantools
import serial.tools.list_ports as sp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from Steer_Status import Steer_Status
from threading import Thread
import time
import sys

class CAN_Comm(QWidget): 
    params = pyqtSignal(tuple)
    
    def __init__(self, status = Steer_Status()):
        super().__init__()
        self.status = status

    def set_comoprt(self):
        COM_list = sp.comports()
        com_port = {}

        for port, desc, _ in sorted(COM_list):
            com_port[desc] = port
        return com_port
    
    def start(self, port, signals, db):
        t=Thread(target=self.read_can, args=(port, signals, db), daemon=True)
        t.start()

    def read_can(self, port, signals, db):
        bus = can.interface.Bus(bustype='slcan', channel=port, bitrate=500000)

        timestamp_len = 0
        num = 300

        while timestamp_len < num:
            data = bus.recv()
            if data.arbitration_id in signals:
                self.status.timestamp[signals.index(data.arbitration_id)].append(data.timestamp)
            timestamp_len = sum([len(self.status.timestamp[i]) for i in range(3)])

        over_num = [len(self.status.timestamp[i]) < num * 0.3 for i in range(3)]
        self.status.time_diff = [self.status.can_diff[i]/1000 for i in over_num]

        while True:
            data = bus.recv()
            if data.arbitration_id in signals:
                for signal in db.decode_message(data.arbitration_id, data.data).items():
                    self.params.emit(signal)

def main():
    app = QApplication(sys.argv)
    
    can_com = CAN_Comm()
    signals = [688, 593, 897]
    db =  cantools.database.load_file('MDPS.dbc')
    can_com.start('COM9', signals, db)    
    while True:
        a=1
        time.sleep(1)
    
    sys.exit(app.exec_())
        
if __name__ == '__main__':
    main()