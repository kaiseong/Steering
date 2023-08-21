import os
import getpass
import csv
from datetime import datetime

class save_data():
    def __init__(self, status):
        # mcu_mode = {'Set up':0,'Ready':1, 'Sine Str Term':2, 'GO Ready':3,
        #             'Pause':4, 'L2L':5, 'Const Str':6, 'Sine Str':7, 'L2L Term':8,'Not Connected':9}
        self.status = status
        self.mcu_mode = dict(zip(self.status.disp_Mode, range(len(self.status.disp_Mode))))

        self.last_status = 0

        username = getpass.getuser()
        self.path = "C:/Users/"+username+"/Desktop/data/"
        if not(os.path.isdir(self.path)):
            os.makedirs((os.path.join(self.path)))

    def save_data(self, CAN_name):
        mode = self.status.MCU_status
        if self.status.motor_status != self.mcu_mode["Set up"]:                                                                      # Motor On
            if mode == self.mcu_mode["L2L"] or mode == self.mcu_mode["Const_Str"] or mode == self.mcu_mode["Sine_Str"]:
                if self.last_status != mode and self.last_status != self.mcu_mode["Pause"]:                                          # Init
                    self.last_status = mode
                    self.file_name = self.path + datetime.now().strftime('%y%m%d_%H%M%S') +'_'+ list(self.mcu_mode.keys())[mode]
                    self.save_start = [self.status.serial_num] + self.status.can_num
            elif mode == self.mcu_mode["Pause"]:
                pass
            elif self.last_status == self.mcu_mode["L2L"] or self.last_status == self.mcu_mode["Const_Str"] or self.last_status == self.mcu_mode["Sine_Str"]:
                    save_finish = [self.status.serial_num] + self.status.can_num
                    for i in range(4):
                        if i != 0:
                            if save_finish[i] !=0 and CAN_name != None:
                                name = self.file_name + '_CAN_' + CAN_name[i - 1] + '.csv'
                                self.f = open(name,'w', newline='')
                                wr = csv.writer(self.f)
                                for num in range(self.save_start[i], save_finish[i]):
                                    wr.writerow([self.status.can_buff[i - 1][num]])
                        else:
                            name = self.file_name + '.csv'
                            self.f = open(name,'w', newline='')
                            wr = csv.writer(self.f)
                            wr.writerow(["Angle(deg)","Torque(N)"])
                            for num in range(self.save_start[i], save_finish[i]):
                                wr.writerow([self.status.ang_buff[num],self.status.tor_buff[num]])
                        self.f.close()
            self.last_status = mode