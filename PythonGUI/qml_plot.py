import os
import sys

from PyQt5.QtCore import QObject, pyqtSlot, QUrl, QPointF
from PyQt5.QtQuick import QQuickView
from PyQt5.QtWidgets import *
from PyQt5.QtQml import qmlRegisterType
from PyQt5 import QtChart
import numpy as np
from Steer_Status import Steer_Status
DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        push_button = QPushButton("here")
        
        lay = QHBoxLayout(central_widget)
        lay.addWidget(push_button)
        
        lay.addWidget(qml_Chart("angle", 0).widget)
        lay.addWidget(qml_Chart("torque", 1).widget)
        lay.addWidget(qml_Chart("can", 2).widget)

class Plot_data(QObject):
    def __init__(self, parent=None):
        super(Plot_data, self).__init__(parent)
        self.status = Steer_Status()
        self.x = self.status.time_buff[0]
        self.y = np.zeros(len(self.x))
        self.set_data_list()
        
    @pyqtSlot(QtChart.QAbstractSeries, int)
    def update_data(self, series, num):
        self.x = self.status.time_buff[num]
        if num == 0:
            self.y = self.status.ang_buff[-len(self.x):]
        elif num == 1:
            self.y = self.status.tor_buff[-len(self.x):]
        else:
            self.y = self.status.can_buff[num - 2][-len(self.x):]
        self.y = [0] * (len(self.x) - len(self.y)) + self.y
        self.set_data_list()
        series.replace(self.list)
    
    def set_data_list(self):
        data_list = []
        for a,b in zip(self.x, self.y):
            data_list.append(QPointF(a,b))
        self.list = data_list

class qml_Chart(QWidget):
    def __init__(self, title, num):
        super().__init__()
        self.status = Steer_Status()
        file = os.path.join(DIR_PATH, "plot.qml")
        view = QQuickView()
        view.statusChanged.connect(self.on_statusChanged)
        view.setResizeMode(QQuickView.SizeRootObjectToView)
        
        engine = view.engine()
        self.context = engine.rootContext()
        self.context.setContextProperty("name", title)
        self.context.setContextProperty("col", ["blue","green","red","blue","green"][num])
        self.context.setContextProperty("mode", num)
        self.context.setContextProperty("intv", self.status.time_interval)
        self.context.setContextProperty("xmin", self.status.axis_limit[num][0])
        self.context.setContextProperty("xmax", self.status.axis_limit[num][1])
        self.context.setContextProperty("ymin", self.status.axis_limit[num][2])
        self.context.setContextProperty("ymax", self.status.axis_limit[num][3])
        
        qmlRegisterType(Plot_data, 'QMLPLOT', 1, 0, 'Plot_data')
        view.setSource(QUrl.fromLocalFile(file))
        self.widget = QWidget.createWindowContainer(view)
        
    def on_statusChanged(self, status):
        if status == QQuickView.Error:
            for error in self.view.errors():
                print(error.toString())
            sys.exit(-1)

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.showMaximized()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()