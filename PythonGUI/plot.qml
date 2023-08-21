import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import QMLPLOT 1.0

Item {
    Timer {
        interval: intv * 10000; 
        running: true 
        repeat: true
        onTriggered: {
            send_data.update_data(series, mode)
        }
    }

    Plot_data {
        id: send_data
    }

    ChartView {
        title: name
        titleFont: Qt.font({pointSize: 15, bold:true})
        id: chart
        anchors.fill: parent
        legend.visible: false

        ValueAxis {
            id: axisX
            min: xmin
            max: xmax
            tickCount: 5
        }

        ValueAxis {
            id: axisY
            min: ymin
            max: ymax
            tickCount: 5
        }
        LineSeries {
            id: series
            axisX: axisX
            axisY: axisY
            color:col
            width:2
        }
    }
}