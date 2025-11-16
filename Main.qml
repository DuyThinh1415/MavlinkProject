import QtQuick 2.15
import QtQuick.Controls 2.15
import com.example 1.0

ApplicationWindow {
    width: 400; height: 500; visible: true

    Column {
        anchors.centerIn: parent; spacing: 10
        Button {
            text: "create"
            onClicked: MavlinkComm.trigger1()
            width: 120
            height: 50
        }
        Button {
            text: "Heartbeat once"
            onClicked: MavlinkComm.trigger2()
            width: 120
            height: 50
        }
        Button {
            text: "cam def"
            onClicked: MavlinkComm.trigger3()
            width: 120
            height: 50
        }
        Button {
            text: "Trigger 4"
            onClicked: MavlinkComm.trigger4()
            width: 120
            height: 50
        }

        Button {
            text: "Timer Switch"
            onClicked: {
                moveTimer.running != moveTimer.running
            }

            width: 120
            height: 50
        }

        Timer{
            id: moveTimer
            interval: 1000
            repeat: true
            running: false
            onTriggered: {
                MavlinkComm.triggerMove()
            }
        }
    }
}
