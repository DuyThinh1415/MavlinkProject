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
            id: hbButton
            property int hbStatus: 0
            text: hbStatus?"Heartbeat ON":"Heartbeat OFF"
            onClicked: hbStatus = 1 - hbStatus
            width: 120
            height: 50
            Timer{
                interval: 1000
                repeat: true
                running: hbButton.hbStatus
                onTriggered: {
                    MavlinkComm.trigger2()
                    // MavlinkComm.trigger3()
                }
            }
        }
        Button {
            text: "Send cam def once"
            onClicked: MavlinkComm.trigger3()
            width: 120
            height: 50
        }
        Button {
            text: basicStreamTimer.running?"Basic Stream running":"Basic Stream"
            onClicked: {
                basicStreamTimer.running = !basicStreamTimer.running
            }

            width: 120
            height: 50
        }

        Timer{
            id: basicStreamTimer
            interval: 100
            repeat: true
            running: false
            onTriggered: {
                MavlinkComm.trigger4()
            }
        }

        Button {
            text: "Timer Switch"
            onClicked: {
                moveTimer.running = !moveTimer.running
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
