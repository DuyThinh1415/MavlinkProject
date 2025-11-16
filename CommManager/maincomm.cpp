#include "maincomm.h"

MainComm MainComm::m_instance;

void MainComm::trigger1()
{

    if (mavlinkCommManager != nullptr){
        qDebug("delete it");
        delete mavlinkCommManager;
        mavlinkCommManager = nullptr;
    } else {
        qDebug("create it");
        mavlinkCommManager = new MavlinkCommManager;
    }
}

void MainComm::trigger2()
{
    if (mavlinkCommManager == nullptr) {
        qDebug("wtf bro ??? create it");
        return;
    }
    qDebug("send heartbeat");
    mavlinkCommManager->sendHeartbeat();
    // mavlinkCommManager.sendCameraDefinition("C://Users//Admin//Desktop//Mini Test//Camera_Definition_001.xml");

}

void MainComm::trigger3()
{
    if (mavlinkCommManager == nullptr) {
        qDebug("wtf bro ??? create it");
        return;
    }
    qDebug("send camera def");
    mavlinkCommManager->sendCameraDefinition("C://Users//Admin//Desktop//Mini Test//Camera_Definition_001.xml");

}

void MainComm::trigger4()
{
    if (mavlinkCommManager == nullptr) {
        qDebug("wtf bro ??? create it");
        return;
    }
    qDebug("hehe");

}

void MainComm::triggerMove()
{
    if (mavlinkCommManager == nullptr) {
        qDebug("wtf bro ??? create it");
        return;
    }
    qDebug("Moving ...");
    mavlinkCommManager->sendGlobalPosition(lastlongPos, 105.8542, 100.0f, 0, 0, 0);
    lastlongPos += 0.01;
}
