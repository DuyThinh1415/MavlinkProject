// mavlinkcommmanager.h
#pragma once
#include "CommManager/ParseXML/httphost.h"
#include "CommManager/ackhandler.h"
#include <QObject>
#include <QUdpSocket>
#include <mavlink/install/include/mavlink/common/mavlink.h>

class MavlinkCommManager : public QObject {
    Q_OBJECT
public:
    explicit MavlinkCommManager(QObject *parent = nullptr);
    void sendHeartbeat();
    void sendGlobalPosition(double lat, double lon, float alt, float vx, float vy, float vz);
    void sendCameraDefinition(const QString& filePath);
    void sendCameraDefinitionV2();
    void sendAllBasicStreams(mavlink_system_t sys);
    void sendCameraParam(const QString& xmlFilePath);

    mavlink_system_t mavlink_system = {
        .sysid = 1,     // ID
        .compid = 1     // MAV_COMP_ID_AUTOPILOT1
    };

private:
    QUdpSocket socket;
    HttpHost testHttpHost; // test http for cam def
    AckHandler ackHandler;

    void handleMavlinkMessage(const mavlink_message_t& msg,
                         const QHostAddress& sender,
                         quint16 senderPort);

    void handleCommandLong(const mavlink_command_long_t& cmd,
                           uint8_t target_system, uint8_t target_component,
                           const QHostAddress& sender, quint16 senderPort);

    void sendCommandAck(uint16_t command, uint8_t result,
                   uint8_t target_system, uint8_t target_component,
                   const QHostAddress& sender, quint16 senderPort);


    // Header struct (MAVLink v2)
    struct Header {
        uint8_t magic = 0xFD;
        uint8_t len;
        uint8_t incompat_flags = 0;
        uint8_t compat_flags = 0;
        uint8_t seq = 0;
        uint8_t sysid = 1;
        uint8_t compid = 1;
        uint8_t msgid[3] = {0,0,0}; // HEARTBEAT msgid = 0
    };

    struct CameraParam {
        char name[17];         // 16 ký tự + null
        MAV_PARAM_TYPE type;   // INT32 hoặc REAL32
        union {
            float f;
            int32_t i;
        } value;
    };

    void sendPacket(const Header& header, const uint8_t* body, uint16_t bodyLen);



signals:
    void receivedAck(uint16_t command);

private slots:
    void readDatagram();

};
