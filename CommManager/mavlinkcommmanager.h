// mavlinkcommmanager.h
#pragma once
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

private:
    QUdpSocket socket;

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

    void sendPacket(const Header& header, const uint8_t* body, uint16_t bodyLen);

signals:
    void receivedAck(uint16_t command);

private slots:
    void readDatagram();

};
