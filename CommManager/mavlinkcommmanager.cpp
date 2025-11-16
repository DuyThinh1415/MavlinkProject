// MavlinkCommManager.cpp
#include "mavlinkcommmanager.h"
#include <qdatetime.h>
#include <QFile>
#include <math.h>

MavlinkCommManager::MavlinkCommManager(QObject *parent)
    : QObject(parent) {
    socket.bind(QHostAddress::Any, 14550);
    connect(&socket, &QUdpSocket::readyRead, this, &MavlinkCommManager::readDatagram);
}

void MavlinkCommManager::sendHeartbeat() {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    // Hardcoded params: Quadcopter PX4, active, manual mode
    uint8_t sys_id = 1;        // System ID
    uint8_t comp_id = 1;       // Autopilot component
    uint8_t type = 2;          // MAV_TYPE_QUADROTOR
    uint8_t autopilot = 3;     // MAV_AUTOPILOT_PX4
    uint8_t base_mode = 0;     // No flags
    uint32_t custom_mode = 0;  // Manual mode
    uint8_t system_status = 4; // MAV_STATE_ACTIVE

    mavlink_msg_heartbeat_pack(sys_id, comp_id, &msg, type, autopilot, base_mode, custom_mode, system_status);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    QHostAddress addr("127.0.0.1");
    quint16 port = 14550;
    socket.writeDatagram((char*)buffer, len, addr, port);
}

void MavlinkCommManager::sendPacket(const Header& header, const uint8_t* body, uint16_t bodyLen) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    memcpy(buf, &header, sizeof(Header));
    memcpy(buf + sizeof(Header), body, bodyLen);
    socket.writeDatagram((char*)buf, sizeof(Header) + bodyLen, QHostAddress("127.0.0.1"), 14550);
}

void MavlinkCommManager::sendGlobalPosition(double lat, double lon, float alt, float vx, float vy, float vz)
{
    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(1, 1, &msg,
                                         QDateTime::currentMSecsSinceEpoch(), // time_boot_ms
                                         lat * 1e7, lon * 1e7, alt * 1000, // mm
                                         alt * 1000, // relative_alt
                                         vx * 100, vy * 100, vz * 100, // cm/s
                                         0); // hdg

    uint8_t body[MAVLINK_MAX_PACKET_LEN];
    uint16_t bodyLen = mavlink_msg_to_send_buffer(body, &msg);

    Header header;
    header.len = bodyLen - sizeof(Header); // payload len
    header.msgid[0] = MAVLINK_MSG_ID_GLOBAL_POSITION_INT & 0xFF;
    header.msgid[1] = (MAVLINK_MSG_ID_GLOBAL_POSITION_INT >> 8) & 0xFF;
    header.msgid[2] = (MAVLINK_MSG_ID_GLOBAL_POSITION_INT >> 16) & 0xFF;

    sendPacket(header, body + sizeof(Header), header.len);
}

void MavlinkCommManager::sendCameraDefinition(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) return;

    QByteArray data = file.readAll();
    file.close();

    uint16_t chunkSize = 239; // MAVLink v2 max payload common
    uint16_t total = data.size();
    uint16_t seq = 0;

    for (int offset = 0; offset < total; offset += chunkSize) {
        uint16_t len = fmin(chunkSize, total - offset);
        mavlink_message_t msg;

        // Dùng DATA_TRANSMISSION_HANDSHAKE + ENCLOSED_DATA (common)
        if (seq == 0) {
            mavlink_msg_data_transmission_handshake_pack(1, 1, &msg,
                                                         MAVLINK_TYPE_UINT8_T, total, 200, len, seq, 0, 0);
        }

        // Gửi phần dữ liệu bằng ENCLOSED_DATA
        mavlink_msg_encapsulated_data_pack(1, 1, &msg, seq, (uint8_t*)data.mid(offset, len).data());

        uint8_t body[MAVLINK_MAX_PACKET_LEN];
        uint16_t bodyLen = mavlink_msg_to_send_buffer(body, &msg);

        Header header;
        header.len = bodyLen - sizeof(Header);
        uint16_t msgid = (seq == 0) ? MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE : MAVLINK_MSG_ID_ENCAPSULATED_DATA;
        header.msgid[0] = msgid & 0xFF;
        header.msgid[1] = (msgid >> 8) & 0xFF;
        header.msgid[2] = (msgid >> 16) & 0xFF;

        sendPacket(header, body + sizeof(Header), header.len);
        seq++;
    }
}

void MavlinkCommManager::readDatagram() {
    while (socket.hasPendingDatagrams()) {
        qDebug() << "Pending ack: ";
        QByteArray datagram;
        datagram.resize(socket.pendingDatagramSize());
        QHostAddress sender; quint16 port;
        socket.readDatagram(datagram.data(), datagram.size(), &sender, &port);

        mavlink_message_t msg;
        mavlink_status_t status;
        for (int i = 0; i < datagram.size(); ++i) {
            if (mavlink_parse_char(0, datagram[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_command_ack_t ack;
                    mavlink_msg_command_ack_decode(&msg, &ack);
                    emit receivedAck(ack.command);
                    qDebug() << "QGC ACK: " << ack.command;
                }
            }
        }
    }
}
