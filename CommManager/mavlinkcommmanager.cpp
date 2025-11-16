// MavlinkCommManager.cpp
#include "mavlinkcommmanager.h"
#include <qdatetime.h>
#include <QFile>
#include <QFileInfo>
#include <math.h>
#include <qxmlstream.h>
#include <CommManager/ParseXML/httphost.h>
#include <CommManager/ackhandler.h>

MavlinkCommManager::MavlinkCommManager(QObject *parent)
    : QObject(parent) {
    socket.bind(QHostAddress::Any, 14551);

    ackHandler.socket = &socket;
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



void MavlinkCommManager::sendCameraDefinition(const QString& xmlFilePath) {

    if (!testHttpHost.isRunning()) {
        if (testHttpHost.start(xmlFilePath,8080)){
            qDebug() << "Http sever start";
        } else {
            qDebug() << "fuck Http sever";
        }
    }


    QFile file(xmlFilePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot open camera definition file:" << xmlFilePath;
        return;
    }

    QString vendor = "Generic";
    QString model = "Camera";

    QXmlStreamReader xml(&file);
    while (!xml.atEnd()) {
        xml.readNext();
        if (xml.isStartElement()) {
            if (xml.name().toString() == "vendor") {
                vendor = xml.readElementText();
            } else if (xml.name().toString() == "model") {
                model = xml.readElementText();
            }
        }
    }
    file.close();


    mavlink_message_t msg;
    mavlink_camera_information_t cam_info;
    memset(&cam_info, 0, sizeof(cam_info));

    cam_info.time_boot_ms = QDateTime::currentMSecsSinceEpoch();

    strncpy((char*)cam_info.vendor_name, vendor.toUtf8().constData(),
            sizeof(cam_info.vendor_name) - 1);
    strncpy((char*)cam_info.model_name, model.toUtf8().constData(),
            sizeof(cam_info.model_name) - 1);

    cam_info.firmware_version = 0x01000000;
    cam_info.focal_length = 0.0f;
    cam_info.sensor_size_h = 0.0f;
    cam_info.sensor_size_v = 0.0f;
    cam_info.resolution_h = 1920;
    cam_info.resolution_v = 1080;
    cam_info.lens_id = 0;

    cam_info.flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                     CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                     CAMERA_CAP_FLAGS_HAS_MODES;

    cam_info.cam_definition_version = 0;

    strncpy((char*)cam_info.cam_definition_uri, testHttpHost.getUri().toUtf8().constData(),
            sizeof(cam_info.cam_definition_uri) - 1);

    uint8_t system_id = 1;
    uint8_t component_id = MAV_COMP_ID_CAMERA;

    mavlink_msg_camera_information_encode(system_id, component_id, &msg, &cam_info);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    socket.writeDatagram((const char*)buf, len, QHostAddress("127.0.0.1"), 14550);

    qDebug() << "Camera information sent:";
    qDebug() << "  Vendor:" << vendor;
    qDebug() << "  Model:" << model;
    qDebug() << "  Definition URI:" << testHttpHost.getUri();

}

void MavlinkCommManager::handleMavlinkMessage(const mavlink_message_t &msg, const QHostAddress &sender, quint16 senderPort)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);
        handleCommandLong(cmd, msg.sysid, msg.compid, sender, senderPort);
        break;
    }

    case MAVLINK_MSG_ID_CAMERA_INFORMATION: {
        mavlink_camera_information_t cam_info;
        mavlink_msg_camera_information_decode(&msg, &cam_info);
        qDebug() << "Received CAMERA_INFORMATION";
        break;
    }

    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
        qDebug() << "recive MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ";
        break;
    }

    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
        qDebug() << "recive MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST";
        break;
    }

    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
        qDebug() << "recive MAVLINK_MSG_ID_PARAM_EXT_SET";
        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
        mavlink_request_data_stream_t req;
        mavlink_msg_request_data_stream_decode(&msg, &req);
        qDebug() << "REQUEST_DATA_STREAM - Stream ID:" << req.req_stream_id
                 << "Rate:" << req.req_message_rate << "Hz"
                 << "Start/Stop:" << req.start_stop;

        mavlink_message_t ack_msg;
        mavlink_command_ack_t ack;

        ack.command = 0;
        ack.result = MAV_RESULT_ACCEPTED;
        ack.target_system = msg.sysid;
        ack.target_component = msg.compid;

        mavlink_msg_command_ack_encode(1, MAV_COMP_ID_CAMERA, &ack_msg, &ack);

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &ack_msg);

        socket.writeDatagram((const char*)buf, len, sender, senderPort);
        break;
    }

    default:
        qDebug() << "Unhandled MAVLink message ID:" << msg.msgid;
        break;
    }
}

void MavlinkCommManager::handleCommandLong(const mavlink_command_long_t &cmd, uint8_t target_system, uint8_t target_component, const QHostAddress &sender, quint16 senderPort)
{
    if (cmd.command == MAV_CMD_REQUEST_CAMERA_INFORMATION) {
        qDebug() << "Received REQUEST_CAMERA_INFORMATION";
        // resend CAMERA_INFORMATION
        // sendCameraInformation(&socket, cameraDefPath, httpServer->getUri());

        // Gửi ACK
        // sendCommandAck(cmd.command, MAV_RESULT_ACCEPTED, target_system, target_component, sender, senderPort);
    }
}

void MavlinkCommManager::sendCommandAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, const QHostAddress &sender, quint16 senderPort)
{
    mavlink_message_t msg;
    mavlink_command_ack_t ack;

    ack.command = command;
    ack.result = result;
    ack.target_system = target_system;
    ack.target_component = target_component;

    mavlink_msg_command_ack_encode(1, MAV_COMP_ID_CAMERA, &msg, &ack);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    socket.writeDatagram((const char*)buf, len, sender, senderPort);
}

void MavlinkCommManager::readDatagram() {
    while (socket.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(socket.pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        mavlink_message_t msg;
        mavlink_status_t status;

        qDebug() << "Received datagram, first byte:" << QString::number((uint8_t)datagram[0], 16);

        for (int i = 0; i < datagram.size(); i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, datagram[i], &msg, &status)) {
                handleMavlinkMessage(msg, sender, senderPort);
            }
        }
    }
}
