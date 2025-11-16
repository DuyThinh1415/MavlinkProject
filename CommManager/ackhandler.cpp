#include "ackhandler.h"

#include <QUdpSocket.h>
#include <mavlink/install/include/mavlink/common/mavlink.h>

AckHandler::AckHandler(QObject *parent)
    : QObject{parent}
{}

void AckHandler::readQgcAck()
{
    while (socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < datagram.size(); i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, datagram[i], &msg, &status)) {
                handleMessage(msg, sender, senderPort);
            }
        }
    }
}

void AckHandler::handleMessage(const mavlink_message_t &msg, const QHostAddress &sender, quint16 senderPort)
{

}
