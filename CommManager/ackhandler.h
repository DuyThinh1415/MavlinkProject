#ifndef ACKHANDLER_H
#define ACKHANDLER_H

#include <QObject>
#include <mavlink/install/include/mavlink/common/mavlink.h>
#include <QUdpSocket>

class AckHandler : public QObject
{
    Q_OBJECT
public:
    explicit AckHandler(QObject *parent = nullptr);
    QUdpSocket *socket; // get from manager, litte stupid, but i'm lazy TODO: update this
    void readQgcAck();
    void handleMessage(const mavlink_message_t& msg, const QHostAddress& sender, quint16 senderPort);

signals:
};

#endif // ACKHANDLER_H
