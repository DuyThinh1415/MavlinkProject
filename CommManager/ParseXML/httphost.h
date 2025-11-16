#ifndef HTTPHOST_H
#define HTTPHOST_H

#include <QObject>
#include <QHttpServer>
#include <QHttpServerResponse>
#include <QFile>

class HttpHost : public QObject
{
    Q_OBJECT
public:
    explicit HttpHost(QObject* parent = nullptr)
        : QObject(parent), httpServer(new QHttpServer(this)), serverPort(0) {}

    bool start(const QString& xmlFilePath, quint16 port = 8080) {
        cameraDefPath = xmlFilePath;

        httpServer->route("/camera_definition.xml", [this]() {
            QFile file(cameraDefPath);
            if (!file.open(QIODevice::ReadOnly)) {
                return QHttpServerResponse(QHttpServerResponse::StatusCode::NotFound);
            }
            QByteArray data = file.readAll();
            file.close();
            return QHttpServerResponse("application/xml", data);
        });

        if (httpServer->listen(QHostAddress::Any, port)) {
            serverPort = port;
            return true;
        }
        return false;
    }

    void stop() {
        httpServer->disconnect();
        serverPort = 0;
    }

    QString getUri() const {
        if (serverPort > 0) {
            return QString("http://127.0.0.1:%1/camera_definition.xml").arg(serverPort);
        }
        return QString();
    }

    bool isRunning() const { return serverPort > 0; }

private:
    QHttpServer* httpServer;
    QString cameraDefPath;
    quint16 serverPort;
};

#endif // HTTPHOST_H
