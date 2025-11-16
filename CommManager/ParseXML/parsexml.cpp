#include "ParseXML.h"
#include <QFile>
#include <QXmlStreamReader>
#include <QDebug>

bool ParseXML::parse(const QString& resourcePath) {
    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Open fail:" << resourcePath;
        return false;
    }
    QXmlStreamReader xml(&file);
    while (!xml.atEnd() && !xml.hasError()) {
        QXmlStreamReader::TokenType token = xml.readNext();
        if (token == QXmlStreamReader::StartElement) {
            if (xml.name() == QLatin1String("parameter")) {
                Param p;
                auto attrs = xml.attributes();
                p.name = attrs.value("name").toString();
                p.type = attrs.value("type").toString();
                p.def = attrs.value("default").toString();
                p.control = attrs.value("control").toInt();
                // Đọc con
                while (!xml.atEnd() && !xml.hasError()) {
                    token = xml.readNext();
                    if (token == QXmlStreamReader::StartElement && xml.name() == QLatin1String("option")) {
                        auto opt = xml.attributes();
                        QString optName = opt.value("name").toString();
                        QString optVal = opt.value("value").toString();
                        p.options[optName] = optVal;
                    } else if (token == QXmlStreamReader::EndElement && xml.name() == QLatin1String("parameter")) {
                        break;
                    }
                }
                params.append(p);
            }
        }
    }
    if (xml.hasError()) {
        qDebug() << "XML error:" << xml.errorString();
        return false;
    }
    return true;
}
