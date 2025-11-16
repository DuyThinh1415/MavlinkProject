#ifndef PARSEXML_H
#define PARSEXML_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>

class ParseXML : public QObject {
    Q_OBJECT
public:
    struct Param {
        QString name, type, def;
        int control = 0;
        QMap<QString, QString> options;
    };
    QList<Param> params;
    bool parse(const QString& resourcePath);
};

#endif // PARSEXML_H
