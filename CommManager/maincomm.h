#ifndef MAINCOMM_H
#define MAINCOMM_H

#include <QObject>
#include <CommManager/mavlinkcommmanager.h>

class MainComm : public QObject {
    Q_OBJECT
public:
    static MainComm* instance() { return &m_instance; }
    Q_INVOKABLE void trigger1();
    Q_INVOKABLE void trigger2();
    Q_INVOKABLE void trigger3();
    Q_INVOKABLE void trigger4();

    Q_INVOKABLE void triggerMove(); // just to make sure it connected

private:
    MainComm() = default;
    static MainComm m_instance;
    MavlinkCommManager* mavlinkCommManager;

    float lastlongPos = 20;
};

#endif // MAINCOMM_H
