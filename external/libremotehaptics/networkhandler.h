#ifndef NETWORKHANDLER_H
#define NETWORKHANDLER_H

#include <QThread>
#include <QUdpSocket>
#include <QMutex>
#include "libremotehaptics.h"

class NetworkHandler : public QThread
{
    Q_OBJECT

public:
    explicit NetworkHandler(QObject *parent = 0);
    bool init(int port);
    void getPosition(Libremotehaptics::Vector3d &p);
    void getOrientation(Libremotehaptics::RotationMatrix &r);
    void setForce(Libremotehaptics::Vector3d &p);
    void setTorque(Libremotehaptics::Vector3d &t);
    bool getUserSwitch(int index=0);
    
signals:
    
public slots:
    void readPendingDatagrams();

protected:
    virtual void run();

    QUdpSocket *udpSocket;
    Libremotehaptics::Vector3d lastPosition;
    Libremotehaptics::RotationMatrix lastOrientation;
    bool lastUserSwitchState[2];
    Libremotehaptics::Vector3d nextForce;
    Libremotehaptics::Vector3d nextTorque;

    QMutex nextForceTorqueMutex;
    QMutex lastPositionOrientaionMutex;
};

#endif // NETWORKHANDLER_H
