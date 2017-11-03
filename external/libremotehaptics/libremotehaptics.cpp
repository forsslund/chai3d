#include "libremotehaptics.h"
#include "networkhandler.h"

Libremotehaptics::Libremotehaptics() : netHandler(0)
{

}

bool Libremotehaptics::init(int port)
{
    netHandler = new NetworkHandler();
    return netHandler->init(port);
}


void Libremotehaptics::getPosition(Libremotehaptics::Vector3d &p)
{
    netHandler->getPosition(p);
}

void Libremotehaptics::getOrientation(Libremotehaptics::RotationMatrix &r)
{
    netHandler->getOrientation(r);
}

void Libremotehaptics::setForce(Libremotehaptics::Vector3d &p)
{
    netHandler->setForce(p);
}

void Libremotehaptics::setTorque(Libremotehaptics::Vector3d &t)
{
    netHandler->setTorque(t);
}

bool Libremotehaptics::getUserSwitch(int index)
{
    return netHandler->getUserSwitch(index);
}
