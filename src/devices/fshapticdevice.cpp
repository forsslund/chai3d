#include "fshapticdevice.h"
#include "fshapticdevicethread.h"

FsHapticDevice::FsHapticDevice()
{
    Kinematics::configuration c = Kinematics::configuration::polhem_v1();
    //Kinematics::configuration c = Kinematics::configuration::woodenhaptics_v2015();
    //Kinematics::configuration c = Kinematics::configuration::aluhaptics_v2();
    //Kinematics::configuration c = Kinematics::configuration::vintage();

    // Should the thread block and wait for at least one new position message before continue? (may improve stability)
    bool wait_for_next_message = true;

    // Create haptics communication thread. If configuration is emitted it reads from ~/woodenhaptics.json,
    // and if that file does not exist it creates it. Only available in unix-like systems for now.
    //FsHapticDeviceThread fs(wait_for_next_message);
    fsthread = new FsHapticDeviceThread(wait_for_next_message, c);

    std::cout << "FsHapticDevice() configuration used:\n" << toJSON(fsthread->kinematics.m_config) << std::endl;

fsthread = new FsHapticDeviceThread(wait_for_next_message, c);
}

FsHapticDevice::~FsHapticDevice()
{
    cout << "~FsHapticDevice()\n";

}

fsVec3d FsHapticDevice::getPos()
{
    return fsthread->getPos();
}

fsRot FsHapticDevice::getRot()
{
    return fsthread->getRot();
}

void FsHapticDevice::setForce(fsVec3d f)
{
    fsthread->setForce(f);
}

Kinematics::configuration FsHapticDevice::getConfig()
{
    return fsthread->kinematics.m_config;
}

void FsHapticDevice::open()
{
    fsthread->open();
}


