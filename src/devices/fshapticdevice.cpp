#include "fshapticdevice.h"
#include "fshapticdevicethread.h"

FsHapticDevice::FsHapticDevice()
{
    fsthread = new FsHapticDeviceThread();
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


