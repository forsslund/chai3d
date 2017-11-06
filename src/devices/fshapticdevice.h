#ifndef FSHAPTICDEVICE_H
#define FSHAPTICDEVICE_H

#include "kinematics.h"
class FsHapticDeviceThread;

class FsHapticDevice
{
public:
    FsHapticDevice();
    fsVec3d getPos();
    fsRot getRot();
    void setForce(fsVec3d f);
    FsHapticDeviceThread *fsthread;
};

#endif // FSHAPTICDEVICE_H
