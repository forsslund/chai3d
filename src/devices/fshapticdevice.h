#ifndef FSHAPTICDEVICE_H
#define FSHAPTICDEVICE_H

#include "kinematics.h"
class FsHapticDeviceThread;

class FsHapticDevice
{
public:
    FsHapticDevice();
    ~FsHapticDevice();
    fsVec3d getPos();
    fsRot getRot();
    void setForce(fsVec3d f);
    FsHapticDeviceThread *fsthread=0;
    Kinematics::configuration getConfig();
    void open();
};

#endif // FSHAPTICDEVICE_H
