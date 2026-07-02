//==============================================================================
/*
    CHAI3D haptic-device backend for the open SensAble Phantom driver
    (open_phantom / libopenphantom). Drives the Phantom Premium 1.5 / 6DOF over
    soft-EPP with no vendor OpenHaptics stack, using open forward kinematics.

    Enabled by the CMake option USE_OPENPHANTOM (which also disables the vendor
    cPhantomDevice so only one backend claims the parport). Mirrors the
    cMyCustomDevice template / cHaptikfabrikenDevice pattern.

    \author    open_phantom
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef COpenPhantomDeviceH
#define COpenPhantomDeviceH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_OPENPHANTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------
// forward-declared so the public CHAI3D header stays free of the C driver header
struct op_handle;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       COpenPhantomDevice.h
    \brief      Open (vendor-free) backend for the Phantom Premium 1.5 / 6DOF.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cOpenPhantomDevice;
typedef std::shared_ptr<cOpenPhantomDevice> cOpenPhantomDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cOpenPhantomDevice
    \ingroup    devices

    \brief      Interfaces CHAI3D to the open soft-EPP Phantom driver.
*/
//==============================================================================
class cOpenPhantomDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    cOpenPhantomDevice(unsigned int a_deviceNumber = 0);
    virtual ~cOpenPhantomDevice();

    static cOpenPhantomDevicePtr create(unsigned int a_deviceNumber = 0)
        { return (std::make_shared<cOpenPhantomDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    virtual bool open();
    virtual bool close();
    virtual bool calibrate(bool a_forceCalibration = false);
    virtual bool getPosition(cVector3d& a_position);
    virtual bool getRotation(cMatrix3d& a_rotation);
    virtual bool getGripperAngleRad(double& a_angle);
    virtual bool getUserSwitches(unsigned int& a_userSwitches);
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                  const cVector3d& a_torque,
                                                  double a_gripperForce);


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public:

    static unsigned int getNumDevices();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! handle to the open driver (libopenphantom), NULL until open()
    op_handle* m_op;

    //! parport device node this backend drives
    std::string m_dev;
};

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_OPENPHANTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
