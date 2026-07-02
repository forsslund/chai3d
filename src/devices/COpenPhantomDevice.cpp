//==============================================================================
/*
    CHAI3D backend for the open Phantom driver — see COpenPhantomDevice.h.

    Thin wrapper over libopenphantom (open_phantom): open() starts the soft-EPP
    servo + our forward kinematics, getPosition() reads the cached pose, and
    setForceAndTorqueAndGripperForce() feeds a Cartesian force into the J^T path.
    Motor amps stay OFF until explicitly enabled in the driver (safe to develop
    the force path with the amp-box switches off).

    \author    open_phantom
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/COpenPhantomDevice.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_OPENPHANTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "libopenphantom.h"   // C API from open_phantom (extern "C")
#include <unistd.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

// parport node the Phantom lives on (blanca: Axxon OXPCIe952 -> /dev/parport0)
static const char* OP_DEFAULT_DEV = "/dev/parport0";


//==============================================================================
// Constructor: declare the device specifications (Premium 1.5 / 6DOF).
//==============================================================================
cOpenPhantomDevice::cOpenPhantomDevice(unsigned int a_deviceNumber)
    : m_op(NULL), m_dev(OP_DEFAULT_DEV)
{
    m_deviceReady = false;
    m_deviceNumber = a_deviceNumber;

    // --- identity ---
    m_specifications.m_model            = C_HAPTIC_DEVICE_PHANTOM_15_6DOF;
    m_specifications.m_manufacturerName = "SensAble (open driver)";
    m_specifications.m_modelName        = "Phantom Premium 1.5 / 6DOF";

    // --- characteristics (from the Windows ground-truth config) ---
    m_specifications.m_maxLinearForce          = 7.9;     // [N]  NominalMaxForce
    m_specifications.m_maxAngularTorque        = 0.0;     // [N*m] gimbal torque not yet driven
    m_specifications.m_maxGripperForce         = 0.0;     // [N]
    m_specifications.m_maxLinearStiffness      = 800.0;   // [N/m] conservative for a 1 kHz loop
    m_specifications.m_maxAngularStiffness     = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness = 0.0;   // [N/m]
    m_specifications.m_workspaceRadius         = 0.10;    // [m]  ~Premium 1.5 half-workspace
    m_specifications.m_gripperMaxAngleRad      = 0.0;

    // --- damping ---
    m_specifications.m_maxLinearDamping        = 8.0;     // [N/(m/s)]
    m_specifications.m_maxAngularDamping       = 0.0;
    m_specifications.m_maxGripperAngularDamping= 0.0;

    // --- capabilities (first pass: 3-DOF position + force; no wrist/gripper) ---
    m_specifications.m_sensedPosition          = true;
    m_specifications.m_sensedRotation          = false;   // gimbal field map unresolved
    m_specifications.m_sensedGripper           = false;
    m_specifications.m_actuatedPosition        = true;
    m_specifications.m_actuatedRotation        = false;
    m_specifications.m_actuatedGripper         = false;
    m_specifications.m_leftHand                = true;
    m_specifications.m_rightHand               = true;

    // availability: is the parport node reachable?
    m_deviceAvailable = (access(m_dev.c_str(), R_OK | W_OK) == 0);
}


//==============================================================================
cOpenPhantomDevice::~cOpenPhantomDevice()
{
    if (m_deviceReady) close();
}


//==============================================================================
unsigned int cOpenPhantomDevice::getNumDevices()
{
    // one Phantom on the parport; present if the node is accessible
    return (access(OP_DEFAULT_DEV, R_OK | W_OK) == 0) ? 1 : 0;
}


//==============================================================================
bool cOpenPhantomDevice::open()
{
    if (!m_deviceAvailable) return (C_ERROR);
    if (m_deviceReady)      return (C_ERROR);

    // start the soft-EPP servo + kinematics (6DOF: Double-EPP)
    m_op = op_open(m_dev.c_str(), OP_PREMIUM_6DOF);
    if (m_op == NULL)
    {
        m_deviceReady = false;
        return (C_ERROR);
    }

    m_deviceReady = true;
    return (C_SUCCESS);
}


//==============================================================================
bool cOpenPhantomDevice::close()
{
    if (!m_deviceReady) return (C_ERROR);

    if (m_op) { op_close(m_op); m_op = NULL; }   // zero-forces on the way out

    m_deviceReady = false;
    return (C_SUCCESS);
}


//==============================================================================
bool cOpenPhantomDevice::calibrate(bool a_forceCalibration)
{
    if (!m_deviceReady) return (C_ERROR);
    // The Phantom self-zeros its encoders at power-up; nothing to do here yet.
    // (Future: park-pose homing via op_set_config theta_offset.)
    (void)a_forceCalibration;
    return (C_SUCCESS);
}


//==============================================================================
bool cOpenPhantomDevice::getPosition(cVector3d& a_position)
{
    if (!m_deviceReady || !m_op) return (C_ERROR);

    op_state st;
    op_get_state(m_op, &st);                 // st.pos already in CHAI3D frame [m]
    a_position.set(st.pos[0], st.pos[1], st.pos[2]);

    estimateLinearVelocity(a_position);
    return (C_SUCCESS);
}


//==============================================================================
bool cOpenPhantomDevice::getRotation(cMatrix3d& a_rotation)
{
    if (!m_deviceReady || !m_op) return (C_ERROR);

    // Gimbal orientation not yet solved (motor D seized, D/E/F field map open).
    // Report identity; estimateAngularVelocity keeps the velocity signal sane.
    cMatrix3d frame;
    frame.identity();
    a_rotation = frame;

    estimateAngularVelocity(a_rotation);
    return (C_SUCCESS);
}


//==============================================================================
bool cOpenPhantomDevice::getGripperAngleRad(double& a_angle)
{
    if (!m_deviceReady) return (C_ERROR);
    a_angle = 0.0;
    estimateGripperVelocity(a_angle);
    return (C_SUCCESS);
}


//==============================================================================
bool cOpenPhantomDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                          const cVector3d& a_torque,
                                                          double a_gripperForce)
{
    if (!m_deviceReady || !m_op) return (C_ERROR);

    m_prevForce        = a_force;
    m_prevTorque       = a_torque;
    m_prevGripperForce = a_gripperForce;

    // Cartesian force [N] -> driver (J^T -> base DACs happens in the servo).
    double f[3]   = { a_force(0),  a_force(1),  a_force(2)  };
    double tau[3] = { a_torque(0), a_torque(1), a_torque(2) };  // reserved (gimbal)
    op_set_force(m_op, f, tau);

    return (C_SUCCESS);
}


//==============================================================================
bool cOpenPhantomDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    if (!m_deviceReady || !m_op) return (C_ERROR);
    op_state st;
    op_get_state(m_op, &st);
    a_userSwitches = st.buttons;
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_OPENPHANTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
