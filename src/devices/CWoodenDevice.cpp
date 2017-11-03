//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    Jonas Forsslund
    \author    Royal Institute of Technology
    \version   3.0.0 $Rev: 1244 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"

//------------------------------------------------------------------------------
// Define in src/system/CGlobals.h !
#if defined(C_ENABLE_WOODEN_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

#include "devices/CWoodenDevice.h"

// Following includes are only used for reading/writing config file and to find 
// the user's home directory (where the config file will be stored)
#include <iostream>
#include <fstream> 
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

// For USB HID version
#include "hidapi.h"
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include "hidapi.h"


#define FSDEVICE
//#define USB  // define this to use the usb version
//#define DELAY /// To test delay
//#define PWM        // For PWM from DAQ, do not use with USB
//#define SAVE_LOG
//#define SERIAL  // for reading the orientation

#ifndef USB
#define SENSORAY
#include "../../external/s826/include/826api.h"
#endif

// For delay testing
#include <chrono>
#include <thread>
#include <ratio>




#ifdef SERIAL
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <math/CQuaternion.h>
#endif


////////////////////////////////////////////////////////////////////////////////
/*
    This file is based upon CMyCustomDevice.cpp and communicates directly
    with a Sensoray S826 PCIe DAQ board that will read encoder values,
    calculate a position, and set motor torques (through analogue output).

    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file CWoodenDevice.h for some initial 
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    bellow.
*/
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------





std::string toJSON(const woodenhaptics_message& m) {
    std::stringstream ss;
    ss << "{" << std::endl <<
        "  'position_x':       " << m.position_x << "," << std::endl <<
        "  'position_y':       " << m.position_y << "," << std::endl <<
        "  'position_z':       " << m.position_z << "," << std::endl <<
        "  'command_force_x':  " << m.command_force_x << "," << std::endl <<
        "  'command_force_y':  " << m.command_force_y << "," << std::endl <<
        "  'command_force_z':  " << m.command_force_z << "," << std::endl <<
        "  'actual_current_0': " << m.actual_current_0 << "," << std::endl <<
        "  'actual_current_1': " << m.actual_current_1 << "," << std::endl <<
        "  'actual_current_2': " << m.actual_current_2 << "," << std::endl <<
        "  'temperture_0':     " << m.temperature_0 << "," << std::endl <<
        "  'temperture_1':     " << m.temperature_1 << "," << std::endl <<
        "  'temperture_2':     " << m.temperature_2 << "," << std::endl <<
        "}" << std::endl;

    return ss.str();
}

//==============================================================================
// WoodenHaptics configuration helper files.
//==============================================================================

cWoodenDevice::configuration default_woody(){
    double data[] = { 0, 0.010, 0.010, 0.010,
                      0.080, 0.205, 0.245,
                      0.160, 0.120, 0.120,
                      0.220, 0.000, 0.080, 0.100, 
                      0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                      5.0, 1000.0, 8.0,
                      0.170, 0.110, 0.051, 0.091, 0};
    return cWoodenDevice::configuration(data); 
}

double v(const std::string& json, const std::string& key){
    int p = json.find(":", json.find(key));
    return atof(json.substr(p+1).c_str());
}

cWoodenDevice::configuration fromJSON(std::string json){
    double d[]= {
        v(json,"variant"),
        v(json,"diameter_capstan_a"),
        v(json,"diameter_capstan_b"),      
        v(json,"diameter_capstan_c"),      
        v(json,"length_body_a"),           
        v(json,"length_body_b"),           
        v(json,"length_body_c"),           
        v(json,"diameter_body_a"),         
        v(json,"diameter_body_b"),         
        v(json,"diameter_body_c"),         
        v(json,"workspace_origin_x"),      
        v(json,"workspace_origin_y"),      
        v(json,"workspace_origin_z"),      
        v(json,"workspace_radius"),      
        v(json,"torque_constant_motor_a"), 
        v(json,"torque_constant_motor_b"), 
        v(json,"torque_constant_motor_c"), 
        v(json,"current_for_10_v_signal"), 
        v(json,"cpr_encoder_a"), 
        v(json,"cpr_encoder_b"),
        v(json,"cpr_encoder_c"),  
        v(json,"max_linear_force"),   
        v(json,"max_linear_stiffness"), 
        v(json,"max_linear_damping"), 
        v(json,"mass_body_b"),
        v(json,"mass_body_c"),
        v(json,"length_cm_body_b"),
        v(json,"length_cm_body_c"),
        v(json,"g_constant")       
    }; 
    return cWoodenDevice::configuration(d);
}

std::string j(const std::string& key, const double& value){
   std::stringstream s;
   s << "    \"" << key << "\":";
   while(s.str().length()<32) s<< " ";
   s << value << "," << std::endl;
   return s.str();
}
std::string toJSON(const cWoodenDevice::configuration& c){
   using namespace std;
   stringstream json;
   json << "{" << endl
        << j("variant",c.variant)
        << j("diameter_capstan_a",c.diameter_capstan_a)
        << j("diameter_capstan_b",c.diameter_capstan_b)
        << j("diameter_capstan_c",c.diameter_capstan_c)
        << j("length_body_a",c.length_body_a)
        << j("length_body_b",c.length_body_b)
        << j("length_body_c",c.length_body_c)
        << j("diameter_body_a",c.diameter_body_a)
        << j("diameter_body_b",c.diameter_body_b)
        << j("diameter_body_c",c.diameter_body_c)
        << j("workspace_origin_x",c.workspace_origin_x)
        << j("workspace_origin_y",c.workspace_origin_y)
        << j("workspace_origin_z",c.workspace_origin_z)
        << j("workspace_radius",c.workspace_radius)
        << j("torque_constant_motor_a",c.torque_constant_motor_a)
        << j("torque_constant_motor_b",c.torque_constant_motor_b)
        << j("torque_constant_motor_c",c.torque_constant_motor_c)
        << j("current_for_10_v_signal",c.current_for_10_v_signal)
        << j("cpr_encoder_a",c.cpr_encoder_a)
        << j("cpr_encoder_b",c.cpr_encoder_b)
        << j("cpr_encoder_c",c.cpr_encoder_c)
        << j("max_linear_force",c.max_linear_force)
        << j("max_linear_stiffness",c.max_linear_stiffness)
        << j("max_linear_damping",c.max_linear_damping)
        << j("mass_body_b",c.mass_body_b)
        << j("mass_body_c",c.mass_body_c)
        << j("length_cm_body_b",c.length_cm_body_b)
        << j("length_cm_body_c",c.length_cm_body_c)
        << j("g_constant",c.g_constant)
        << "}" << endl;
   return json.str();
}

void write_config_file(const cWoodenDevice::configuration& config){
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Writing configuration to: "<< homedir 
              << "/woodenhaptics.json" << std::endl;
    std::ofstream ofile;
    ofile.open(std::string(homedir) + "/woodenhaptics.json");
    ofile << toJSON(config);
    ofile.close();
}

cWoodenDevice::configuration read_config_file(){
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Trying loading configuration from: "<< homedir 
              << "/woodenhaptics.json" << std::endl;

    std::ifstream ifile;
    ifile.open(std::string(homedir) + "/woodenhaptics.json");
    if(ifile.is_open()){
        std::stringstream buffer;
        buffer << ifile.rdbuf();
        ifile.close();
        std::cout << "Success. " << std::endl;
        return fromJSON(buffer.str());    
    } else {
        std::cout << "File not found. We will write one "
                  << "based on default configuration values." << std::endl;

        write_config_file(default_woody());
        return default_woody();
    }
}
//==============================================================================



//==============================================================================
/*!
    Constructor of cWoodenDevice.
*/
//==============================================================================
cWoodenDevice::cWoodenDevice(unsigned int a_deviceNumber): 
    m_config(read_config_file())
{
    // the connection to your device has not yet been established.
    m_deviceReady = false;
    lost_messages = 0;

    for(int i=0;i<3;++i)
        global_pwm_percent[i]=0.1;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 1:

        Here you should define the specifications of your device.
        These values only need to be estimates. Since haptic devices often perform
        differently depending of their configuration withing their workspace,
        simply use average values.
    */
    ////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    // NAME: WoodenHaptics
    //--------------------------------------------------------------------------

    // If we have a config file, use its values, otherwise, 
    // use standard values (and write them to config file)
    std::cout << std::endl << "WoodenHaptics configuration used: " << std::endl 
              << toJSON(m_config) << std::endl; 

    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_HAPTIC_DEVICE_WOODEN;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "KTH and Stanford University";

    // name of your device
    m_specifications.m_modelName                     = "WoodenHaptics";


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following values must be positive or equal to zero)
    //--------------------------------------------------------------------------

    // the maximum force [N] the device can produce along the x,y,z axis.
    m_specifications.m_maxLinearForce                = m_config.max_linear_force;     // [N]

    // the maximum amount of torque your device can provide arround its
    // rotation degrees of freedom.
    m_specifications.m_maxAngularTorque              = 0.2;     // [N*m]


    // the maximum amount of torque which can be provided by your gripper
    m_specifications.m_maxGripperForce               = 3.0;     // [N]

    // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
    m_specifications.m_maxLinearStiffness             = m_config.max_linear_stiffness; // [N/m]

    // the maximum amount of angular stiffness
    m_specifications.m_maxAngularStiffness            = 1.0;    // [N*m/Rad]

    // the maximum amount of stiffness supported by the gripper
    m_specifications.m_maxGripperLinearStiffness      = 1000;   // [N*m]

    // the radius of the physical workspace of the device (x,y,z axis)
    m_specifications.m_workspaceRadius                = m_config.workspace_radius;     // [m]

    // the maximum opening angle of the gripper
    m_specifications.m_gripperMaxAngleRad             = cDegToRad(30.0);


    ////////////////////////////////////////////////////////////////////////////
    /*
        DAMPING PROPERTIES:

        Start with small values as damping terms can be high;y sensitive to 
        the quality of your velocity signal and the spatial resolution of your
        device. Try gradually increasing the values by using example "01-devices" 
        and by enabling viscosity with key command "2".
    */
    ////////////////////////////////////////////////////////////////////////////
    
    // Maximum recommended linear damping factor Kv
    m_specifications.m_maxLinearDamping			      = m_config.max_linear_damping;   // [N/(m/s)]

    //! Maximum recommended angular damping factor Kv (if actuated torques are available)
    m_specifications.m_maxAngularDamping			  = 0.0;	  // [N*m/(Rad/s)]

    //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
    m_specifications.m_maxGripperAngularDamping		  = 0.0; // [N*m/(Rad/s)]


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following are of boolean type: (true or false)
    //--------------------------------------------------------------------------

    // does your device provide sensed position (x,y,z axis)?
    m_specifications.m_sensedPosition                = true;

    // does your device provide sensed rotations (i.e stylus)?
    m_specifications.m_sensedRotation                = true;

    // does your device provide a gripper which can be sensed?
    m_specifications.m_sensedGripper                 = false;

    // is you device actuated on the translation degrees of freedom?
    m_specifications.m_actuatedPosition              = true;

    // is your device actuated on the rotation degrees of freedom?
    m_specifications.m_actuatedRotation              = false;

    // is the gripper of your device actuated?
    m_specifications.m_actuatedGripper               = false;

    // can the device be used with the left hand?
    m_specifications.m_leftHand                      = true;

    // can the device be used with the right hand?
    m_specifications.m_rightHand                     = true;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 2:

        Here, you shall  implement code which tells the application if your
        device is actually connected to your computer and can be accessed.
        In practice this may be consist in checking if your I/O board
        is active or if your drivers are available.

        If your device can be accessed, set:
        m_systemAvailable = true;

        Otherwise set:
        m_systemAvailable = false;

        Your actual code may look like:

        bool result = checkIfMyDeviceIsAvailable()
        m_systemAvailable = result;

        If want to support multiple devices, using the method argument
        a_deviceNumber to know which device to setup
    */  
    ////////////////////////////////////////////////////////////////////////////
        

    // *** INSERT YOUR CODE HERE ***
    m_deviceAvailable = true; // this value should become 'true' when the device is available.
    start_of_app = std::chrono::steady_clock::now();
}


//==============================================================================
/*!
    Destructor of cWoodenDevice.
*/
//==============================================================================
cWoodenDevice::~cWoodenDevice()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
}


//==============================================================================
/*!
    Open connection to your device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 3:

        Here you shall implement to open a connection to your
        device. This may include opening a connection to an interface board
        for instance or a USB port.

        If the connection succeeds, set the variable 'result' to true.
        otherwise, set the variable 'result' to false.

        Verify that your device is calibrated. If your device 
        needs calibration then call method calibrate() for wich you will 
        provide code in STEP 5 further bellow.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_ERROR; // this value will need to become "C_SUCCESS" for the device to be marked as ready.
    result = true; // TODO: Verify

    // *** INSERT YOUR CODE HERE ***
    fs = new FsHapticDeviceThread(false);



    // update device status
    if (result)
    {
        m_deviceReady = true;
        return (C_SUCCESS);
    }
    else
    {
        m_deviceReady = false;
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    Close connection to your device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 4:

        Here you shall implement code that closes the connection to your
        device.

        If the operation fails, simply set the variable 'result' to C_ERROR   .
        If the connection succeeds, set the variable 'result' to C_SUCCESS.
    */
    ////////////////////////////////////////////////////////////////////////////


    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

    // *** INSERT YOUR CODE HERE ***

    std::cout << "\nClosing\n";


    // update status
    m_deviceReady = false;

    return (result);
}

//==============================================================================
/*!
    Calibrate your device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::calibrate(bool a_forceCalibration)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 5:
        
        Here you shall implement code that handles a calibration procedure of the 
        device. In practice this may include initializing the registers of the
        encoder counters for instance. 

        If the device is already calibrated and  a_forceCalibration == false,
        the method may immediately return without further action.
        If a_forceCalibration == true, then the calibrartion procedure
        shall be executed even if the device has already been calibrated.
 
        If the calibration procedure succeeds, the method returns C_SUCCESS,
        otherwise return C_ERROR.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    // error = calibrateMyDevice()

    return (result);
}


//==============================================================================
/*!
    Returns the number of devices available from this class of device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
unsigned int cWoodenDevice::getNumDevices()
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 6:

        Here you shall implement code that returns the number of available
        haptic devices of type "cWoodenDevice" which are currently connected
        to your computer.

        In practice you will often have either 0 or 1 device. In which case
        the code here below is already implemented for you.

        If you have support more than 1 devices connected at the same time,
        then simply modify the code accordingly so that "numberOfDevices" takes
        the correct value.
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***

    int numberOfDevices = 1;  // At least set to 1 if a device is available.

    // numberOfDevices = getNumberOfDevicesConnectedToTheComputer();

    return (numberOfDevices);
}


//==============================================================================
/*!
    Read the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::getPosition(cVector3d& a_position)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 7:

        Here you shall implement code that reads the position (X,Y,Z) fromincoming_msg
        your haptic device. Read the values from your device and modify
        the local variable (x,y,z) accordingly.
        If the operation fails return an C_ERROR, C_SUCCESS otherwise

        Note:
        For consistency, units must be in meters.
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky. 
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    fsVec3d p = fs->getPos();
    a_position = cVector3d(p.x(),p.y(),p.z());
    //a_position = a_position * 0;


    // estimate linear velocity
    estimateLinearVelocity(a_position);

    // exit
    return (result);
}


//==============================================================================
/*!
    Read the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::getRotation(cMatrix3d& a_rotation)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 8:

        Here you shall implement code which reads the orientation frame from
        your haptic device. The orientation frame is expressed by a 3x3
        rotation matrix. The 1st column of this matrix corresponds to the
        x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
        The length of each column vector should be of length 1 and vectors need
        to be orthogonal to each other.

        Note:
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        If your device has a stylus, make sure that you set the reference frame
        so that the x-axis corresponds to the axis of the stylus.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // variables that describe the rotation matrix
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    cMatrix3d frame;
    frame.identity();

    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***









#ifdef SERIAL
    char buffer[320];
    int n = read(fd, buffer, sizeof(buffer));
    if (n < 0)
       fputs("read failed!\n", stderr);
    //std::cout << std::string(buffer) << std::endl;

    using namespace std;
    string s = string(buffer);
    //s.split



    //string s = "0.41,0.13,0.05,-0.90\n";
    /*
    istringstream sn (s);
    string oneline;
    while(!sn.eof()){

        getline( sn, oneline, '\n' );  // try to read the next field into it
    }
    */
    std::size_t last_end = s.rfind(']');
    std::size_t last_start = s.rfind('[');
    cout << s << endl;
    cout << last_end << "  " << last_start << endl;
    if(last_start >= 0 && last_end >= 0 && last_end > last_start){
        string oneline = s.substr(last_start+1,last_end-last_start-1);

        cout << "oneline: "  << oneline << endl;
        istringstream ss( oneline );
        double d[4]; int i=0;
        while (!ss.eof())         // See the WARNING above for WHY we're doing this!
        {
          string x;               // here's a nice, empty string
          getline( ss, x, ',' );  // try to read the next field into it
          cout << x << endl;      // print it out, EVEN IF WE ALREADY HIT EOF
          d[i++] = atof(x.c_str());
          //cout << "double " << d << endl;
          if(i>4) cout << "Much problem!" << endl;
        }
        if(i==4){
            cQuaternion q(d[0],d[1],d[2],d[3]);
            q.toRotMat(frame);
            cout << "set " << d[0] << "  " << d[1] << "  " << d[2] << "  " << d[3] <<  endl;
        } else
            cout << "warning: i<4" << endl;
    }

#endif







    // if the device does not provide any rotation capabilities 
    // set the rotation matrix equal to the identity matrix.
    /*
    r00 = 1.0;  r01 = 0.0;  r02 = 0.0;
    r10 = 0.0;  r11 = 1.0;  r12 = 0.0;
    r20 = 0.0;  r21 = 0.0;  r22 = 1.0;

    frame.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);
    */

    fsRot r = fs->getRot();
    frame.set(r.m[0][0], r.m[0][1], r.m[0][2],
              r.m[1][0], r.m[1][1], r.m[1][2],
              r.m[2][0], r.m[2][1], r.m[2][2]);

    // store new rotation matrix
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // exit
    return (result);
}


//==============================================================================
/*!
    Read the gripper angle in radian.

    \param   a_angle  Return value.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::getGripperAngleRad(double& a_angle)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 9:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as C_ERROR for instance.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***

    // return gripper angle in radian
    a_angle = 0.0;  // a_angle = getGripperAngleInRadianFromMyDevice();

    // estimate gripper velocity
    estimateGripperVelocity(a_angle);

    // exitelseifdef
    return (result);
}


//==============================================================================
/*!
    Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 10:
        
        Here you may implement code which sends a force (fx,elseifdeffy,fz),
        torque (tx, ty, tz) and/or gripper force (gf) command to your haptic device.

        If your device does not support one of more of the force, torque and 
        gripper force capabilities, you can simply ignore them. 

        Note:
        For consistency, units must be in Newtons and Newton-meters
        If your device is placed in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        For instance: if the force = (1,0,0), the device should move towards
        the operator, if the force = (0,0,1), the device should move upwards.
        A torque (1,0,0) would rotate the handle counter clock-wise around the 
        x-axis.
    */
    ////////////////////////////////////////////////////////////////////////////

    latest_force = a_force;
    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    fsVec3d f = fsVec3d(a_force.x(),a_force.y(), a_force.z());
    fs->setForce(f);

    // exit
    return (result);
}


//==============================================================================
/*!
    Read the status of the user switch [__true__ = \e ON / __false__ = \e OFF].

    \param   a_switchIndex  index number of the switch.
    \param   a_status result value from reading the selected input switch.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cWoodenDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 11:

        Here you shall implement code that reads the status of one or
        more user switches on your device. An application may request to read the status
        of a switch by passing its index number. The primary user switch mounted
        on the stylus of a haptic device will receive the index number 0. The
        second user switch is referred to as 1, and so on.

        The return value of a switch (a_status) shall be equal to "true" if the button
        is pressed or "false" otherwise.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    a_status = false;  // a_status = getUserSwitchOfMyDevice(a_switchIndex)

    return (result);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_WOODEN_DEVICE_SUPPORT
//------------------------------------------------------------------------------
