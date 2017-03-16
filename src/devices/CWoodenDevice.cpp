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
#include "hidapi/hidapi.h"
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>



#define USB  // define this to use the usb version
//#define DELAY /// To test delay
//#define PWM        // For PWM from DAQ, do not use with USB
//#define SAVE_LOG
//#define ALUHAPTICS

#ifndef USB
#define SENSORAY
#include "../../external/s826/include/826api.h"
#endif

// For delay testing
#include <chrono>
#include <thread>
#include <ratio>

//------------------------------------------------------------------------------
#define C_ENABLE_WOODEN_DEVICE_SUPPORT
#if defined(C_ENABLE_WOODEN_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

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
    double data[] = { 0.010, 0.010, 0.010, 
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
    m_specifications.m_sensedRotation                = false;

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

#ifndef USB
    //t = std::thread(&cWoodenDevice::set_dir,this);
    int boardflags  = S826_SystemOpen();

    std::cout << "S826 boardflags: " << boardflags << std::endl;

    S826_SafeWrenWrite(0,2);

    // Initialize counters for channel 3,4,5
    for(int i=0;i<3;++i){
        S826_CounterFilterWrite(0,i+3,0);
        S826_CounterModeWrite(0,i+3,0x70);
        S826_CounterPreloadWrite(0,i+3,0,0); // Load 0
        S826_CounterPreload(0,i+3,0,0);
        S826_CounterStateWrite(0,i+3,1);

#ifdef PWM

        // THIS ROUTES pwm FROM COUNTER TO DIO 0,1,2
        uint data2[2]= {7,0}; // DIO 0,1,2
        S826_DioOutputSourceWrite(0,data2);

/*
        S826_CounterPreloadWrite(0,i,0,500);//period*pwm_percent);     // On time in us
        S826_CounterPreloadWrite(0,i,1,500);//period*(1-pwm_percent)); // Off time in us
        S826_CounterModeWrite(0,i,0x01682020); //+131072 (invert)
        S826_CounterStateWrite(0,i,1);
        S826_CounterPreloadWrite(0,i,0,500);//period*pwm_percent);     // On time in us
        S826_CounterPreloadWrite(0,i,1,500);//period*(1-pwm_percent)); // Off time in us
        S826_CounterModeWrite(0,i,0x01682020); //+131072 (invert)
        S826_CounterStateWrite(0,i,1);

        */


        using namespace std;
        cout << "\nCounter Preload (register 0): " << S826_CounterPreloadWrite(0,i,0,50000); // On time in us (.1ms)
        cout << "\nCounter Preload (register 1): " << S826_CounterPreloadWrite(0,i,1,50000); // Off time in us (.9ms)
        cout << "\nCounter Mode: " << S826_CounterModeWrite(0,i,0x01682020+16); //+16 for 50Mhz clock instead of 1Mhz
        S826_CounterPreload(0,i,0,0);
        cout << "\nCounter State: " << S826_CounterStateWrite(0,i,1);

#else
        // And range of analoge signal to escon
        S826_DacRangeWrite(0,i,3,0); //-10 to 10 V

        // Enable ESCON driver by a digial signal, which in our case
        // is an analogue signal due to the fact that we only have the
        // analgoue and encoder breakout board of the S826.
        //
        S826_DacDataWrite(0,4+i,0xFFFF,0); // Channel 4,5,6 = Pin 41,43,45
#endif

#ifdef PWM
    uint data[2];
    data[0] = 8; // DIO3 (bit 4 set = 8)
    data[1] = 0;
    S826_DioOutputWrite(0,data,1); // Clear DIO3 = pin 41 is set HIGH
#endif

    }

    std::cout << "Opened SENSORAY Board" << std::endl;
#endif

    // USB HID ---------------------------------------------
#ifdef USB
    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs;
    while (cur_dev) {
        printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
        printf("\n");
        printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
        printf("  Product:      %ls\n", cur_dev->product_string);
        printf("  Release:      %hx\n", cur_dev->release_number);
        printf("  Interface:    %d\n",  cur_dev->interface_number);
        printf("\n");
        cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);

    // Open the device using the VID, PID,
    // and optionally the Serial number.
    ////handle = hid_open(0x4d8, 0x3f, L"12345");
    handle = hid_open(0x1234, 0x6, NULL);
    if (!handle) {
        printf("unable to open device\n");
        return 1;
    }
    // Set the hid_read() function to be non-blocking.
    hid_set_nonblocking(handle, 1);

    std::cout << "Opened USB Connection" << std::endl;
    // ------------------------------------------------------
#endif





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

    // Save log
#ifdef SAVE_LOG
    using namespace std;

    ofstream myfile;
    myfile.open ("log.m");
    int lines = timestamp.size() < forces.size() ? timestamp.size() : forces.size();
    lines = positions.size() < lines ? positions.size() : lines;
    string channel[] = {"force_x=[","force_y=[","force_z=[","timestamp=[","pos_x=[","pos_y=[","pos_z=["};
    for(int c=0;c<7;++c){
        myfile << channel[c];
        for(int i=0;i<lines;++i){
            if(c==0) myfile << forces[i].x();
            if(c==1) myfile << forces[i].y();
            if(c==2) myfile << forces[i].z();
            if(c==3) myfile << timestamp[i]*0.000001;
            if(c==4) myfile << positions[i].x()*1000;
            if(c==5) myfile << positions[i].y()*1000;
            if(c==6) myfile << positions[i].z()*1000;
            myfile << " ";
        }
        myfile << "];\n";
    }
    myfile.close();
#endif



    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

    // *** INSERT YOUR CODE HERE ***

    std::cout << "\nClosing\n";
    // Disable power
#ifdef PWM
    uint data[2];
    data[0] = 8; // Bit 3 set = 4
    data[1] = 0;
    S826_DioOutputWrite(0,data,2); // SET DIO3 = pin 41 is set LOW

    S826_CounterStateWrite(0,0,0);
    S826_CounterStateWrite(0,1,0);
    S826_CounterStateWrite(0,2,0);
#else
#ifndef USB
    S826_DacDataWrite(0,4,0x0,0); // Channel 4 = Pin 41
    S826_DacDataWrite(0,5,0x0,0); // Channel 5 = Pin 43
    S826_DacDataWrite(0,6,0x0,0); // Channel 6 = Pin 45
#endif
#endif


    //close HID device
    hid_close(handle);
    hid_exit();

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





const double pi = 3.14159265359;

//==============================================================================
// Helper functions for getPosition & setForce
//==============================================================================
#ifdef SENSORAY
double getMotorAngle(int motor, double cpr) {    
    const unsigned int maxdata = 0xFFFFFFFF; // 32 bit

    uint encoderValue;
    S826_CounterSnapshot(0,motor+3);
    uint ctstamp;
    uint reason;
    S826_CounterSnapshotRead(0,motor+3,&encoderValue,&ctstamp,&reason,0);


    //uint encoderValue2;
    //S826_CounterSnapshot(0,0);
    //S826_CounterSnapshotRead(0,0,&encoderValue2,&ctstamp,&reason,0);
    //std::cout << encoderValue2 << std::endl;

    if(encoderValue >= maxdata/2)
        return -1.0*2.0*pi*(maxdata-encoderValue)/cpr;
    return 2.0*pi*encoderValue/cpr;



}

void setVolt(double v, int motor){
    if(v > 10 || v< -10) { printf("Volt outside +/- 10 Volt\n"); return; }

//    if(motor == 0){
//        std::cout << "Volt " << v << std::endl;
//    }

    //if(v > 6.6 || v< -6.6) { printf("Volt outside +/- 6.6 Volt = 2 Ampere (soft cap)\n"); return; }
    // -10V to +10V is mapped from 0x0000 to 0xFFFF
    unsigned int signal = (v+10.0)/20.0 * 0xFFFF;
    S826_DacDataWrite(0,motor,signal,0);
}
#endif

struct pose {
    double Ln;
    double Lb;
    double Lc;
    double tA;  // angle of body A (theta_A)
    double tB;  // angle of body B (theta_B)
    double tC;  // angle of body C (theta_C)
};

pose calculate_pose(const cWoodenDevice::configuration& c, double* encoder_values) {
    pose p;

    double cpr[] = { c.cpr_encoder_a, c.cpr_encoder_b, c.cpr_encoder_c };
    double gearRatio[] = { c.diameter_body_a / c.diameter_capstan_a,
                   -c.diameter_body_b / c.diameter_capstan_b,
                    c.diameter_body_c / c.diameter_capstan_c };

    double dofAngle[3];
#ifdef USB
    for(int i=0;i<3;i++)
        dofAngle[i] = (2.0*pi*encoder_values[i]/cpr[i]) / gearRatio[i];
#else
    for(int i=0;i<3;i++)
        dofAngle[i] = -getMotorAngle(i,cpr[i]) / gearRatio[i];
#endif

#ifdef ALUHAPTICS
    dofAngle[0] = -dofAngle[0];
    dofAngle[1] = dofAngle[1];
    dofAngle[2] = dofAngle[2];
#endif

    // Calculate dof angles (theta) for each body
    p.Ln = c.length_body_a; 
    p.Lb = c.length_body_b; 
    p.Lc = c.length_body_c; 
    p.tA = dofAngle[0];
    p.tB = dofAngle[1];
    //p.tC = dofAngle[2] - dofAngle[1];
    p.tC = dofAngle[2]; // 2016-05-30

    return p;
}

double deg(double rad){
    return 360*rad/(2*3.141592);
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
    double x,y,z;

    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***
#ifndef USB
    const pose p = calculate_pose(m_config,0);
    const double& Ln = p.Ln;
    const double& Lb = p.Lb; 
    const double& Lc = p.Lc; 
    const double& tA = p.tA; 
    double tB = p.tB;
    double tC = p.tC;

    // Do forward kinematics (thetas -> xyz)
    x = cos(tA)*(Lb*sin(tB)+Lc*cos(tB+tC))     - m_config.workspace_origin_x;
    y = sin(tA)*(Lb*sin(tB)+Lc*cos(tB+tC))     - m_config.workspace_origin_y;
    z = Ln + Lb*cos(tB) - Lc*sin(tB+tC)        - m_config.workspace_origin_z;


    // Mike kinematics 2016-05-30
    //std::cout << "Raw tA, tB, tC: " << deg(tA) << " " << deg(tB) << " " << deg(tC);

    // we have to remove the false contribution from tB
    //tC = tC+tB;

    // according to figure is our usual "starting position" at 90' theta c
    //tC = -tC+3.141592/2;
    //tC = tC+3.141592/2;
    tB = tB + 3.141592/2;

    //std::cout << " Modified tA, tB, tC: "<< deg(tA) << " " << deg(tB) << " " << deg(tC) << std::endl;

    x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC))    - m_config.workspace_origin_x;
    y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
    z = Ln+Lb*cos(tB)-Lc*cos(tC) - m_config.workspace_origin_z;

#endif

#ifdef USB

    /*
    res = hid_read(handle, buf, sizeof(buf));
    if(res) {
        woodenhaptics_message msg_in = *reinterpret_cast<woodenhaptics_message*>(buf);

        x = msg_in.position_x;
        y = msg_in.position_y;
        z = msg_in.position_z;

        // store new position values
        a_position.set(x, y, z);

        // estimate linear velocity
        estimateLinearVelocity(a_position);
    }
    */

/*
    res = 0;
    while (res == 0) {
        res = hid_read(handle, buf, sizeof(buf));

        usleep(100);
    }


    // Got one, but is it the latest?
    int flush=0;
    while(int res2 = hid_read(handle, buf, sizeof(buf))){
        ++flush;
    }
    //std::cout << "Flushed " << flush << " messages." << std::endl;

    if(flush || res){
        woodenhaptics_message msg_in = *reinterpret_cast<woodenhaptics_message*>(buf);

        x = msg_in.position_x;
        y = msg_in.position_y;
        z = msg_in.position_z;
    };
    */


    int res=0;
    while (res == 0) {
        res = hid_read(handle, buf, sizeof(buf));
        if(res==8) // Got a correct message
            //incoming_msg = *reinterpret_cast<woodenhaptics_message*>(buf);
            hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(buf);
        usleep(10);
    }

    int flush=0;
    while(int res2 = hid_read(handle, buf, sizeof(buf))){
        if(res==8) // Got a correct message
            //incoming_msg = *reinterpret_cast<woodenhaptics_message*>(buf);
            hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(buf);
        ++flush;
    }
    //if(flush)
    //    std::cout << "Flushed " << flush << " messages." << std::endl;
    lost_messages += flush;


    /*
    while(int res = hid_read(handle, buf, sizeof(buf))){
        if(res==48) // Got a correct message
            incoming_msg = *reinterpret_cast<woodenhaptics_message*>(buf);
    }
    */


//    double encoder_values[] = { incoming_msg.temperature_0,
//                                incoming_msg.temperature_1,
//                                incoming_msg.temperature_2 };
    //double encoder_values[] = { 0,                             0, 0 };
    double encoder_values[] = { -hid_to_pc.encoder_a,
                                hid_to_pc.encoder_b,
                                hid_to_pc.encoder_c };

    //std::cout << "a: " << hid_to_pc.encoder_a << " b:" << hid_to_pc.encoder_b << " c:"<< hid_to_pc.encoder_c << "\n";

    using namespace std;
    //cout << encoder_values[0] << ", " << encoder_values[1] << ", " << encoder_values[2] << endl;
    pose p  = calculate_pose(m_config, encoder_values);
    const double& Ln = p.Ln;
    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    double tB = p.tB;
    double tC = p.tC;

    // Do forward kinematics (thetas -> xyz)
    /*
    x = cos(tA)*(Lb*sin(tB)+Lc*cos(tB+tC))     - m_config.workspace_origin_x;
    y = sin(tA)*(Lb*sin(tB)+Lc*cos(tB+tC))     - m_config.workspace_origin_y;
    z = Ln + Lb*cos(tB) - Lc*sin(tB+tC)        - m_config.workspace_origin_z;
    */


    // Mike edition
#ifdef ALUHAPTICS
    tB = tB + 3.141592/2;
#else
    tC = -tC + 3.141592/2;
#endif
    x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC))    - m_config.workspace_origin_x;
    y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
    z = Ln+Lb*cos(tB)-Lc*cos(tC) - m_config.workspace_origin_z;




/*
    std::cout << " xyz "<< x << " " << y << " " << z << std::endl;

    std::cout << " Modified tA, tB, tC: "<< deg(tA) << " " << deg(tB) << " " << deg(tC) << std::endl;

    std::cout << "cos(tA) " << cos(tA) << " Lb: " << Lb << " sin(tB): " << sin(tB)
              << " Lc:" << Lc << " sin(tC): " << sin(tC) << " origin_x: " << m_config.workspace_origin_x << "\n";
*/

    /*



    x = incoming_msg.position_x;
    y = incoming_msg.position_y;
    z = incoming_msg.position_z;
*/







#endif


#ifdef DELAY
    using namespace std::chrono;
    duration<int, std::micro> d{400};
    std::this_thread::sleep_for(d);
#endif

    // store new position values
    a_position.set(x, y, z);

#ifdef SAVE_LOG
    positions.push_back(a_position);
#endif

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

    // if the device does not provide any rotation capabilities 
    // set the rotation matrix equal to the identity matrix.
    r00 = 1.0;  r01 = 0.0;  r02 = 0.0;
    r10 = 0.0;  r11 = 1.0;  r12 = 0.0;
    r20 = 0.0;  r21 = 0.0;  r22 = 1.0;

    frame.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);

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

#ifdef SENSORAY
void cWoodenDevice::set_dir(){
    double period = 1000*50; //us (*50 for 50Mhz clock)
    using namespace std::chrono;
    duration<int, std::micro> d{1};

    int my_dir_sum = 0;


    double next_percentages[3] = {0.1,0.1,0.1};
    uint next_dir_sum = 0;
    uint last_val;

    while(true){
        /*
        for(int i=0;i<3;++i){
            S826_CounterPreloadWrite(0,i,0,period*(1-global_pwm_percent[i]));     // On time in us
            S826_CounterPreloadWrite(0,i,1,period*global_pwm_percent[i]);         // Off time in us
        }
        */


        uint ctstamp;
        uint reason;
        uint counter;
        S826_CounterSnapshot(0,0);
        S826_CounterSnapshotRead(0,0,&counter,&ctstamp,&reason,0);

        if(last_val<counter){
            // means we have reloaded 1000
            for(int i=0;i<3;++i){
                next_percentages[i] = global_pwm_percent[i];
                next_dir_sum = global_dir_sum;
            }
        }
        last_val = counter;


        uint pins[] = {1,2,4};
        uint dio = 0; // (8 = dio4 = enable, but set it high = low = off)
        //std::cout << counter << " ";
        for(int i=0;i<3;++i){
            if(counter<(1-next_percentages[i])*period)
                dio += pins[i];
        }
        dio += next_dir_sum;


        //if(global_dir_sum==my_dir_sum)
        //    std::this_thread::sleep_for(d);

        //my_dir_sum = global_dir_sum;


        // All direction pins at once
        uint data[2];
        data[0] = dio;//global_dir_sum;
        data[1] = 0;
        S826_DioOutputWrite(0,data,0);

        // Sleep one micro
        duration<int, std::micro> dt{1};
        std::this_thread::sleep_for(dt);


        //std::this_thread::sleep_for(d);
    }
}
#endif


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




    //if(a_force.length()>0.0001){

#ifdef SAVE_LOG
        forces.push_back(a_force);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start_of_app).count();
        timestamp.push_back(duration_us);
#endif
    //}



    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    double gf = a_gripperForce;

    // *** INSERT YOUR CODE HERE ***
#ifdef USB
    double encoder_values[] = { -hid_to_pc.encoder_a,
                                hid_to_pc.encoder_b,
                                hid_to_pc.encoder_c };
    const pose p = calculate_pose(m_config, encoder_values);
#else
    const pose p = calculate_pose(m_config,0);
#endif
    const double& Ln = p.Ln;
    const double& Lb = p.Lb; 
    const double& Lc = p.Lc; 
    const double& tA = p.tA; 
    double tB = p.tB;
    double tC = p.tC;



    // Starting position 90'
    //tC = tC+3.141592/2;
    //tB = tB + 3.141592/2;

    //std::cout << " Modified tA, tB, tC: "<< deg(tA) << " " << deg(tB) << " " << deg(tC) << std::endl;
    //x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC))    - m_config.workspace_origin_x;
    //y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
    //z = Ln+Lb*cos(tB)-Lc*cos(tC) - m_config.workspace_origin_z;

     // Mike edition
#ifdef ALUHAPTICS
    tB = tB + 3.141592/2;
#else
    tC = -tC + 3.141592/2;
#endif




    /*
    // Make Jacobian 2015-style
    double jac123[4][4];
    jac123[1][1] = -sin(tA)*(Lb*sin(tB)+Lc*cos(tB+tC));
    jac123[1][2] =  cos(tA)*(Lb*cos(tB)); //-Lc*sin(tB+tC));
    jac123[1][3] = -Lc*cos(tA)*sin(tB+tC);
    jac123[2][1] = cos(tA)*(Lb*sin(tB)+Lc*cos(tB+tC));
    jac123[2][2] = sin(tA)*(Lb*cos(tB)); //-Lc*sin(tB+tC));
    jac123[2][3] = -Lc*sin(tA)*sin(tB+tC);
    jac123[3][1] = 0;
    jac123[3][2] = -Lb*sin(tB)-Lc*cos(tB+tC);
    jac123[3][3] = -Lc*cos(tB+tC);

    // Calculate torque = J transpose F
    cMatrix3d J;
    J.set(jac123[1][1],jac123[1][2],jac123[1][3],
          jac123[2][1],jac123[2][2],jac123[2][3],
          jac123[3][1],jac123[3][2],jac123[3][3]);
    */

    // Make Jacobian 2016-05-30
    cMatrix3d J;
    J.set(  -sin(tA)*(Lb*sin(tB)+Lc*sin(tC)),    Lb*cos(tA)*cos(tB),   Lc*cos(tA)*cos(tC),
             cos(tA)*(Lb*sin(tB)+Lc*sin(tC)),    Lb*sin(tA)*cos(tB),   Lc*sin(tA)*cos(tC),
                        0,                           -Lb*sin(tB),           Lc*sin(tC)     );


    cVector3d f=a_force;
    //f += cVector3d(-0.2,0,0);
    cVector3d t=cTranspose(J)*f;

    // Gravity compensation
    const double& g=m_config.g_constant;
    const double& Lb_cm = m_config.length_cm_body_b;
    const double& Lc_cm = m_config.length_cm_body_c;
    const double& mB = m_config.mass_body_b;
    const double& mC = m_config.mass_body_c;
    //double grav_comp[] { 0,
    //                    -g*(Lb*mC*sin(tB)+Lb_cm*mB*sin(tB)+Lc_cm*mC*cos(tB+tC)),
    //                    -g*(Lc_cm*mC*cos(tB+tC)) };
    //t = t + cVector3d(grav_comp[0],grav_comp[1],grav_comp[2]);


    t = t + -g*cVector3d( 0,
                          mB*Lb_cm*sin(tB) + mC*(Lb_cm + Lc_cm)*sin(tC),
                          mC*Lc_cm*sin(tC) );




    // Gear down 
    double motorTorque[] = {
            t.x() * m_config.diameter_capstan_a / m_config.diameter_body_a,
            -t.y() * m_config.diameter_capstan_b / m_config.diameter_body_b,
            -t.z() * m_config.diameter_capstan_c / m_config.diameter_body_c }; // switched sign 2016-05-30


#ifdef ALUHAPTICS
    motorTorque[0] = motorTorque[0];
    motorTorque[1] = motorTorque[1];
    motorTorque[2] = -motorTorque[2];
#endif


    // Set motor torque (t)
    double torque_constant[] = { m_config.torque_constant_motor_a, 
                                 m_config.torque_constant_motor_b,
                                 m_config.torque_constant_motor_c };

    short signalToSend[3] = {0,0,0};
    int dir[3];
    int dir_chan[3] = {16,32,64}; // DIO4, DIO5, DIO6
    int dir_sum=0;

    for(int i=0;i<3;++i){
        double motorAmpere = motorTorque[i] / torque_constant[i];
        double signal = motorAmpere * 10.0 / m_config.current_for_10_v_signal; 
        if(signal>10.0)  signal =  10.0;
        if(signal<-10.0) signal = -10.0;

        dir[i] = cSign(signal) > 0 ? 1 : 2;
        dir_sum += cSign(signal) < 0 ? dir_chan[i] : 0;

        //if(i==0) motorAmpere = 0.5;


#ifdef USB
        if(motorAmpere>3) motorAmpere = 3;
        if(motorAmpere<-3) motorAmpere = -3;
        // 90 % = 3 A set in microcontroller. Escon configured as 90% = 1A.
       signalToSend[i] = short(motorAmpere*1000);
#endif

#ifdef PWM
        // "-10V to +10V" is mapped to -3 to 3 Amp
        // 0 Amp = 10% PWM, 3 Amp (or "10V") is 90%
        // Direction is set separately.
        double pwm_percent = (cSign(signal)*signal/10.0)*0.8 + 0.1; // (|v| / vMax) * 80% + 10%
        double period = 200*50; //us (*50 for 50Mhz clock)

        //std::cout << "Motor " << motor << " PWM percent " << pwm_percent << std::endl;
        S826_CounterPreloadWrite(0,i,0,period*(1-pwm_percent));     // On time in us
        S826_CounterPreloadWrite(0,i,1,period*pwm_percent);         // Off time in us

        global_pwm_percent[i] = pwm_percent;
#endif

#ifndef USB
        // One at a time
        uint direction = cSign(v) > 0 ? 1 : 2;

        uint data[2];
        data[0] = dir_chan[i];
        data[1] = 0;
        S826_DioOutputWrite(0,data,direction);

        //global_dir_sum = dir_sum;

        //using namespace std::chrono;
        //duration<int, std::micro> d{500};
        //std::this_thread::sleep_for(d);

        setVolt(signal,i);
        //setVolt(2.0,i);
#endif
    }



#ifdef USB    
    //signalToSend[0] = 0;
    //signalToSend[1] = 0;
    //signalToSend[2] = 0;

    unsigned char out_buf[9];

    //woodenhaptics_message msg;
    //msg.command_force_x = signalToSend[0];
    //msg.command_force_y = signalToSend[1];
    //msg.command_force_z = signalToSend[2];
    pc_to_hid.current_motor_a_mA = signalToSend[0];
    pc_to_hid.current_motor_b_mA = signalToSend[1];
    pc_to_hid.current_motor_c_mA = signalToSend[2];

    torqueSignals = cVector3d(signalToSend[0], signalToSend[1], signalToSend[2]);

    unsigned char* msg_buf = reinterpret_cast<unsigned char*>(&pc_to_hid);

    //Fill the report
    out_buf[0] = 0;
    for (int i = 1; i < 9; i++) {
        out_buf[i] = msg_buf[i-1];
    }
    int error = hid_write(handle,out_buf,sizeof(out_buf));
    if(error!=9){
        std::cout << "hid_write return " << error << std::endl;
    }

    //usleep(20);


    //std::cout << "Send Force! " << msg.command_force_x << std::endl;


#endif

    // setTorqueToMyDevice(tx, ty, tz);
    // setForceToGripper(fg);

#ifdef DELAY
    using namespace std::chrono;
    duration<int, std::micro> d{400};
    std::this_thread::sleep_for(d);
#endif

    /*
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    if(duration_us < 0 || duration_us>1000000){}
    else if(duration_us<900){
        using namespace std::chrono;
        int sleep_for = 900-duration_us;
        //std::cout << "duration " << duration_us << " sleep for" << sleep_for << std::endl;
        duration<int, std::micro> d{sleep_for};
        std::this_thread::sleep_for(d);
    }
    start = std::chrono::steady_clock::now();
    */

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
