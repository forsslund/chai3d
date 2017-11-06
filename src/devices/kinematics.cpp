#ifdef USE_HAPTICS // For H3D
#include <HAPI/kinematics.h>
#else
#include "kinematics.h"
#endif
#include <string>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

//#define UNIX
#ifdef UNIX
namespace unix {
    // Following includes are only used for reading/writing config file and to find
    // the user's home directory (where the config file will be stored)
    #include <unistd.h>
    #include <sys/types.h>
    #include <pwd.h>
}
#endif



void fsRot::identity() {
    double a[3][3] = { {1, 0, 0 },
                       {0, 1, 0 },
                       {0, 0, 1 }};
    set(a);
}
void fsRot::rot_x(double t){
    double a[3][3] = { {1,   0,       0    },
                       {0, cos(t), -sin(t) },
                       {0, sin(t), cos(t)  }};
    set(a);
}
void fsRot::rot_y(double t){
    double a[3][3] = { {cos(t),  0, sin(t) },
                       {   0,    1,   0    },
                       {-sin(t), 0, cos(t) }};
    set(a);
}
void fsRot::rot_z(double t){
    double a[3][3] = { {cos(t), -sin(t), 0 },
                       {sin(t), cos(t), 0 },
                       {0, 0, 1 }};
    set(a);
}


const double pi = 3.14159265359;

//==============================================================================
// Helper functions for getPosition & setForce
//==============================================================================
struct pose {
    double Ln;
    double Lb;
    double Lc;
    double tA;  // angle of body A (theta_A)
    double tB;  // angle of body B (theta_B)
    double tC;  // angle of body C (theta_C)
};

pose calculate_pose(const Kinematics::configuration& c, int* encoder_values) {
    pose p;

    double cpr[] = { c.cpr_encoder_a, c.cpr_encoder_b, c.cpr_encoder_c };
    double gearRatio[] = {-c.diameter_body_a / c.diameter_capstan_a,
                          -c.diameter_body_b / c.diameter_capstan_b,
                           c.diameter_body_c / c.diameter_capstan_c };

    double dofAngle[3];
    for(int i=0;i<3;i++)
        dofAngle[i] = (2.0*pi*double(encoder_values[i])/cpr[i]) / gearRatio[i];

    if(int(c.variant) == 1){ // ALUHAPTICS
        dofAngle[0] = -dofAngle[0];
        dofAngle[1] =  dofAngle[1];
        dofAngle[2] =  dofAngle[2];
    }

    // Calculate dof angles (theta) for each body
    p.Ln = c.length_body_a;
    p.Lb = c.length_body_b;
    p.Lc = c.length_body_c;
    p.tA = dofAngle[0];
    p.tB = dofAngle[1];
    p.tC = dofAngle[2];

    return p;
}

//==============================================================================
// WoodenHaptics configuration helper files.
//==============================================================================

Kinematics::configuration default_woody(){
    double data[] = { 0, 0.010, 0.010, 0.010,
                      0.080, 0.205, 0.245,
                      0.160, 0.120, 0.120,
                      0.220, 0.000, 0.080, 0.100,
                      0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                      5.0, 1000.0, 8.0,
                      0.170, 0.110, 0.051, 0.091, 0};
    return Kinematics::configuration(data);
}

double v(const std::string& json, const std::string& key){
    int p = json.find(":", json.find(key));
    return atof(json.substr(p+1).c_str());
}

Kinematics::configuration fromJSON(std::string json){
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
    return Kinematics::configuration(d);
}

std::string j(const std::string& key, const double& value){
   std::stringstream s;
   s << "    \"" << key << "\":";
   while(s.str().length()<32) s<< " ";
   s << value << "," << std::endl;
   return s.str();
}
std::string toJSON(const Kinematics::configuration& c){
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

void write_config_file(const Kinematics::configuration& config){
#ifdef UNIX
    using namespace unix;
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = unix::getpwuid(getuid())->pw_dir;
    }

    std::cout << "Writing configuration to: "<< homedir
              << "/woodenhaptics.json" << std::endl;
    std::ofstream ofile;
    ofile.open(std::string(homedir) + "/woodenhaptics.json");
    ofile << toJSON(config);
    ofile.close();
#endif
}

Kinematics::configuration read_config_file(){
#ifdef UNIX
    using namespace unix;
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = unix::getpwuid(getuid())->pw_dir;
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
#else
    return default_woody();
#endif
}
//==============================================================================



Kinematics::Kinematics():m_config(read_config_file())
{

}

fsVec3d Kinematics::computePosition(int *encoderValues)
{
    if(m_config.variant == 2){ // Ramtin device
        encoderValues[0] = -encoderValues[0];
        encoderValues[1] = -encoderValues[1];
        encoderValues[2] = -encoderValues[2];
    }
    pose p  = calculate_pose(m_config, encoderValues);


    const double& Ln = p.Ln;
    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    double tB = p.tB;
    double tC = p.tC;

    if(int(m_config.variant) == 1) // ALUHAPTICS
        tB = tB + 3.141592/2;
    else
        tC = -tC + 3.141592/2;

    double x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_x;
    double y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
    double z = Ln+Lb*cos(tB)-Lc*cos(tC)        - m_config.workspace_origin_z;

    return fsVec3d(x,y,z);
}

fsVec3d Kinematics::computeMotorAmps(fsVec3d force, int *encoderValues)
{
    if(m_config.variant == 2){ // Ramtin device
        encoderValues[0] = -encoderValues[0];
        encoderValues[1] = -encoderValues[1];
        encoderValues[2] = -encoderValues[2];
    }

    const pose p = calculate_pose(m_config, encoderValues);

    const double& Ln = p.Ln;
    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    double tB = p.tB;
    double tC = p.tC;

    if(int(m_config.variant) == 1) // ALUHAPTICS
        tB = tB + 3.141592/2;
    else
        tC = -tC + 3.141592/2;


    // Make Jacobian 2016-05-30
    /*
    cMatrix3d J;
    J.set(  -sin(tA)*(Lb*sin(tB)+Lc*sin(tC)),    Lb*cos(tA)*cos(tB),   Lc*cos(tA)*cos(tC),
             cos(tA)*(Lb*sin(tB)+Lc*sin(tC)),    Lb*sin(tA)*cos(tB),   Lc*sin(tA)*cos(tC),
                        0,                           -Lb*sin(tB),           Lc*sin(tC)     );
    fsVec3d t1=cTranspose(J)*force;
    */


    double fx=force.x();
    double fy=force.y();
    double fz=force.z();
    double tx = -sin(tA)*(Lb*sin(tB)+Lc*sin(tC))*fx +  cos(tA)*(Lb*sin(tB)+Lc*sin(tC))*fy + 0*fz;
    double ty = Lb*cos(tA)*cos(tB)*fx + Lb*sin(tA)*cos(tB)*fy -Lb*sin(tB)*fz;
    double tz = Lc*cos(tA)*cos(tC)*fx + Lc*sin(tA)*cos(tC)*fy + Lc*sin(tC)*fz;
    fsVec3d t = fsVec3d(tx,ty,tz);

    // Gravity compensation
    const double& g=m_config.g_constant;
    const double& Lb_cm = m_config.length_cm_body_b;
    const double& Lc_cm = m_config.length_cm_body_c;
    const double& mB = m_config.mass_body_b;
    const double& mC = m_config.mass_body_c;

    t = t + -g*fsVec3d( 0,
                          mB*Lb_cm*sin(tB) + mC*(Lb_cm + Lc_cm)*sin(tC),
                          mC*Lc_cm*sin(tC) );

    // Gear down
    double motorTorque[] = {
            t.x() * m_config.diameter_capstan_a / m_config.diameter_body_a,
           -t.y() * m_config.diameter_capstan_b / m_config.diameter_body_b,
           -t.z() * m_config.diameter_capstan_c / m_config.diameter_body_c }; // switched sign 2016-05-30


    if(int(m_config.variant) == 1){ // ALUHAPTICS
        motorTorque[0] =  motorTorque[0];
        motorTorque[1] =  motorTorque[1];
        motorTorque[2] = -motorTorque[2];
    }
    if(int(m_config.variant) == 2){ // RAMTIN
        motorTorque[0] = -motorTorque[0];
        motorTorque[1] = -motorTorque[1];
        motorTorque[2] = -motorTorque[2];
    }


    // Set motor torque (t)
    double torque_constant[] = { m_config.torque_constant_motor_a,
                                 m_config.torque_constant_motor_b,
                                 m_config.torque_constant_motor_c };


    double motorAmpere[3];
    for(int i=0;i<3;++i){
        motorAmpere[i] = motorTorque[i] / torque_constant[i];
    }

    return fsVec3d(motorAmpere[0],motorAmpere[1],motorAmpere[2]);
}

fsRot Kinematics::computeRotation(int* encBase, int* encRot)
{
    if(m_config.variant == 2){ // Ramtin device
        encBase[0] = -encBase[0];
        encBase[1] = -encBase[1];
        encBase[2] = -encBase[2];
    }

    // From compute pos -------------------
    pose p  = calculate_pose(m_config, encBase);

    const double& Ln = p.Ln;
    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    double tB = p.tB;
    double tC = p.tC;

    if(int(m_config.variant) == 1) // ALUHAPTICS
        tB = tB + 3.141592/2;
    else
        tC = -tC + 3.141592/2;

    double x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_x;
    double y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
    double z = Ln+Lb*cos(tB)-Lc*cos(tC)        - m_config.workspace_origin_z;
    // -------------------------------------------

    fsRot r;
    r.identity();

#define VINTAGE
#ifdef VINTAGE
    double tD = -encRot[0]*2*pi/2000.0;
    double tE = -encRot[1]*2*pi/2000.0;
    double tF = -encRot[2]*2*pi/2000.0;

    // rotate about z (body a)
    fsRot rA;
    rA.rot_z(tA);

    // rotate about y (body b)
    fsRot rB;
    //rB.rot_y(tB-3.141592/2);

    // rotate about y (body c)
    fsRot rC;
    rC.rot_y(-tC);

    // rotate about x
    fsRot rD;
    rD.rot_z(tD);
    // rotate about y
    fsRot rE;
    rE.rot_y(tE);
    // rotate about x
    fsRot rF;
    rF.rot_x(tF);
    r =  rA*rB*rC*rD*rE*rF;
#endif

#ifdef RAMTIN
    double tD = encRot[0]*2*pi/4096.0;
    double tE = -encRot[1]*2*pi/4096.0;
    double tF = encRot[2]*2*pi/4096.0;

    // rotate about z (body a)
    fsRot rA;
    rA.rot_z(tA);

    // rotate about y (body b)
    fsRot rB;
    rB.rot_y(tB);

    // rotate about y (body c)
    fsRot rC;
    rC.rot_y(-tC+3.141592/2);

    // rotate about x
    fsRot rD;
    rD.rot_x(tD);
    // rotate about y
    fsRot rE;
    rE.rot_y(tE);
    // rotate about x
    fsRot rF;
    rF.rot_x(tF);
    r =  rA*rB*rC*rD*rE *rF;
#endif

    return r;
}




