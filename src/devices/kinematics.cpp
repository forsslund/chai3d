#define PELLE
#ifdef PELLE
#define SUPPORT_POLHEMV2
#include "hfabmath.h"

#include <iostream>

//#define USE_PRECOMPUTED_GRAVCOMP
#define USE_DIRECT_COMPUTE_GRAVITY_COMPENSATION

#ifdef USE_PRECOMPUTED_GRAVCOMP
#include "precomp_gravcomp.h"
#endif

//#include <string>
//#include <sstream>
#include <cmath>
//#include "polhem.h"

constexpr long double pi{3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647L};
constexpr double pi_d{3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647};


#ifdef SUPPORT_POLHEMV2
#ifdef UNIX
#include "polhem.h"
#else
#include "polhem.h"
#endif
#endif


#ifdef UNIX
//namespace unix {
    // Following includes are only used for reading/writing config file and to find
    // the user's home directory (where the config file will be stored)
    #include <unistd.h>
    #include <sys/types.h>
    #include <pwd.h>
//}
#endif


namespace haptikfabriken {

    /*
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

fsRot fsRot::transpose()
{
    double a[3][3] = { {m[0][0], m[1][0], m[2][0] },
                       {m[0][1], m[1][1], m[2][1] },
                       {m[0][2], m[1][2], m[2][2] }};
    fsRot r;
    r.set(a);
    return r;
}

fsVec3d operator*(const fsRot &m, const fsVec3d &v)
{
    fsVec3d r;

    r.m_x = m.m[0][0]*v.m_x + m.m[0][1]*v.m_y + m.m[0][2]*v.m_z;
    r.m_y = m.m[1][0]*v.m_x + m.m[1][1]*v.m_y + m.m[1][2]*v.m_z;
    r.m_z = m.m[2][0]*v.m_x + m.m[2][1]*v.m_y + m.m[2][2]*v.m_z;

    return r;
}
*/

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

pose calculate_pose(const Kinematics::configuration& c, const int* encoder_values) {
    pose p;

    double motorAngle[] = { (2.0*pi_d*double(encoder_values[0])/c.cpr_encoder_a),
                            (2.0*pi_d*double(encoder_values[1])/c.cpr_encoder_b),
                            (2.0*pi_d*double(encoder_values[2])/c.cpr_encoder_c)};

    //std::cout << encoder_values[0] << "  cpr: "  << c.cpr_encoder_a << "  gives motor angle: " << motorAngle[0] << "\n";


    // Calculate dof angles (theta) for each body
    p.Ln = c.length_body_a;
    p.Lb = c.length_body_b;
    p.Lc = c.length_body_c;

    // Different directions of motor on different devices
    double dir[] = {bool(c.motor_and_body_aligned_a)?-1.0:1.0,
                    bool(c.motor_and_body_aligned_b)?-1.0:1.0,
                    bool(c.motor_and_body_aligned_c)?-1.0:1.0};

    /*
    if(c.variant == 3 ) { // POLHEM v2
        p.tA =   motorAngle[0] * c.diameter_capstan_a / c.diameter_body_a;
        p.tB =  -motorAngle[1] * c.diameter_capstan_b / c.diameter_body_b;
        p.tC =   motorAngle[2] * c.diameter_capstan_c / c.diameter_body_c;
    } else {
        p.tA = -motorAngle[0] * c.diameter_capstan_a / c.diameter_body_a;
        p.tB = -motorAngle[1] * c.diameter_capstan_b / c.diameter_body_b;
        p.tC = -motorAngle[2] * c.diameter_capstan_c / c.diameter_body_c;
    }
    */
    p.tA = dir[0] * motorAngle[0] * c.diameter_capstan_a / c.diameter_body_a;
    p.tB = dir[1] * motorAngle[1] * c.diameter_capstan_b / c.diameter_body_b;
    p.tC = dir[2] * motorAngle[2] * c.diameter_capstan_c / c.diameter_body_c;
    //std::cout << "capstan: " << c.diameter_capstan_a << "  body: "  << c.diameter_body_a << "  gives ta: " << p.tA << "\n";

    return p;
}




// This data is not used if polhem_v3() is called..!
double datadata[] = { 3, 0.0288534, 0.0136981, 0.0137300,
                  0.1, 0.165, 0.1308,
                  0.175, 0.100, 0.100,
                  0.2045, 0.0, 0.0325, 0.150, // 0.137 0 0 0.150
                  0.0603, 0.0538, 0.0538, 3.0, 4096, 4096, 4096,1024,1024,1024, // 0.321
                  5.0, 2000.0, 1.0,
                  0.0, 0.0, 0.0, 0.0, 0,
                  8312,-10366,19768,0,30,0,
                  0,1,0,0,0,0};
Kinematics::Kinematics():m_config(Kinematics::configuration(datadata))
{

}

fsVec3d Kinematics::computeBodyAngles(const int *encoderValues)const 
{
    int e[3] = {encoderValues[0],encoderValues[1],encoderValues[2]};    

    pose p  = calculate_pose(m_config, e);
    double third_axis = p.tC;
/*
#ifdef SUPPORT_POLHEMV2
    if(m_config.variant==3)
        third_axis=polhemComputeLambda(p.tB,p.tC);
#endif
*/

    return fsVec3d(p.tA, p.tB, third_axis);
}

// ------------------------------------------------------------------------
// Billy notation
// ------------------------------------------------------------------------
double Cos(double t) { return cos(t); }
double Power(double n, int m) { return pow(n,m); }
double ArcCos(double t) { return acos(t); }
double Sin(double t) { return sin(t); }
double Sqrt(double n) { return sqrt(n); }
// ------------------------------------------------------------------------



double power(double d, int n){
//    if(n!=2) exit(1);
    return d*d;
}


fsVec3d Kinematics::computePosition(const int *encoderValues) 
{
    int e[3] = {encoderValues[0],encoderValues[1],encoderValues[2]};
    pose p  = calculate_pose(m_config, e);

    const double& Ln = p.Ln;
    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    const double& tB = p.tB;
    const double& tC = p.tC;
    double x,y,z;

    if(m_config.variant == 3){ // Polhem ver. 2
        double r[] = {0,0,0,0};
#ifdef SUPPORT_POLHEMV2
        polhemKinematics(tA,tB,tC,r);
        lastComputedLambda=r[3];
#endif
        if (std::isnan(r[0])) 
            std::cout << "NAN0: " << e[0] << " " << e[1] << " " << e[2] << "\n";
        if (std::isnan(r[1])) 
            std::cout << "NAN1: " << e[0] << " " << e[1] << " " << e[2] << "\n";
        if (std::isnan(r[2])) 
            std::cout << "NAN2: " << e[0] << " " << e[1] << " " << e[2] << "\n";
        if (std::isnan(r[3])) 
            std::cout << "NAN3: " << e[0] << " " << e[1] << " " << e[2] << "\n";

        x = (std::isnan(r[0])?-1:r[0]) - m_config.workspace_origin_x;
        y = (std::isnan(r[1])?-1:r[1]) - m_config.workspace_origin_y;
        z = (std::isnan(r[2])?-1:r[2]) - m_config.workspace_origin_z;
    } else { // All other devices have straight forward kinematics
        x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_x;
        y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
        z = Ln+Lb*cos(tB)-Lc*cos(tC)        - m_config.workspace_origin_z;
    }

    //return fsVec3d(tA*180/pi_d,tB*180/pi_d,tC*180/pi_d); // 2020-10-27 Report angles
    return fsVec3d(x,y,z);
}

fsVec3d Kinematics::computeMotorAmps(fsVec3d force, const int *encoderValues) 
{
    int e[3] = {encoderValues[0],encoderValues[1],encoderValues[2]};

    const pose p = calculate_pose(m_config, e);

    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    const double& tB = p.tB;
    const double& tC = p.tC;

    //std::cout << 180*tB/3.141592 << "   " << 180*tC/3.141592 <<"\n";


    double fx=force.x();
    double fy=force.y();
    double fz=force.z();

    double tx,ty,tz;
    fsVec3d t;

    if(m_config.variant == 3){ // Polhem v.2
#ifdef SUPPORT_POLHEMV2
        t = polhemComputeMotorAmps(force, tA, tB, tC);//, debugJacobians, debugTorques);
        
#endif
    } else {
        // All other devices uses same kinematic structure
        tx = -sin(tA)*(Lb*sin(tB)+Lc*sin(tC))*fx +  cos(tA)*(Lb*sin(tB)+Lc*sin(tC))*fy + 0*fz;
        ty = Lb*cos(tA)*cos(tB)*fx + Lb*sin(tA)*cos(tB)*fy -Lb*sin(tB)*fz;
        tz = Lc*cos(tA)*cos(tC)*fx + Lc*sin(tA)*cos(tC)*fy + Lc*sin(tC)*fz;
        t = fsVec3d(tx,ty,tz);

        // Gravity compensation
        const double& g=m_config.g_constant;
        const double& Lb_cm = m_config.length_cm_body_b;
        const double& Lc_cm = m_config.length_cm_body_c;
        const double& mB = m_config.mass_body_b;
        const double& mC = m_config.mass_body_c;

        t = t + g*fsVec3d( 0,
                            mB*Lb_cm*sin(tB) + mC*(Lb_cm + Lc_cm)*sin(tC),
                            mC*Lc_cm*sin(tC) );
    }




    double motorTorque[3];


    // Different directions of motor on different devices
    double dir[] = {m_config.motor_and_body_aligned_a?-1.0 : 1.0,
                    m_config.motor_and_body_aligned_b?-1.0 : 1.0,
                    m_config.motor_and_body_aligned_c?-1.0 : 1.0};

    motorTorque[0] =  t.m_x * dir[0] * m_config.diameter_capstan_a / m_config.diameter_body_a;
    motorTorque[1] =  t.m_y * dir[1] * m_config.diameter_capstan_b / m_config.diameter_body_b;
    motorTorque[2] =  t.m_z * dir[2] * m_config.diameter_capstan_c / m_config.diameter_body_c;


    // Set motor torque (t) through ampere
    double motorAmpere[] = { motorTorque[0] / m_config.torque_constant_motor_a,
                             motorTorque[1] / m_config.torque_constant_motor_b,
                             motorTorque[2] / m_config.torque_constant_motor_c };


    // Add polhem precomputed torques
#ifdef USE_PRECOMPUTED_GRAVCOMP
    motorAmpere[1]+=precompGravcompInterpolate(e[1],e[2],ma1)/1000.0;
    motorAmpere[2]+=precompGravcompInterpolate(e[1],e[2],ma2)/1000.0;
#endif

    return fsVec3d(motorAmpere[0],motorAmpere[1],motorAmpere[2]);
}

//static int debugcount = 0;
// Angles: tA,lambda,tD,tE
fsRot Kinematics::computeRotation(const int* encBase, const int* encRot, volatile double* angles)const 
{
    fsRot r;
    r.identity();

    // If no encoders d-f, we have just a 3-dof device. return identity.
    if(m_config.cpr_encoder_d == 0 ||
       m_config.cpr_encoder_e == 0 ||
       m_config.cpr_encoder_f == 0)
        return r;

    // From compute pos -------------------
    pose p  = calculate_pose(m_config, encBase);

    const double& tA = p.tA;
    const double& tC = p.tC;
    double dir[] = { m_config.enc_and_body_aligned_d?1.0:-1.0,
                     m_config.enc_and_body_aligned_e?1.0:-1.0,
                     m_config.enc_and_body_aligned_f?1.0:-1.0 };
    double tD = dir[0] * encRot[0]*2*pi/m_config.cpr_encoder_d;
    double tE = dir[1] * encRot[1]*2*pi/m_config.cpr_encoder_e;
    double tF = dir[2] * encRot[2]*2*pi/m_config.cpr_encoder_f;

    // Only return angles
    if(angles){
      angles[0] = tA;
      angles[1] = lastComputedLambda;
      angles[2] = tD;
      angles[3] = tE;
      return r;
    }

    // rotate about z (body a)
    fsRot rA;
    rA.rot_z(tA);

    // rotate about x
    fsRot rB;

    // rotate about y
    fsRot rC;
    if(m_config.variant == 3){ // Polhem v.2
#ifdef SUPPORT_POLHEMV2
        //const double& tB = p.tB;
        double lambda = lastComputedLambda;//polhemComputeLambda(tB, tC);
        rC.rot_y(-lambda);
#endif
    } else {
        rC.rot_y(-tC+3.141592/2); // TODO: need verification in vintage.
    }

    // rotate about x
    fsRot rD;
    rD.rot_x(tD);

    // rotate about y
    fsRot rE;
    rE.rot_y(tE);

    // rotate about x
    fsRot rF;
    rF.rot_x(tF);

    r =  rA*rB*rC*rD*rE*rF;
    return r;
}


} // namespace haptikfabriken
#endif