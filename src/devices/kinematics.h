#ifndef KINEMATICS_H
#define KINEMATICS_H

//#include <chai3d.h>
//using namespace chai3d;
#include <string>
#include <sstream>
#include <iomanip>


struct fsVec3d {
    double m_x,m_y,m_z;
    fsVec3d(double x, double y, double z):m_x(x),m_y(y),m_z(z){}
    fsVec3d():m_x(0),m_y(0),m_z(0){}
    double x(){ return m_x; }
    double y(){ return m_y; }
    double z(){ return m_z; }
    void zero(){m_x=0;m_y=0;m_z=0;}
};
inline fsVec3d operator*(const double& d, const fsVec3d& v){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator+(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x+b.m_x, a.m_y+b.m_y, a.m_z+b.m_z); }

struct fsRot {
    //enum {GARBAGE, IDENTITY, ROT_X, ROT_Y, ROT_Z};
    double m[3][3];
    //double operator[](int i){return (double[])(r)[i];}
    fsRot(){identity();}
    inline void set(double m[3][3]){
        for(int i=0;i<3;++i)
            for(int j=0;j<3;++j)
                this->m[i][j] = m[i][j];
    }
    void identity();
    void rot_x(double t);
    void rot_y(double t);
    void rot_z(double t);


    /*
    static fsRot make(enum t=IDENTITY, double rad=0){
        if(t == IDENTITY){
            fsRot r;
            r.m = { {1, 0, 0 },
                    {0, 1, 0 },
                    {0, 0, 1 }};
            return r;
        }
    }*/

    //fsRot(double m[][3]){ for(int i=0;i<9;++i)(double*)(r[i])=(double*)(m);}
};

inline std::string toString(const fsVec3d& r){
   std::stringstream ss;
   ss.precision(3);
   ss.setf(std::ios::fixed);
   ss << std::setw(6) << r.m_x << " " << std::setw(6) << r.m_y << " " << std::setw(6) << r.m_z;
   return ss.str();
}
inline std::string toString(const fsRot& r){
   std::stringstream ss;
   ss.precision(3);
   ss.setf(std::ios::fixed);
   ss << std::setw(6) << r.m[0][0] << " " << std::setw(6) << r.m[0][1] << " " << std::setw(6) << r.m[0][2] << "\n";
   ss << std::setw(6) << r.m[1][0] << " " << std::setw(6) << r.m[1][1] << " " << std::setw(6) << r.m[1][2] << "\n";
   ss << std::setw(6) << r.m[2][0] << " " << std::setw(6) << r.m[2][1] << " " << std::setw(6) << r.m[2][2] << "\n";
   return ss.str();
}

inline fsRot operator*(const fsRot& a, const fsRot& b) {
    fsRot c;
    int i,j,m;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++) {
        c.m[i][j] = 0;
        for(m=0;m<3;m++)
          c.m[i][j] += a.m[i][m]*b.m[m][j];
      }
    }
    return c;
}


// Our 12*4=48 byte message (used both up and down)
struct woodenhaptics_message {
    float position_x;
    float position_y;
    float position_z;
    float command_force_x;
    float command_force_y;
    float command_force_z;
    float actual_current_0;
    float actual_current_1;
    float actual_current_2;
    float temperature_0;
    float temperature_1;
    float temperature_2;

    woodenhaptics_message():position_x(0),position_y(0),position_z(0),
                            command_force_x(0),command_force_y(0),command_force_z(0),
                            actual_current_0(0),actual_current_1(0),actual_current_2(0),
                            temperature_0(0),temperature_1(0),temperature_2(0){}
};


class Kinematics
{
public:

    fsVec3d computePosition(int ch_a, int ch_b, int ch_c){ int enc[3] = {ch_a,ch_b,ch_c}; return computePosition(enc); }
    fsVec3d computePosition(int* encoderValues); // encoders[3]
    fsVec3d computeMotorAmps(fsVec3d force, int* encoderValues);
    fsRot computeRotation(int* encBase, int* encRot);




    //! A collection of variables that can be set in ~/wooden_haptics.json
    struct configuration {
        double variant;                 // 0=WoodenHaptics default, 1=AluHaptics
        double diameter_capstan_a;      // m
        double diameter_capstan_b;      // m
        double diameter_capstan_c;      // m
        double length_body_a;           // m
        double length_body_b;           // m
        double length_body_c;           // m
        double diameter_body_a;         // m
        double diameter_body_b;         // m
        double diameter_body_c;         // m
        double workspace_origin_x;      // m
        double workspace_origin_y;      // m
        double workspace_origin_z;      // m
        double workspace_radius;        // m (for application information)
        double torque_constant_motor_a; // Nm/A
        double torque_constant_motor_b; // Nm/A
        double torque_constant_motor_c; // Nm/A
        double current_for_10_v_signal; // A
        double cpr_encoder_a;           // quadrupled counts per revolution
        double cpr_encoder_b;           // quadrupled counts per revolution
        double cpr_encoder_c;           // quadrupled counts per revolution
        double max_linear_force;        // N
        double max_linear_stiffness;    // N/m
        double max_linear_damping;      // N/(m/s)
        double mass_body_b;             // Kg
        double mass_body_c;             // Kg
        double length_cm_body_b;        // m     distance to center of mass
        double length_cm_body_c;        // m     from previous body
        double g_constant;              // m/s^2 usually 9.81 or 0 to
                                        //       disable gravity compensation

        std::string name;

        // Set values
        configuration(const double* k, std::string name="unnamed variant"):
          variant(k[0]),
          diameter_capstan_a(k[1]), diameter_capstan_b(k[2]), diameter_capstan_c(k[3]),
          length_body_a(k[4]), length_body_b(k[5]), length_body_c(k[6]),
          diameter_body_a(k[7]), diameter_body_b(k[8]), diameter_body_c(k[9]),
          workspace_origin_x(k[10]), workspace_origin_y(k[11]), workspace_origin_z(k[12]),
          workspace_radius(k[13]), torque_constant_motor_a(k[14]),
          torque_constant_motor_b(k[15]), torque_constant_motor_c(k[16]),
          current_for_10_v_signal(k[17]), cpr_encoder_a(k[18]), cpr_encoder_b(k[19]),
          cpr_encoder_c(k[20]), max_linear_force(k[21]), max_linear_stiffness(k[22]),
          max_linear_damping(k[23]), mass_body_b(k[24]), mass_body_c(k[25]),
          length_cm_body_b(k[26]), length_cm_body_c(k[27]), g_constant(k[28]), name(name){}

        configuration(){}

        // Some configurations (won't require to use the filesystem)
        static configuration woodenhaptics_v2015() {
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0};
            return Kinematics::configuration(data,"woodenhaptics_v2015 hardcoded");
        }

        static configuration polhem_v1() {
            double data[] = { 2, 0.010, 0.010, 0.010,
                              0.058, 0.174, 0.133,
                              0.180, 0.100, 0.100,
                              0.140, 0.000, 0.100, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                              5.0, 800.0, 8.0,
                              0.080, 0.080, 0.040, 0.070, 9.81};
            return Kinematics::configuration(data,"polhem_v1 hardcoded");
        }

        static configuration aluhaptics_v2() {
            double data[] = { 1, 0.0138, 0.0098, 0.0098,
                              0.111, 0.140, 0.111,
                              0.116, 0.076, 0.076,
                              0.140, 0.000, 0.000, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0};
            return Kinematics::configuration(data,"aluhaptics_v2 hardcoded");
        }

        static configuration vintage() {
            double data[] = { 1, 0.013, 0.010, 0.010,
                              0.056, 0.138, 0.112,
                              0.117, 0.077, 0.077,
                              0.140, 0.000, 0.000, 0.100,
                              0.0163, 0.0163, 0.0163, 3.0, 4000, 4000, 4000,
                              5.0, 800.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0};
            return Kinematics::configuration(data, "vintage hardcoded");
        }
    };

    Kinematics();
    Kinematics(configuration c):m_config(c) {}


    const configuration m_config;
};

std::string toJSON(const Kinematics::configuration& c);
Kinematics::configuration fromJSON(std::string json);

#endif // KINEMATICS_H
