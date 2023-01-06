#ifndef UHFABMATH_H
#define UHFABMATH_H

#include <string>
#include <sstream>
#include <iomanip>

// -----------------------------------------------------------------------------
namespace haptikfabriken {


struct fsVec3d {
    double m_x,m_y,m_z;
    fsVec3d(double x, double y, double z):m_x(x),m_y(y),m_z(z){}
    fsVec3d():m_x(0),m_y(0),m_z(0){}
    double x(){ return m_x; }
    double y(){ return m_y; }
    double z(){ return m_z; }
    void zero(){m_x=0;m_y=0;m_z=0;}
    double& operator[](int i) { if(i==0) return m_x; if(i==1) return m_y; return m_z;}
};
inline fsVec3d operator*(const double& d, const fsVec3d& v){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator*(const fsVec3d& v, const double& d){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator+(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x+b.m_x, a.m_y+b.m_y, a.m_z+b.m_z); }
inline fsVec3d operator-(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x-b.m_x, a.m_y-b.m_y, a.m_z-b.m_z); }


struct fsRot {
    double m[3][3];
    fsRot(){identity();}
    fsRot(double m[3][3]){set(m);}
    inline void set(double m[3][3]){
        for(int i=0;i<3;++i)
            for(int j=0;j<3;++j)
                this->m[i][j] = m[i][j];
    }
    fsRot(std::initializer_list<double> list){   // To initalize with list e.g.
        auto iter = list.begin();                //    fsRot r{1,2,3,
        for(int i=0;i<3;++i)                     //            4,5,6,
            for(int j=0;j<3;++j){                //            7,8,9};
                m[i][j]=*iter;
                iter++;
            }
    }
    void identity() {
        double a[3][3] = { {1, 0, 0 },
                           {0, 1, 0 },
                           {0, 0, 1 } };
        set(a);
    }
    void rot_x(double t) {
        double a[3][3] = { {1,   0,       0    },
                           {0, cos(t), -sin(t) },
                           {0, sin(t), cos(t)  } };
        set(a);
    }
    void rot_y(double t) {
        double a[3][3] = { {cos(t),  0, sin(t) },
                           {   0,    1,   0    },
                           {-sin(t), 0, cos(t) } };
        set(a);
    }
    void rot_z(double t) {
        double a[3][3] = { {cos(t), -sin(t), 0 },
                           {sin(t), cos(t), 0 },
                           {0, 0, 1 } };
        set(a);
    }
    fsRot transpose() {
        double a[3][3] = { {m[0][0], m[1][0], m[2][0] },
                           {m[0][1], m[1][1], m[2][1] },
                           {m[0][2], m[1][2], m[2][2] } };
        fsRot r;
        r.set(a);
        return r;
    }

};
//fsVec3d operator*(const fsRot& m, const fsVec3d& v);
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
inline fsRot operator*(const fsRot& a, const double& b) {
    fsRot c;
    int i,j;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++) {
         c.m[i][j] = a.m[i][j]*b;
      }
    }
    return c;
}
//std::string toString(const fsVec3d& r);
//std::string toString(const fsRot& r);

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
*/

inline fsVec3d operator*(const fsRot &m, const fsVec3d &v)
{
    fsVec3d r;

    r.m_x = m.m[0][0]*v.m_x + m.m[0][1]*v.m_y + m.m[0][2]*v.m_z;
    r.m_y = m.m[1][0]*v.m_x + m.m[1][1]*v.m_y + m.m[1][2]*v.m_z;
    r.m_z = m.m[2][0]*v.m_x + m.m[2][1]*v.m_y + m.m[2][2]*v.m_z;

    return r;
}

inline std::string toString(const haptikfabriken::fsVec3d &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m_x << ", " << std::setw(6) << r.m_y << ", " << std::setw(6) << r.m_z;
    return ss.str();
}

inline std::string toString(const haptikfabriken::fsRot &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m[0][0] << ", " << std::setw(6) << r.m[0][1] << ", " << std::setw(6) << r.m[0][2] << ",\n";
    ss << std::setw(6) << r.m[1][0] << ", " << std::setw(6) << r.m[1][1] << ", " << std::setw(6) << r.m[1][2] << ",\n";
    ss << std::setw(6) << r.m[2][0] << ", " << std::setw(6) << r.m[2][1] << ", " << std::setw(6) << r.m[2][2] << "\n";
    return ss.str();
}
// -----------------------------------------------------------------------------



typedef fsRot fsMatrix3;
class Kinematics
{
public:

    fsVec3d computePosition(int ch_a, int ch_b, int ch_c) { int enc[3] = { ch_a,ch_b,ch_c }; return computePosition(enc); }
    fsVec3d computePosition(const int* encoderValues); // encoders[3]
    fsVec3d computeBodyAngles(const int* encoderValues) const; // encoders[3]
    fsVec3d computeMotorAmps(fsVec3d force, const int* encoderValues);
    fsRot computeRotation(const int* encBase, const int* encRot, volatile double* angles = 0) const;

    double lastComputedLambda = 0;


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
        double cpr_encoder_d;           // quadrupled counts per revolution
        double cpr_encoder_e;           // quadrupled counts per revolution
        double cpr_encoder_f;           // quadrupled counts per revolution
        double max_linear_force;        // N
        double max_linear_stiffness;    // N/m
        double max_linear_damping;      // N/(m/s)
        double mass_body_b;             // Kg
        double mass_body_c;             // Kg
        double length_cm_body_b;        // m     distance to center of mass
        double length_cm_body_c;        // m     from previous body
        double g_constant;              // m/s^2 usually 9.81 or 0 to
                                        //       disable gravity compensation
        double calibrate_enc_a;
        double calibrate_enc_b;
        double calibrate_enc_c;
        double calibrate_enc_d;
        double calibrate_enc_e;
        double calibrate_enc_f;

        // Look at each body and put your right hand fingers in the direction of
        // where its angle increases. Is the motor pointing in the
        // direction of your thumb?
        double motor_and_body_aligned_a;  // true or false (0 or 1)
        double motor_and_body_aligned_b;
        double motor_and_body_aligned_c;

        // Look at each body and put your right hand fingers in the direction
        // where its angle increases. Is the output from the
        // encoder increasing in the same direction?
        double enc_and_body_aligned_d;  // true or false (0 or 1)
        double enc_and_body_aligned_e;
        double enc_and_body_aligned_f;

        std::string name;

        // Set values
        configuration(const double* k, std::string name = "unnamed variant") :
            variant(k[0]),
            diameter_capstan_a(k[1]), diameter_capstan_b(k[2]), diameter_capstan_c(k[3]),
            length_body_a(k[4]), length_body_b(k[5]), length_body_c(k[6]),
            diameter_body_a(k[7]), diameter_body_b(k[8]), diameter_body_c(k[9]),
            workspace_origin_x(k[10]), workspace_origin_y(k[11]), workspace_origin_z(k[12]),
            workspace_radius(k[13]), torque_constant_motor_a(k[14]),
            torque_constant_motor_b(k[15]), torque_constant_motor_c(k[16]),
            current_for_10_v_signal(k[17]), cpr_encoder_a(k[18]), cpr_encoder_b(k[19]),
            cpr_encoder_c(k[20]), cpr_encoder_d(k[21]), cpr_encoder_e(k[22]),
            cpr_encoder_f(k[23]),
            max_linear_force(k[24]), max_linear_stiffness(k[25]),
            max_linear_damping(k[26]), mass_body_b(k[27]), mass_body_c(k[28]),
            length_cm_body_b(k[29]), length_cm_body_c(k[30]), g_constant(k[31]),
            calibrate_enc_a(k[32]), calibrate_enc_b(k[33]), calibrate_enc_c(k[34]),
            calibrate_enc_d(k[35]), calibrate_enc_e(k[36]), calibrate_enc_f(k[37]),
            motor_and_body_aligned_a(k[38]), motor_and_body_aligned_b(k[39]),
            motor_and_body_aligned_c(k[40]), enc_and_body_aligned_d(k[41]),
            enc_and_body_aligned_e(k[42]), enc_and_body_aligned_f(k[43]),
            name(name) {}

        configuration() {}




        static configuration default_woody() {
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,0,0,0,0,
                              0,1,0,0,0,0 };
            return Kinematics::configuration(data);
        }

        // Some configurations (won't require to use the filesystem)
        static configuration woodenhaptics_v2019() {
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.321, 0.0603, 0.0603, 3.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,6000,0,0,0,
                              0,0,0,0,0,0 };
            return Kinematics::configuration(data, "woodenhaptics_v2019 hardcoded");
        }

        // Some configurations (won't require to use the filesystem)
        static configuration woodenhaptics_v2015() {
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.0259, 0.0259, 0.0259, 6.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,6000,0,0,0,
                              0,0,0,0,0,0 };
            return Kinematics::configuration(data, "woodenhaptics_v2015 hardcoded");
        }

        static configuration polhem_v1() {
            double data[] = { 2, 0.010, 0.010, 0.010,
                              0.058, 0.174, 0.133,
                              0.180, 0.100, 0.100,
                              0.140, 0.000, 0.100, 0.100,
                              0.0538, 0.0538, 0.0538, 3.0, 2000, 2000, 2000,1024,1024,1024,
                              5.0, 800.0, 8.0,
                              0.080, 0.080, 0.040, 0.070, 0,
                              0,0,-5000,0,0,0,
                              1,1,1,0,1,0 };
            return Kinematics::configuration(data, "polhem_v1 hardcoded");
        }


        static configuration polhem_v2() {
            double data[] = { 3, 0.0288, 0.0130, 0.010, // 3, 0.0288,0.0077,0.010 originally
                              0.1, 0.165, 0.1308,
                              0.175, 0.100, 0.100,
                //                              -0.137, 0.182, -0.0217, 0.150,
                                              0.137, 0, 0, 0.150,
                                              0.321, 0.0538, 0.0538, 3.0, 4096, 4096, 4096,1024,1024,1024,
                                              5.0, 5000.0, 1.0,
                                              0.0, 0.0, 0.0, 0.0, 0,
                                              8327,-10926,27140,0,30,0, // second was -18477
                                              0,1,0,0,0,0 };
            return Kinematics::configuration(data, "polhem_v2 hardcoded");
        }

        static configuration polhem_v3() {
            double data[] = { 3, 0.0288534, 0.0136981, 0.0137281, // 2020-12-21
                              0.1, 0.165, 0.1308,
                              0.175, 0.100, 0.100,
                              0.2045, 0.0, 0.0325, 0.150,
                              0.0603, 0.0538, 0.0538, 3.0, 4096, 4096, 4096,1024,1024,1024,
                              5.0, 2000.0, 1.0,
                              0.0, 0.0, 0.0, 0.0, 0,
                              8312,-10366,19764,0,30,0,
                              0,1,0,0,0,0 };
            return Kinematics::configuration(data, "polhem_v3 hardcoded");
        }

        static configuration aluhaptics_v2() {
            double data[] = { 1, 0.0138, 0.0098, 0.0098,
                              0.111, 0.140, 0.111,
                              0.116, 0.076, 0.076,
                              0.140, 0.000, 0.000, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,0,0,0,0,
                              0,0,0,0,0,0 };
            return Kinematics::configuration(data, "aluhaptics_v2 hardcoded");
        }

        static configuration vintage() {
            double data[] = { 1, 0.013, 0.010, 0.010,
                              0.056, 0.138, 0.112,
                              0.117, 0.077, 0.077,
                              0.140, 0.000, 0.000, 0.100,
                              0.0163, 0.0163, 0.0163, 3.0, 4000, 4000, 4000,0,0,0,
                              5.0, 800.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,0,0,0,0,
                              0,0,0,0,1,1 };
            return Kinematics::configuration(data, "vintage hardcoded");
        }
    };

    Kinematics();
    Kinematics(configuration c) :m_config(c) {}

    fsVec3d debugTorques[5];
    fsMatrix3 debugJacobians[5];


    configuration m_config;
};

std::string toJSON(const Kinematics::configuration& c);
Kinematics::configuration fromJSON(std::string json);

} // Namespace haptikfabriken
#endif // UHFABMATH_H
