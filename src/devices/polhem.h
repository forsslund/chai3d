#ifndef POLHEM_H
#define POLHEM_H

#include "hfabmath.h"

//bool isnan(double d){return false;}


namespace haptikfabriken
{

// From e-mail 2019-03-21
constexpr double g = 9.81;

struct fsVec2d : public fsVec3d
{
    fsVec2d(double x, double y)
    {
        m_x = x;
        m_y = y;
        m_z = 0;
    }
    fsVec2d(const fsVec3d &v)
    {
        m_x = v.m_x;
        m_y = v.m_y;
        m_z = 0;
    }
    fsVec2d() : fsVec3d() {}
    double length() { return sqrt(m_x * m_x + m_y * m_y); }
};

constexpr double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647;
constexpr double pi_d = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647;

typedef double ld;
ld square(const ld &n) { return n * n; }
double length(fsVec3d v)
{
    return sqrt(v.m_x * v.m_x + v.m_y * v.m_y + v.m_z * v.m_z);
}

//    double a[3][3] = { {cos(t), -sin(t), 0 },
//                       {sin(t), cos(t), 0 },
//                       {0, 0, 1 }};
fsRot R(double theta)
{
    fsRot r;
    r.rot_z(theta);
    return r;
}

// Billy 2019-04-12
constexpr double m1=157.64e-3;
constexpr double m2=77.33e-3;
constexpr double m3=97.67e-3;
constexpr double m45=112.65e-3;
constexpr double xcdg1=30.123e-3;
constexpr double ycdg1=0.484e-3;
constexpr double xcdg2=-4.086e-3;
constexpr double ycdg2=1.911e-3;
constexpr double xcdg3=87.765e-3;
constexpr double ycdg3=1.978e-3;
constexpr double xcdg45=61.701e-3;
constexpr double ycdg45=0.576e-3;


// Without covers 2019-05-24
/*
constexpr double m1 = 131.96e-3;     // kg (with cover 157.64e-3)
constexpr double m2 = 77.33e-3;      // kg (with cover 77.33e-3)
constexpr double m3 = 51.04e-3;      // kg (with cover 97.67e-3)
constexpr double m45 = 69.85e-3;     // kg (with cover 112.65e-3)
constexpr double xcdg1 = 20.150e-3;  // m (with cover 30.123e-3)
constexpr double ycdg1 = 0.794e-3;   // m (with cover 0.484e-3)
constexpr double xcdg2 = -4.086e-3;  // m (with cover -4.086e-3)
constexpr double ycdg2 = 1.911e-3;   // m (with cover 1.911e-3)
constexpr double xcdg3 = 70.940e-3;  // m (with cover 87.765e-3)
constexpr double ycdg3 = 0.036e-3;   // m (with cover 1.978e-3)
constexpr double xcdg45 = 72.850e-3; // m (with cover 61.701e-3)
constexpr double ycdg45 = 0.733e-3;  // m (with cover 0.576e-3)
*/

constexpr double L0 = 34e-3;
constexpr double L1 = 165e-3;
constexpr double L2 = 25e-3;
constexpr double L3 = 190e-3;
constexpr double L4 = 34e-3;
constexpr double L5 = 130.8e-3;

void polhemGravcompkinematics(const double &t2,
                              const double &t3,
                              fsVec2d *r)
{
    // Lambda computation from above
    double q1 = 2. * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))) * sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3)));
    double q2 = ((-square(L2) + square(L0 + L1 * cos(t2)) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1) * square(sin(t2)) + square(L1 * sin(t2) - L2 * sin(t3))) / (q1));
    double q3 = (2. * L4 * sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3))));
    double q4 = ((-square(L3) + square(L4) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3))) / q3);
    double q5 = (2. * L1 * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))));
    double q6 = ((-square(L0) + square(L1) + square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))) / q5);
    double lambda = t2 - acos(q6) - acos(q4) - acos(q2);

    // From e-mail 2019-03-21
    fsVec2d O1cdg11(xcdg1, ycdg1);
    fsVec2d O2cdg22(xcdg2, ycdg2);
    fsVec2d O3cdg33(xcdg3, ycdg3);
    fsVec2d O45cdg4545(xcdg45, ycdg45);

    // alpha3
    fsVec2d ob(L1 * cos(t2) + L0 / 2, L1 * sin(t2) + 0);
    fsVec2d oc = ob - fsVec2d(L4 * cos(lambda), L4 * sin(lambda));
    fsVec2d oa(-L0 / 2, 0);
    double d9 = length(oc - oa);
    double cos180plusa3 = (L2 * L2 + L3 * L3 - d9 * d9) / (2 * L2 * L3);
    double a3 = acos(cos180plusa3) - pi;

    fsVec2d Ocdg1 = R(t2) * O1cdg11 + fsVec2d(L0 / 2, 0);
    fsVec2d Ocdg2 = R(t3) * O2cdg22 + fsVec2d(-L0 / 2, 0);
    fsVec2d Ocdg3 = R(t3) * (R(a3) * O3cdg33 + fsVec2d(L2, 0)) + fsVec2d(-L0 / 2, 0);
    fsVec2d Ocdg45 = R(t2) * (R(lambda - t2) * O45cdg4545 + fsVec2d(L1, 0)) + fsVec2d(L0 / 2, 0);

    r[0] = Ocdg1;
    r[1] = Ocdg2;
    r[2] = Ocdg3;
    r[3] = Ocdg45;
}

void polhemKinematics(const double t1, const double t2, const double t3, double *r)
{
    // 20190318
    //xb=Cos(theta1)*(L0/2. + L1*Cos(theta2) + L5*Cos(theta2 - ArcCos((-Power(L0,2) + Power(L1,2) + Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2))/(2.*L1*Sqrt(Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2)))) - ArcCos((-Power(L3,2) + Power(L4,2) + Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))/(2.*L4*Sqrt(Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2)))) - ArcCos((-Power(L2,2) + Power(L0 + L1*Cos(theta2),2) + Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1,2)*Power(Sin(theta2),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))/(2.*Sqrt(Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2))*Sqrt(Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))))));
    //yb=(L0/2. + L1*Cos(theta2) + L5*Cos(theta2 - ArcCos((-Power(L0,2) + Power(L1,2) + Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2))/(2.*L1*Sqrt(Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2)))) - ArcCos((-Power(L3,2) + Power(L4,2) + Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))/(2.*L4*Sqrt(Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2)))) - ArcCos((-Power(L2,2) + Power(L0 + L1*Cos(theta2),2) + Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1,2)*Power(Sin(theta2),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))/(2.*Sqrt(Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2))*Sqrt(Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))))))*Sin(theta1);
    //zb=L1*Sin(theta2) + L5*Sin(theta2 - ArcCos((-Power(L0,2) + Power(L1,2) + Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2))/(2.*L1*Sqrt(Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2)))) - ArcCos((-Power(L3,2) + Power(L4,2) + Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))/(2.*L4*Sqrt(Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2)))) - ArcCos((-Power(L2,2) + Power(L0 + L1*Cos(theta2),2) + Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1,2)*Power(Sin(theta2),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2))/(2.*Sqrt(Power(L0 + L1*Cos(theta2),2) + Power(L1,2)*Power(Sin(theta2),2))*Sqrt(Power(L0 + L1*Cos(theta2) - L2*Cos(theta3),2) + Power(L1*Sin(theta2) - L2*Sin(theta3),2)))));

    //xb=cos(theta1)*(L0/2. + L1*cos(theta2) + L5*cos(theta2 - acos((-power(L0,2) + power(L1,2) + power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2))/(2.*L1*sqrt(power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2)))) - acos((-power(L3,2) + power(L4,2) + power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2))/(2.*L4*sqrt(power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2)))) - acos((-power(L2,2) + power(L0 + L1*cos(theta2),2) + power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1,2)*power(sin(theta2),2) + power(L1*sin(theta2) - L2*sin(theta3),2))/(2.*sqrt(power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2))*sqrt(power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2))))));
    //yb=(L0/2. + L1*cos(theta2) + L5*cos(theta2 - acos((-power(L0,2) + power(L1,2) + power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2))/(2.*L1*sqrt(power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2)))) - acos((-power(L3,2) + power(L4,2) + power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2))/(2.*L4*sqrt(power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2)))) - acos((-power(L2,2) + power(L0 + L1*cos(theta2),2) + power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1,2)*power(sin(theta2),2) + power(L1*sin(theta2) - L2*sin(theta3),2))/(2.*sqrt(power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2))*sqrt(power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2))))))*sin(theta1);
    //zb=L1*sin(theta2) + L5*sin(theta2 - acos((-power(L0,2) + power(L1,2) + power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2))/(2.*L1*sqrt(power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2)))) - acos((-power(L3,2) + power(L4,2) + power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2))/(2.*L4*sqrt(power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2)))) - acos((-power(L2,2) + power(L0 + L1*cos(theta2),2) + power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1,2)*power(sin(theta2),2) + power(L1*sin(theta2) - L2*sin(theta3),2))/(2.*sqrt(power(L0 + L1*cos(theta2),2) + power(L1,2)*power(sin(theta2),2))*sqrt(power(L0 + L1*cos(theta2) - L2*cos(theta3),2) + power(L1*sin(theta2) - L2*sin(theta3),2)))));

    // 20190322 Breaking up into parts
    //std::cout << "t1 " << t1 << "    t2 " << t2 << "   t3 " <<t3 << "\n";

    /*
    ld q2 = sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3)));
    ld q3 = 2. * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))) * q2;
    ld q4 = acos((-square(L2) + square(L0 + L1 * cos(t2)) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1) * square(sin(t2)) + square(L1 * sin(t2) - L2 * sin(t3))) / q3);
    ld q5 = acos((-square(L3) + square(L4) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3))) / (2. * L4 * sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3)))));
    ld q7 = 2. * L1 * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2)));
    ld q8 = -square(L0) + square(L1) + square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2));
    ld q6 = acos(q8 / q7);
    ld q1 = L0 / 2. + L1 * cos(t2) + L5 * cos(t2 - q6 - q5 - q4);
    r[0] = cos(t1) * q1;

    q1 = square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3));
    q2 = 2. * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))) * sqrt(q1);
    q3 = -square(L2) + square(L0 + L1 * cos(t2)) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1) * square(sin(t2)) + square(L1 * sin(t2) - L2 * sin(t3));
    q4 = 2. * L4 * sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3)));
    q5 = -square(L3) + square(L4) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3));
    q6 = 2. * L1 * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2)));
    q7 = -square(L0) + square(L1) + square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2));
    q8 = L0 / 2. + L1 * cos(t2) + L5 * cos(t2 - acos(q7 / q6) - acos(q5 / q4) - acos(q3 / q2));
    r[1] = q8 * sin(t1);

    q1 = square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3));
    q2 = (2. * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))) * sqrt(q1));
    q3 = (-square(L2) + square(L0 + L1 * cos(t2)) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1) * square(sin(t2)) + square(L1 * sin(t2) - L2 * sin(t3))) / q2;
    q4 = 2. * L4 * sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3)));
    q5 = (-square(L3) + square(L4) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3)));
    q6 = (2. * L1 * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))));
    q7 = (-square(L0) + square(L1) + square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2)));
    q8 = t2 - acos(q7 / q6) - acos(q5 / (q4)) - acos(q3);
    r[2] = L1 * sin(t2) + L5 * sin(q8);
    */

    constexpr double sqL0 = L0*L0;
    constexpr double sqL1 = L1*L1;
    constexpr double sqL2 = L2*L2;
    constexpr double sqL3 = L3*L3;
    constexpr double sqL4 = L4*L4;
    constexpr double L0half = L0/2l;

    double L1cost2 = L1*cos(t2);
    double L1sint2 = L1*sin(t2);

    double a = L0 + L1cost2;
    double sq1 = square(a - L2*cos(t3));
    double sq2 = square(L1sint2 - L2*sin(t3));
    double sq3 = a*a;
    double sq4 = L1sint2*L1sint2;
    double sqrt1 = sqrt(sq1 + sq2);
    double sqrt2 = sqrt(sq3 + sq4);

    double x1 = 2.l*sqrt2*sqrt1;
    double x2 = -sqL3 + sqL4 + sq1 + sq2;
    double x3 = 2.l*L1*sqrt2;
    double x4 = 2.l*L4*sqrt1;
    double x5 = -sqL0 + sqL1 + sq3 + sq4;
    double x6 = -sqL2 + sq1 + sq2 + sq3 + sq4;

    double y1 = t2 - acos(x5/x3) - acos(x2/x4) - acos(x6/x1);
    double x7 = L0half + L1cost2 + L5*cos(y1);

    r[0]=x7*cos(t1);
    r[1]=x7*sin(t1);
    r[2]=L1sint2 + L5*sin(y1);    
    r[3]=y1; // "lambda"
}

fsVec3d polhemComputeMotorAmps(fsVec3d force, const double theta1,
                               const double theta2,
                               const double theta3) //, fsMatrix3 *debugJacobians, fsVec3d *debugTorques
{
    fsVec3d t;

    

    // Billy notation
    //----------------------------------------------------------------
    //----------------------------------------------------------------
    //----------------------------------------------------------------
    //Jacobian
    //----------------------------------------------------------------
    //----------------------------------------------------------------
    //----------------------------------------------------------------

    // Estimate a J
    constexpr double h = 0.00001;
    double dp[4];
    double dm[4];



    polhemKinematics(theta1 + h / 2, theta2, theta3, dp);
    polhemKinematics(theta1 - h / 2, theta2, theta3, dm);
    double dxyz_dt1[] = {(dp[0] - dm[0]) / h, (dp[1] - dm[1]) / h, (dp[2] - dm[2]) / h};
    polhemKinematics(theta1, theta2 + h / 2, theta3, dp);
    polhemKinematics(theta1, theta2 - h / 2, theta3, dm);
    double dxyz_dt2[] = {(dp[0] - dm[0]) / h, (dp[1] - dm[1]) / h, (dp[2] - dm[2]) / h};
    polhemKinematics(theta1, theta2, theta3 + h / 2, dp);
    polhemKinematics(theta1, theta2, theta3 - h / 2, dm);
    double dxyz_dt3[] = {(dp[0] - dm[0]) / h, (dp[1] - dm[1]) / h, (dp[2] - dm[2]) / h};

    double Jest[3][3] = {{double(dxyz_dt1[0]), double(dxyz_dt2[0]), double(dxyz_dt3[0])},
                         {double(dxyz_dt1[1]), double(dxyz_dt2[1]), double(dxyz_dt3[1])},
                         {double(dxyz_dt1[2]), double(dxyz_dt2[2]), double(dxyz_dt3[2])}};
    fsMatrix3 MJest;
    MJest.set(Jest);

    //debugJacobians[0] = MJest;

    //std::cout << "thetas: " << theta1 << " " << theta2 << " " << theta3 << "\n";
    //std::cout << "---------- JACOBIAN NUMERICAL -------\n";
    //std::cout << toString(MJest*1000.0);
    //std::cout << "-------------------------------------\n";
        //return fsVec3d(12,23,34);
    t = MJest.transpose() * force;

    //debugTorques[0] = t;
    #ifdef USE_DIRECT_COMPUTE_GRAVITY_COMPENSATION
    if (isnan(t.m_x) || isnan(t.m_y) || isnan(t.m_z))
    #else
    if (false)//(isnan(t.m_x) || isnan(t.m_y) || isnan(t.m_z))
    #endif
    {
        //std::cout << "Warning: Torque computation is NAN. Sets to 0\n";
        t.zero();
        //exit(1);
        fsMatrix3 m;
        //debugJacobians[0] = m;
    }
    else
    {

        // Have valid torque.
        // Compute gravity compensation
#ifdef USE_DIRECT_COMPUTE_GRAVITY_COMPENSATION
        fsVec3d tgravcomp[4];
        fsVec3d Fg[] = {g * fsVec3d(0, 0, m1),
                        g * fsVec3d(0, 0, m2),
                        g * fsVec3d(0, 0, m3),
                        g * fsVec3d(0, 0, m45)};

        constexpr double h = 0.001;
        constexpr double h1 = 1.0 / h;

        fsVec2d dp[4];
        fsVec2d dm[4];

        polhemGravcompkinematics(theta2 + h / 2, theta3, dp);
        polhemGravcompkinematics(theta2 - h / 2, theta3, dm);
        fsVec2d dxz_dt2[] = {(dp[0] - dm[0]) * h1,
                             (dp[1] - dm[1]) * h1,
                             (dp[2] - dm[2]) * h1,
                             (dp[3] - dm[3]) * h1};

        polhemGravcompkinematics(theta2, theta3 + h / 2, dp);
        polhemGravcompkinematics(theta2, theta3 - h / 2, dm);
        fsVec2d dxz_dt3[] = {(dp[0] - dm[0]) * h1,
                             (dp[1] - dm[1]) * h1,
                             (dp[2] - dm[2]) * h1,
                             (dp[3] - dm[3]) * h1};

        fsMatrix3 MJestG[4];
        for (int i = 0; i < 4; ++i)
        {
            double Jest[3][3] = {{0, dxz_dt2[i].m_x, dxz_dt3[i].m_x},
                                 {0, 0, 0},
                                 {0, dxz_dt2[i].m_y, dxz_dt3[i].m_y}};

            MJestG[i].set(Jest);
            //debugJacobians[i + 1] = MJestG[i];

            tgravcomp[i] = MJestG[i].transpose() * Fg[i];

            //std::cout << "Jacobian" << i << "T=\n" << toString(MJestG[i].transpose()) << "\n";
            //std::cout << "Fg"<< i << "= " << toString(Fg[i]) << "  JTF = " << toString(tgravcomp[i]) <<"\n";

            if (isnan(tgravcomp[i].m_x) ||
                isnan(tgravcomp[i].m_y) ||
                isnan(tgravcomp[i].m_z))
            {
                // std::cout << "NAN" << std::endl;

                tgravcomp[i].zero();
            }

            //debugTorques[i + 1] = tgravcomp[i];
        }

        // Add gravity compt torque to total torque
        for (int i = 0; i < 4; ++i)
            t = t + tgravcomp[i]; // Add to total torque
#endif

        
        
    }

    return t;
}

double polhemComputeLambda(const double t2, const double t3)
{

    double q1 = 2. * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))) * sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3)));
    double q2 = ((-square(L2) + square(L0 + L1 * cos(t2)) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1) * square(sin(t2)) + square(L1 * sin(t2) - L2 * sin(t3))) / (q1));
    double q3 = (2. * L4 * sqrt(square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3))));
    double q4 = ((-square(L3) + square(L4) + square(L0 + L1 * cos(t2) - L2 * cos(t3)) + square(L1 * sin(t2) - L2 * sin(t3))) / q3);
    double q5 = (2. * L1 * sqrt(square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))));
    double q6 = ((-square(L0) + square(L1) + square(L0 + L1 * cos(t2)) + square(L1) * square(sin(t2))) / q5);
    double lambda = t2 - acos(q6) - acos(q4) - acos(q2);
    if (isnan(lambda))
        lambda = 3.141592 / 2;
    return lambda;
}

} // namespace haptikfabriken

#endif // POLHEM_H
