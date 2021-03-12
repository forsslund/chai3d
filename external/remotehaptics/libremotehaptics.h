#ifndef LIBREMOTEHAPTICS_H
#define LIBREMOTEHAPTICS_H

class MyNetworkHandler;

class Libremotehaptics
{
public:
    struct Vector3d {
        double x;
        double y;
        double z;

        void zero(){
            x=0; y=0; z=0;
        }
    };

    typedef double RotationMatrix[9];

    Libremotehaptics();    
    bool init(int port);
    void getPosition(Vector3d &p);
    void getOrientation(RotationMatrix &r);
    void setForce(Vector3d &p);
    void setTorque(Vector3d &t);
    bool getUserSwitch(int index=0);




protected:
    MyNetworkHandler *netHandler;

};

#endif // LIBREMOTEHAPTICS_H
