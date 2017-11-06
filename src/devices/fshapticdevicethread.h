#ifndef FSHAPTICDEVICETHREAD_H
#define FSHAPTICDEVICETHREAD_H
//#define WIN32_LEAN_AND_MEAN

#include <boost/asio.hpp> // Note: pollutes namespace
#include <boost/thread/mutex.hpp>

// For sleeping
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>



#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <fstream>

#include "kinematics.h"
//#include <chai3d.h>
using namespace std;
using boost::asio::ip::udp;





class FsHapticDeviceThread
{
public:
    FsHapticDeviceThread(bool wait_for_next_message=false);
    void server(boost::asio::io_service& io_service, unsigned short port);
    void thread();

    Kinematics kinematics;
    chrono::steady_clock::time_point app_start;

    fsVec3d latestPos;
    fsRot latestRot;
    int latestEnc[3];
    fsVec3d currentForce;
    fsVec3d nextForce;

    boost::mutex mtx_pos;
    boost::mutex mtx_force;
    boost::interprocess::interprocess_semaphore sem_force_sent;
    bool newforce;

    const bool wait_for_next_message;

    inline fsVec3d getPos() {
        mtx_pos.lock();
        fsVec3d p = latestPos;
        mtx_pos.unlock();
        return p;
    }
    inline fsRot getRot() {
        mtx_pos.lock();
        fsRot r = latestRot;
        mtx_pos.unlock();
        return r;
    }

    inline void setForce(fsVec3d f){
        mtx_force.lock();
        nextForce = f;
        newforce = true;
        mtx_force.unlock();

        // wait until at least one new force message has been sent (received a new package)
        if(wait_for_next_message)
            sem_force_sent.wait();
    }




    struct out_msg {
        unsigned char start = 0xa4;
        unsigned char number = 0x07;
        unsigned char number2 = 0x51;
        int milliamps_motor_a;
        int milliamps_motor_b;
        int milliamps_motor_c;
        unsigned char end = 0xa4;
        string toString() { stringstream ss; ss<<"mA: " << milliamps_motor_a << " "
                                                        << milliamps_motor_b << " "
                                                        << milliamps_motor_c << "";
                            return ss.str(); }
    };

    // A unsigned number in 2 bytes
    struct utwobyte {
        unsigned char low;
        unsigned char high;
        utwobyte(unsigned char low, unsigned char high):low(low),high(high){}
        utwobyte(){}
        int toInt(){
            return low+(high<<8);
        }
    };
    struct twobyte {
        unsigned char low;
        unsigned char high;
        twobyte(unsigned char low, unsigned char high):low(low),high(high){}
        twobyte(){}
        int toInt(){
            int ch = low+(high<<8);
            return (ch > 0x7FFF)? ch - 0xFFFF : ch;
        }
    };

    struct in_msg {
        unsigned char start = 0xa4;
        unsigned char number1 = 0x07;
        unsigned char number2 = 0x42;
        unsigned char encoder_abcdef[12];
        utwobyte timestamp;
        utwobyte id;
        unsigned char checksum;


        int getEnc(int i) {
            int ch = encoder_abcdef[i*2]+(encoder_abcdef[i*2+1]<<8);
            return (ch > 0x7FFF)? ch - 0xFFFF : ch;
        }

        string toString() { stringstream ss; ss<<"enc: " << getEnc(0) << " "
                                                         << getEnc(1) << " "
                                                         << getEnc(2) << " "
                                                         << getEnc(3) << " "
                                                         << getEnc(4) << " "
                                                         << getEnc(5) << " "
                                               <<"ts_mbed: " <<  timestamp.toInt() << " "
                                               <<"id_mbed: " <<  id.toInt();
                            return ss.str(); }
    };


    double to_us(chrono::steady_clock::time_point ts) {
        auto diff = ts-app_start;
        return double(chrono::duration <double, micro> (diff).count());
    }












    boost::asio::io_service* io_service;

















};

#endif // FSHAPTICDEVICETHREAD_H
