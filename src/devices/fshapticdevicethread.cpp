#include "kinematics.h"
#include "fshapticdevicethread.h"

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>


#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <string>

using boost::asio::ip::udp;
using namespace std;

enum { max_length = 1024 };

void FsHapticDeviceThread::server(boost::asio::io_service& io_service, unsigned short port)
{
  udp::socket sock(io_service, udp::endpoint(udp::v4(), port));
  for (;;)
  {

    unsigned char data[max_length];
    //for(int i=0;i<max_length;++i)
    //    data[i]='\0';


    udp::endpoint sender_endpoint;
    size_t length = sock.receive_from(
        boost::asio::buffer(data, max_length), sender_endpoint);
    mtx_force.lock();
    if(wait_for_next_message && newforce){
        sem_force_sent.post();
        newforce=false;
    }
    mtx_force.unlock();


/*
    cout << "Got " << length << " bytes:\n";
    for(int i=0;i<length;++i){
        cout << i << ": " << int(data[i]) << "\n";

    }
*/

    int ch_a=0;
    int ch_b=0;
    int ch_c=0;

    fsVec3d pos;
    pos.zero();

    if (data[0]==0xA4 && length>8){
        in_msg* in = reinterpret_cast<in_msg*>(data);

        ch_a = in->getEnc(0);
        ch_b = in->getEnc(1);
        ch_c = in->getEnc(2);


        // Compute position
        pos = kinematics.computePosition(ch_a,ch_b,ch_c);
        int base[] = {ch_a, ch_b, ch_c};
        int rot[]  = {in->getEnc(3), in->getEnc(4), in->getEnc(5)};
        fsRot r = kinematics.computeRotation(base,rot);
/*
        std::cout << "Ch readings: " << ch_a << ", " << ch_b << ", " << ch_c << "      " << rot[0] << "," << rot[1] << "," << rot[2] << "\n";
        //std::cout << in->toString() << "\n";
        std::cout << "Computed position: " << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
        std::cout << "Computed rotation: [" << r.m[0][0] << " " << r.m[0][1] << " " << r.m[0][2] << "\n"
                                            << r.m[1][0] << " " << r.m[1][1] << " " << r.m[1][2] << "\n"
                                            << r.m[2][0] << " " << r.m[2][1] << " " << r.m[2][2] << " ]\n\n";
                                            */
        mtx_pos.lock();
        latestPos = pos;
        latestRot = r;
        latestEnc[0]=ch_a;
        latestEnc[1]=ch_b;
        latestEnc[2]=ch_c;
        mtx_pos.unlock();
    }



    fsVec3d f;
    f.zero();


#define COMMANDED


#ifdef COMMANDED
    mtx_force.lock();
    f = nextForce;
    mtx_force.unlock();

#endif

#ifdef SPRING
    double k=100;
    f = k * (-pos); // Spring to middle
#endif

#ifdef WALL
    double k=2000;
//      if(pos.y() < 0) f = fsVec3d(0,-k*pos.y(),0);
//     if(ch_a<0)
#endif

#ifdef BOX
    double k=1000;
    double b=0.03;
    double x=pos.x();
    double y=pos.y();
    double z=pos.z();
    double fx,fy,fz;
    fx=0;fy=0;fz=0;

    if(x >  b) fx = -k*(x-b);
    if(x < -b) fx = -k*(x+b);
    if(y >  b) fy = -k*(y-b);
    if(y < -b) fy = -k*(y+b);
    if(z >  b) fz = -k*(z-b);
    if(z < -b) fz = -k*(z+b);
    f = fsVec3d(fx,fy,fz);
#endif

    int enc[3] = {ch_a,ch_b,ch_c};
    fsVec3d amps = kinematics.computeMotorAmps(f,enc);

#ifdef INSTAB
    amps.zero();
    if(ch_a < 0)
    amps.m_x = -double(ch_a)/10.0;
#endif

    out_msg out;
    out.milliamps_motor_a = int(amps.x()*1000.0);
    out.milliamps_motor_b = int(amps.y()*1000.0);
    out.milliamps_motor_c = int(amps.z()*1000.0);


#ifdef STATIC_CURRENT
    if(pos.y()>0){
        out.milliamps_motor_a = -1000;
        out.milliamps_motor_b = -1000;
        out.milliamps_motor_c = -1000;
    }
#endif




    // Cap at 2A since Escons 24/4 cant do more than that for 4s
    if(out.milliamps_motor_a >= 2000) out.milliamps_motor_a = 1999;
    if(out.milliamps_motor_b >= 2000) out.milliamps_motor_b = 1999;
    if(out.milliamps_motor_c >= 2000) out.milliamps_motor_c = 1999;

    if(out.milliamps_motor_a <= -2000) out.milliamps_motor_a = -1999;
    if(out.milliamps_motor_b <= -2000) out.milliamps_motor_b = -1999;
    if(out.milliamps_motor_c <= -2000) out.milliamps_motor_c = -1999;


    //std::cout << "Force: " << f.x() << ", " << f.y() << ", " << f.z() << "\n";
    //std::cout << "mA: " << out.milliamps_motor_a << ", " << out.milliamps_motor_b << ", " << out.milliamps_motor_c << "\n";




    char* const buf = reinterpret_cast<char*>(&out);
/*
    boost::this_thread::sleep_for( boost::chrono::microseconds(500) );
    string out = "1 2 3";


    //sock.send_to(boost::asio::buffer(data, length), sender_endpoint);

*/
    sock.send_to(boost::asio::buffer(buf, sizeof(out)), sender_endpoint);
    currentForce = f;

    // Current force is now the set force
    //mtx_force.unlock();
    //mtx_force.lock();

}
}

FsHapticDeviceThread::FsHapticDeviceThread(bool wait_for_next_message):
    sem_force_sent(0),newforce(false),wait_for_next_message(wait_for_next_message)
{
    app_start = chrono::steady_clock::now();

    io_service = new boost::asio::io_service();

    boost::thread* m_thread;
    m_thread = new boost::thread(boost::bind(&FsHapticDeviceThread::thread, this));

    latestEnc[0]=0;
    latestEnc[1]=0;
    latestEnc[2]=0;



}

void FsHapticDeviceThread::thread()
{


  try
  {



    server(*io_service, 47111);

  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
    }

}

