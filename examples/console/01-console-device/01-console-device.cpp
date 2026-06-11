//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2026, CHAI3D
    (www.chai3d.org)

    Console example: enumerate haptic devices, open the first one, and stream
    its position/button state to the terminal — no GUI, no GLFW. Useful for
    verifying device detection (e.g. a SensAble PHANTOM via OpenHaptics, a
    Haptikfabriken device, etc.) over ssh or from scripts.

    Usage:  01-console-device [seconds]   (default: run until Ctrl-C)
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <csignal>
#include <cstdio>
#include <cstdlib>
//------------------------------------------------------------------------------
using namespace chai3d;
//------------------------------------------------------------------------------

static volatile sig_atomic_t stop = 0;
static void onSigInt(int) { stop = 1; }

int main(int argc, char* argv[])
{
    printf("-----------------------------------\n");
    printf("CHAI3D — 01-console-device\n");
    printf("-----------------------------------\n");

    // optional run duration in seconds (0 = until Ctrl-C)
    double duration = 0.0;
    if (argc > 1) duration = atof(argv[1]);
    signal(SIGINT, onSigInt);

    // enumerate haptic devices
    cHapticDeviceHandler* handler = new cHapticDeviceHandler();
    unsigned int numDevices = handler->getNumDevices();
    printf("haptic devices found: %u\n", numDevices);

    for (unsigned int i = 0; i < numDevices; i++)
    {
        cGenericHapticDevicePtr d;
        handler->getDevice(d, i);
        cHapticDeviceInfo info = d->getSpecifications();
        printf("  [%u] model: \"%s\"  manufacturer: \"%s\"\n",
               i, info.m_modelName.c_str(), info.m_manufacturerName.c_str());
    }

    if (numDevices == 0)
    {
        printf("no haptic device found.\n");
        printf("hint (PHANTOM on a PCIe parallel card): run through the soft-EPP\n");
        printf("shim and library path, e.g.  phantom-run ./01-console-device\n");
        return 1;
    }

    // open the first device
    cGenericHapticDevicePtr hapticDevice;
    handler->getDevice(hapticDevice, 0);
    if (!hapticDevice->open())
    {
        printf("error: failed to open haptic device.\n");
        return 2;
    }
    hapticDevice->calibrate();
    printf("device 0 open. streaming position (Ctrl-C to quit)\n\n");

    // stream pose + button state; print ~10x/s, read ~1 kHz
    cPrecisionClock clock;
    clock.start(true);
    double tNextPrint = 0.0;
    unsigned long reads = 0;

    while (!stop)
    {
        cVector3d position;
        hapticDevice->getPosition(position);
        bool button0 = false;
        hapticDevice->getUserSwitch(0, button0);
        reads++;

        double t = clock.getCurrentTimeSeconds();
        if (t >= tNextPrint)
        {
            printf("\rt=%6.1fs  pos [m]: % .5f % .5f % .5f  button0: %d  reads: %lu   ",
                   t, position.x(), position.y(), position.z(), (int)button0, reads);
            fflush(stdout);
            tNextPrint = t + 0.1;
        }
        if (duration > 0.0 && t >= duration) break;
        cSleepMs(1);
    }

    printf("\nclosing device.\n");
    hapticDevice->close();
    return 0;
}
