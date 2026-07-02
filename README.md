Install in Linux:

1. Clone hfabapi as a sibling folder to this folder
2. Install dependencies: sudo apt install libasound2-dev libxcursor-dev libxrandr-dev libxinerama-dev build-essential libgl1-mesa-dev libglu1-mesa-dev libusb-1.0-0-dev
3. cmake .
4. make -j5

Run:
cd bin/lin-x86_64
./01-mydevice

## Building a module (BULLET, GEL, ...) and its examples

Modules live under `modules/<NAME>` and build in-source too, against the
chai3d tree built above. Their `CMakeLists.txt` needs to find chai3d (and,
for the GLFW-based examples, GLFW) via `CMAKE_PREFIX_PATH` — pass absolute
paths:

```bash
cd modules/BULLET   # or modules/GEL
cmake . -DCMAKE_PREFIX_PATH="$(cd ../..; pwd);$(cd ../../extras/GLFW; pwd)"
make -j$(nproc)
```

This produces `libchai3d-<NAME>.a` in the module directory and the module's
example executables under `modules/<NAME>/bin/lin-x86_64/`.
