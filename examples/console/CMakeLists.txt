#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2026, CHAI3D
#  (www.chai3d.org)
#
#  Console (no-GUI) examples — device detection / streaming over a terminal.

# build all targets
foreach (example 01-console-device)

  file (GLOB source ${example}/*.cpp)
  add_executable (${example} ${source})
  target_link_libraries (${example} ${CHAI3D_LIBRARIES})

endforeach ()
