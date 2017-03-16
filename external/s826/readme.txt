
 2014-12-08
  
 This folder is for the driver and api of the Sensoray S826 DAQ PCIe card. It is the 
 only dependency for the Wooden Haptics (and similar) devices. 

 In /lib you find pre-compiled (on Ubuntu 14.04 64bit) statically linked versions 
 of the 826 sdk ready to be linked with Chai3D.

 The source for those and the driver for the card itself is found in sdk_826_linux_3.3.7.tar.bz2.

 If you have a S826 card, you have to install its drivers to use it:
 - Unpack (tar -xjvf) sdk_826_linux_3.3.7.tar.bz2
 - Follow the instructions in their README


   Hint: I needed to apt-get install gcc-multilib in order
         to compile the kernel module.

/Jonas (jofo02@kth.se) 


