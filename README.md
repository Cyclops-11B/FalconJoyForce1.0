Software to run the Novint Falcon universally as an XInput device that receives rumble data from various programs and transforms it into 3D forces. 

This program fixes seveal issues with the aging Novint Falcon, includeing no modern software support (32-bit vs 64), no modern programing support (needing specific mods for new games), and erronious anti-cheat banning. 
This program is designed to work in conjunction with an external microcontroller as the real XInput device.

Step 1: Acquire a Raspberry Pi Pico 2 W and a CP2102. Wire pin 1 (GP0) to Rx on the CP, pin 2 (GP1) to Tx on the CP, and pin 3 (GND) to Ground on the CP.

Step 2: Flash one of the Rumble Route programs to the Raspberry Pi Pico.

Step 3: Install the Force Dimension SDK and the driver for the CP2102.

Step 4: Find the CP2102 in your device manager and note what COM line it is running on (COM4, COM,6, etc)

Step 5: Clone this repository and input that COM# line into the configuration. 


Startup and go
