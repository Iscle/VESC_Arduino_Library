This is a library for UART communication between the VESC and an Arduino board. Almost the full interface of the VESC is implemented in the library, only firmware upload is missing.  
  
Connect D0 (RX) to TX on the VESC  
Connect D1 (TX) to RX on the VESC  
Connect the grounds between the Arduino board and the VESC.  

This won't work on ATMEGA328 devices, due to lack of RAM memory.
Will work on Arduino MEGA 2560 and similar.
