Steps:



Sensors pick up raw analog signals

Stores these analog signals onto a communication chip

Chip sends signals over to the FPGA

FPGA converts the signal into digital via ADC

Using the ADC converted signal, the FPGA uses DSP to clean up the signal

Via the STM32 hosted server, the FPGA sends over the processed signal

The STM32 then sends over the signal onto the computer

The computer then enters all of the data from the signal into MATLAB

In MATLAB, using a machine learning model it identifies exactly what the signal is

Once identified, it creates a text file where it describes what it is

It then sends over that text file onto the STM32 server

The STM32 then sends over the file onto the glasses

The glasses reads the text file and uses text to speech to read the file outloud for the user



Tech:


Microcontroller - STM32F446RE NUCLEO

Sensors - VL53L3CX, STHS34PF80, and an Arduino Ultrasonic Sensor

Coding Languages - C/C++
