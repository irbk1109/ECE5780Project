# ECE5780Project
Authors:
  Isaac Kime, Sam Wimmer, Sean Koo, Subin Kim


  
Basic Setup Instructions:
  1. Insert sensors into the 3D parking garage and assemble the garage.
  2. Connect the black ground, red voltage (5 volts), and white output wires of the sensors to the PCB.
  3. Connect the outputs of the PCB to PB4 and PB5 of the STM32 microcontroller. (PB4 is parking status and PB5 is gate status)
  4. Connect OLED to the STM32 at pins ************* add this.
  5. Download and run the main.c file and supporting OLED library then build and flash it to the microcontroller.
  6. The status of a car at the gate or in the stall will be shown on the OLED screen as well as by the LEDs from the PCB.
