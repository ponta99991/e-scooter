# autobike22-labview

Here is where the autobike22 labview code lives.
Explains the code functionality and usage.

##Contents 

- GUI
- PID
- Library path

# Files

- escooter.lvproj

"dll_so" folder:
- vesc.dll
- libvesc.so

"Bin" folder:
- ForwardMotor.vi
- IMU.vi
- PID.vi
- InitFPGA.vi
- InitIMU.vi
- Main.vi
- PID.vi
- SteeringControl.vi
- UART.vi
- VescSetRpm.vi

# GUI inside Main.vi

(1) Control steering with one or two PID values, angle limit, and reset encoder. View the steering with encoder values and the duty cycle (changing steering speed) coming from the PID control.
(2) Forward Motor speed adjustment.
(3) Start steering and forward motor. This needs to be done while running the program.
(4) Stop the program with E-stop. The indicator lights up when you press.
(5) View what the duty cycle is, to test IMU.
(6), (7) IMU charts, both gyro and acceleration. The program is currently using "X Gyro" only.
(8) Compiling speed of loops.

# PID.vi

Using previous iteration PIDs. One using two (inner) PID, and one using a single PID. Outputs PWM, indicating speed of steering motor.

# .so library path

The path to the .so file needs to be changed to your path (i.e. to the dll_so folder) inside the Call Library Function of VescSetRpm.vi. The function used is int32_t setRpm(uint8_t *array, int32_t rpm);

