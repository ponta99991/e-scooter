# Scooter model simulation

Explains the simulation of the e-scooter.

## Contents

 [CHANGELOG 2022-11-08]

- [GUI Startup](#gui-startup)

 [CHANGELOG 2022-10-31]

- [Simulation explained](#simulation-explained)
- [Startup simulation](#startup-simulation)
- [PID optimize](#pid-optimize)
- [Install Toolboxes](#install-toolboxes)
- [Import CAD files into MATLAB](#import-cad-files-into-matlab)
- [Setup the e-scooter](#setup-the-e-scooter)
- [Construct a plane](#construct-a-plane)
- [Combine plane with e-scooter](#combine-plane-with-e-scooter)


### GUI Startup

The easiest way to configure and run the simulation is with the GUI menu. 
In [MATLAB](#install-toolboxes) after installing the toolboxes and softwares in command window type "GUI" and enter key.
The GUI comes with several test variables to play around with and test the sytem with. 
The GUI comes with two tabs "Main menu" and "config". 
The main menu has options for altering the PID values both for the inner and outer PID. 
To have use of the Inner PID the "inner PID on" switch need to be on(otherwise the system will only care for one PID).
Press the "Run simulation" button to run the simulation. The black dot will shortly turn yellow to indicate that the simulation is up and running and turn green when all operations is done.

In the config tab it opens up configurations for further testing. 
change of:

- velocity to determine the constant forward velocity in km/h(the acceleration is 0 in the system).
- Simulation Time to determine for how long to simulate in seconds.
- Ts in seconds is how often the IMU takes sensor samples, such 0.01 as the standard is equal to 100Hz samplerate and 100 samples per second.
- X and Y determines how big of a plane the scooter is having where Y determines the width of the plane and X determines the length the scooter can travel in meters.
- Disturbance are defined in a Push in Newton meter and at which time the push should be introduced originally these are tested with [5 2] set the push to be outside of the simulation time scope to discard it(<0 or simulationtime>)
- The option reset returns the parameters from "scooterparam.m", otherwise if you want to save down a perticular configuration "save config" button does that and "load config" loads the parameters if they have been saved earlier.
- Pid optimization switch calculated either for one or two PIDs the optimal value. The "inner PID on" switch determines for one or two PIDs here as well. ***NOTE: see [PID optimize](#pid-optimize) before use***

### Simulation Explained

[CHANGELOG 2022-10-31]

The simulation simulates the lean angle using a sensor on the scooter body which is inputed to the control system consisting of two PID controllers.
The outer PID is determined with Outer\_p,Outer\_i,Outer\_d while the inner is determined with inner\_p,inner\_i,inner\_d(these values can be found in Scooterparam.m).
The Steering joint is controlled through the angle position in radians. 
Choose between ***Deltastar or Deltadot*** to determine one or two PIDs use, these are controlled by changing PIDswitch between "0" and "1" where "0" is using one PID and "1" controls both PIDs. 
Deltastar is only calculating the wanted stering angle and sets the joint to it. 
Deltadot is using two PIDs and outputs an velocity that is integrated(inside the PID block) to give an position to the joint.  
Both methods can balance the e-scooter and is limited to +/- 1 rad in stering angle.
Multiple scopes are located in the top part of the simulink including comparision between current and desired steering angle.
A small push is applied to the scooter at second 2(configure the disturbance with pushamp and phase in scooterparam.m) which is in simulink "scootermodelsim.slx" connected in scooter block-> 6-DOF joint block.

### Startup Simulation

Files needed:

- Scooterparam.m
- scootermodel\_DataFile.m
- PID\_optimisation.m
- scootermodelsim.slx
- Body\_Default\_sldprt.STEP
- Handle\_Default\_sldprt.STEP
- wheel\_Default\_sldprt.STEP

 [CHANGELOG 2022-10-26]

- wheel\_Default\_sldprt\_round.STEP
- plane.slx
- Equation_Bicycle_Model.slx

[CHANGELOG 2022-10-26]

The scooter function with one revolute joint on the handle and one revolute joint each for the wheels with an initialized constant velocity with 8km/h(can be confiured in scooterparam.m as "v") on the forward wheel. 
The wheels has two options one with rounded edges and one with a cylindrical form that can be changed in scooter block -> wheel_RIGID block.
In scooterparam.m the paramteres are measured from the [Mi Electric Scooter PRO 2](https://mistore.se/collections/pro-2/products/mi-electric-scooter-pro2-25km-h).
Also in scooterparam.m there exist some extra tweakable variables that could be changed. 

- friction variables depending on material on ground, in terms of Friction\_stat\_front, Friction\_dyn\_front, Friction\_stat\_back, Friction\_dyn\_back.
- damping and spring coefficients, in terms of damping\_front, spring\_front, damping\_back, spring\_back.
- plane variables to determine where the scooter can be, in terms of xPla, Ypla, Zpla.

[CHANGELOG 2022-10-31]

- PID coefficients for inner and outer loop  Outer\_p,Outer\_i,Outer\_d,inner\_p,inner\_i,inner\_d.

[CHANGELOG 2022-11-08]

Remove the comment at the bottom under the line "%Run the simulation" section to be able to run simulation from "scooterparam.m"
Run scooterparam.m to start simulation.

***NOTE: Recommended to try the [GUI](#gui-startup) first and understand it before altering other variables***

### PID optimize

If using PID optimisation, please have a look at PID_optimisation.m, under "Init".

- By changing outer_x and inner_x, PID combinations will be changed. For example, outer_p = 3.1:0.1:3.5 will test 3.1, 3.2, 3.3, 3.4 and 3.5, in combination with all possible other values specified.
    ***CAUTION: Adding values will increase operations quickly. For example, if using 10 values for each of the 6 PID variables of two PIDs, there will be 1 000 000 calculations. If each 6 in parallel operation takes 18 seconds on average, the execution time will be about one month.***
- half_error_band, startt, endt, timeresolution, analysetime are all for assessing stability of the PID value combinations, by looking at settling time.

The PID optimisation script will first run simulations as specified in GUI (one or two PIDs) and in "Init" section (PID combinations). Then, it will calculate settling time for these.

The GUI graphs will display 1s as 100 samples, this is because the lowest sampling rate is 1/Ts Hz (outer PID, which should have higher or equal sampling rate to inner). The best PID values will be shown under "Config". If there is no settled system within time = endtime-analysetime, then the graphs will display 'No asymptotically stable system found' and best PID values will be set to 0.

The best PID settling time and combination will be saved as "best_PID.mat" and the settling time of each PID combination as "settling_time.mat". The first column of these displays settling time(s) and the following columns PID values in order outer_p to inner_d. The simulation outputs used for calculating settling times will be saved as "simOut.mat". Each row of "settling_time.mat" correspond to each column of "simOut.mat".

### Install Toolboxes 

The simulation is done through [MATLAB](https://se.mathworks.com/products/new_products/latest_features.html) R2022b, [Simulink](https://se.mathworks.com/products/simulink.html), [Simscape](https://se.mathworks.com/products/simscape.html) together with [Simscape Multibody](https://se.mathworks.com/products/simscape-multibody.html) dynamics toolbox.

[CHANGELOG 2022-10-31]

 [Robotics System Toolbox](https://se.mathworks.com/products/robotics.html) and  [Control System Toolbox](https://se.mathworks.com/products/control.html).

 [CHANGELOG 2022-11-08]

 To be able to run parallell PID optimisation code "PID\_optimisation.m" the [Parallel Computing Toolbox](https://se.mathworks.com/products/parallel-computing.html),
 [MATLAB Report Generator](https://se.mathworks.com/products/matlab-report-generator.html) and [Simulink Report Generator](https://se.mathworks.com/products/simulink-report-generator.html) is required.


### Import CAD files into MATLAB.

(Only needed if the CAD model needs to be updated)

This simulation is based upon designing a e-scooter in CAD which then can be visualised in the simulation. 
Solidworks 2022 is used with the Multibody toolbox which also is compatible with PTCCreo and Autodesk Inventor.

If a new model is inserted follow this [tutorial](https://se.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html)
which creates a link between MATLAB and your CAD program.
Be sure to download both the multibody plugin and the install addon.

***Only PTCCreo, Autodesk Inventor and Solidworks is compatible***

After that [enable Simscape Multibody Link Plugin in SolidWorks](https://se.mathworks.com/help/smlink/ref/linking-and-unlinking-simmechanics-link-software-with-solidworks.html). By doing this you can export parts from solidworks directly into the workpath in matlab. 
- Type smlink_linksw in matlab command window to link to CAD program 
-  export the .xml file along with the individual parts in your workspace folder.
- calling the smimport('scootermodel');  in matlab command window creates a Datafile.m with a simulation CAD model. 

***Check if the joints are connected correctly in simulation and in datafile.***

The scooter model had a cylindrical joint originally that was changed to a revolute joint 'Frontwheel joint' in Simulink. Also a planar joint constrained the front wheel to the body which was removed. This demanded changes in DataFile, removed planar joint and cylindrical joint and copied a new revolute joint. The transformation or rotations should not be needing any configuration. 

Revolute joint config:

smiData.RevoluteJoint(3).Rz.Pos = 0;

smiData.RevoluteJoint(3).ID = "[Handle-1:-:wheel-2]";

### Setup the e-scooter 

The scooter has a individual block that takes frame and contact points as inputs and outputs the new frame. 

- Frame input - consist of the world frame which is rotated to allign to Z-axis which is done with 
rotationangle and rotationaxis in scooterparam.m file.
- contact points comes from the plane. With each scooter part having it's individual frame connected to the plane. 
without the contact points block it would be a scooter and a plane with no correspondance such the scooter would fall straight through the plane. Both wheels have an applied friction coefficient found in scooterparam.m. 
Friction\_stat/dyn\_back/front.

The steering angle is initialized with smiData.RevoluteJoint(2).Rz.Pos=0, thus the handle is straight. 

### Construct a plane

The plane comes from a solid block in simulink. 
It is defined with xPla, yPla, zPla stated in 
scooterparam.m 
The plane forwards it to contact points block to create conditions with the e-scooter.

###  Combine plane with e-scooter

The World is defined outside of this setting the gravity constant in z axis and adding the drop height translation. 
This is combined with the plane.

Thus the Simulation has a work flow like:

- World-> plane->contactpoints->scooter
- current steering angle and tilt angle->PID->scooter
- World-> scooter
- scooter->Sense tilt angle and velocity

### TF ref

https://lucris.lub.lu.se/ws/portalfiles/portal/4692388/625565.pdf
https://ieeexplore.ieee.org/document/9655223