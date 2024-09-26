%% Monitor and Tune PX4 Host Target Flight Controller with Simulink-Based Plant Model
% This example shows how to use the UAV Toolbox Support Package for PX4
% Autopilots to verify the controller design using PX4 Host Target versus the simulator designed in Simulink(R).
%
% The UAV Toolbox Support Package for PX4 Autopilots enables you to use Simulink to design a flight controller algorithm to stabilize the vehicle based on the 
% current vehicle attitude, position, and velocity and also track the desired attitude using Simulink. The MAVlink blocks in the UAV Toolbox enable you to read and write the 
% MAVLink HIL_* messages and design the plant dynamics. 
%
% This example shows how to validate a position controller design on a medium-sized quadrotor plant using a single Simulink model, and then take the same controller 
% and plant model and simulate it with the PX4 source code in, what the PX4 community calls it, Software In The Loop (SITL) simulation.

% Copyright 2022 -23 The MathWorks, Inc.

%% Prerequisites
%
% * If you are new to Simulink, watch the   
% <https://www.mathworks.com/videos/simulink-quick-start-78774.html Simulink Quick Start> video.
%
% * Perform the initial <docid:uav_doccenter#mw_536a8bb9-a780-45b4-9ed5-5d4aa47126d8 setup and configuration tasks> of the 
% support package using Hardware Setup screens. In the *Select a PX4 Autopilot and Build Target* screen, select |PX4 Host Target| as the PX4 Autopilot board from the drop-down list.
%
% <<../px_sitl_hwsetup.png>>
% 
% * For more information on how to verify the controller design using the jMAVSim simulator, 
% see <docid:px4_ug#mw_e3a8839f-ac1c-401c-8e25-fb70f0636df8 Deployment and Verification Using PX4 Host Target and jMAVSim> and <docid:px4_ref#mw_81df3540-d3b2-4e98-b574-af303a7fd41e>. 
%
% * For more information on designing the controller model and verifying it using the simulator plant model designed in Simulink, 
% see <docid:px4_ug#mw_19d9a55f-a164-4ed1-813f-3c5169a4dc6e>.
%  
%% Model
% To get started, launch the Simulink
% <matlab:openExample('px4/MonitorTunePX4HostTargetFlightPlantModelExample') Px4DemoHostTargetWithSimulinkPlant> project.
%
% Once the project launches, it will load the required workspace variables
% and open the top model.
%                                                        
% <<../px_sitl_project1.png>>
%
%% Model Architecture and Conventions
% This project consists of the following models:
%
% * The top model, |QuadcopterSimulation|, consists of multiple subsystems and model references.
%
% * The Controller subsystem contains the |FlightController| model reference, which contains the position and attitude controller. 
% The controller designed in this example follows the same structure as described in <docid:px4_ref#mw_81df3540-d3b2-4e98-b574-af303a7fd41e>.
%
% * The Plant and Visualization subsystem contains the model reference |Quad_Plant_dynamics| model reference, which has the Quadcopter plant and sensor dynamics.
%
% * The |px4Demo_FlightController_top| is the harness model that contains the |FlightController| model reference. The model is set up to run with the PX4 source code by creating a PX4 Host Target executable.
%
% * The |Quad_Plant_top| harness model contains the |Quad_Plant_dynamics| model reference. This harness model is set up to run in normal simulation pacing mode in lockstep with the |px4Demo_FlightController_top| harness model.
%
% The Project shortcuts guide you through the three tasks as you progress
% through the example.
% 
% <<../px_sitl_project_shortcuts.png>>
%
%% Task 1: Simulate the Drone
%
% *1.* Open the example <matlab:openExample('px4/MonitorTunePX4HostTargetFlightPlantModelExample') Px4DemoHostTargetWithSimulinkPlant> project.
%
% The |QuadcopterSimulation| model opens when the project starts. You can also open the model by clicking |Run Quadcopter full simulation| in the Projects Shortcut tab.
%
% *2.* Navigate through the different subsystems to learn about the model hierarchy and quadcopter dynamics.
%
% *3.* The Controller and the Plant and Visualization subsystems
% exchange the  HIL_ACTUATOR_CONTROLS, HIL_SENSOR, HIL_GPS, and HIL_STATE_QUATERNION MAVlink messages as described in <docid:px4_ug#mw_19d9a55f-a164-4ed1-813f-3c5169a4dc6e>.
%
% *4.* The Commands subsystem provides the desired X, Y, Z coordinates and the yaw values for the quadcopter.
%
% <<../px_sitl_commands_subsystem.png>>
%                                                                               
% *5.* To simulate the model, go to the *Simulation* tab in the Simulink model window and click *Run*. The lower-left corner of the model window displays status while Simulink prepares to run the model on the host computer.
%
% *6.* Observe that the quadcopter hovers at an altitude commanded by the Dashboard slider block.
%
% *7.* Click the *Circular Guidance* toggle switch to observe the quadcopter following a circular trajectory.
%
% <<../px_sitl_circular_guidancepng.png>>
%                                                
% *8.* You can also bring up the UAV Animation window by clicking on the *Show
% animation* button on the UAV Animation block. To locate the block, go to Plant and Visualization > Quad_Plant_Dynamics > Visualization Subsystem.
%
% *9.* Note that the controller gains are optimized for close tracking of a circular trajectory. It may be possible that for other types of input commands 
% such as step or ramp, the response is not ideal. In such a scenario, the
% controller gains must be adjusted to meet the specified requirements. For
% more details on the controller tuning, see
% <docid:px4_ref#mw_ab1310a9-a220-4cf5-9c0b-01ffe538b8fb>.
%
% In this task, we designed the multicopter plant and controller, and ensured that its performance is satisfactory in simulation.
%
%% Task 2: Deploy Controller as Host Target and Run Plant Model in Simulink
% In this task, we use the controller designed in Task 1 and run it with
% PX4 source code using the PX4 Host Target feature (for more information,
% see <docid:px4_ug#mw_19d9a55f-a164-4ed1-813f-3c5169a4dc6e>). Use two separate instances of MATLAB to run controller and plant model. 
%
% *Step 1: Launch Controller Model*
%
% *1.* Click the _Monitor and Tune Host Target controller_ in the Project Shortcuts tab to open the |px4Demo_FlightController_top| model.
%
% <<../px_sitl_flightcontroller_top.png>>
%        
% The model block references the |FlightController| model, which was used in the |QuadcopterSimulation| model.
%
% The uORB Read blocks are used to subscribe to the |vehicle_local_position| and |vehicle_attitude| topics. 
% These topics contain data sent by the simulator plant model using HIL_STATE_QUATERNION and HIL_GPS messages.
%
% The Signal Conditioning subsystem inside the FlightController model reference extracts the current position, current velocity, and 
% current attitude data and feeds it to the Controller subsystem. The Controller subsystem designs the Rate controller and the Position and Attitude controller as explained in <docid:px4_ref#mw_81df3540-d3b2-4e98-b574-af303a7fd41e>.
% 
% The |FlightController| model outputs the actuator values that are then fed to the PX4 PWM Output block.
%
% *2.* Copy the MATLAB Project Path to clipboard. 
%
% <<../px4_project_path.png>>
% 
% *Step 2: Launch Simulink Plant model in second MATLAB*
%
% *1.* Open the second instance of the same MATLAB version. 
%
% *2.* Navigate to the project path previously copied in current MATLAB. 
%
% <<../px4_path_copied.png>>
% 
% *3.* Click on the |.prj| file to launch the same Project in current MATLAB.
%
% <<../px4_open_prj.png>>
% 
% *4.* Click |Run quadcopter plant in Normal simulation| in the Project
% Shortcuts tab to open the |Quad_Plant_top| model.
%
% <<../px4_quadmodel.png>>
%
% The model block references the |Quad_Plant_dynamics| model, which was
% used in |QuadcopterSimulation|.
%
% Ensure that the *Simulation Pacing* option for this model is enabled, as described in <docid:px4_ug#mw_19d9a55f-a164-4ed1-813f-3c5169a4dc6e>.
%
% The |TCP read from PX4 Host Target| MATLAB System block is used to read the MAVLink data sent from the |px4Demo_FlightController_top| model. 
%
% The Enabled subsystem has the MAVLink Deserializer block that extracts the HIL_ACTUATOR_CONTROLS message.
%
% <<../px_sitl_enabled_ss.png>>
% 
% The |TCP write to PX4 Host Target| MATLAB System block sends the HIL_SENSOR, HIL_GPS, and HIL_STATE_QUATERNION MAVLink messages to the |px4Demo_FlightController_top| model.
%
% *Step 3: Deploy controller model over Monitor & Tune and run Plant
% Simulation*
%
% *1.* In Configuration parameters > Hardware Implementation, set the *Hardware board* parameter to |PX4 Host Target|.
%
% *2.* Under *Target hardware resources* > *Build Options*, set *Simulator* to |Simulink|.
%
% *3.* In the *Simulation* tab, set the Simulation *Stop time* to _inf_.
%
% *4.* On the *Hardware* tab, in the *Mode* section, select *Run on board* and then click *Monitor & Tune* to start signal monitoring and parameter tuning.
%
% *5.* Wait for Simulink to complete the code generation. The following dialog box appears. *Do not click OK yet*.   
%
% <<../px_sitl_startplant.png>>
% 
% In the plant model launched in second MATLAB, follow the below steps.
%
% *1.* In the *Simulation* tab, set the Simulation *Stop Time* to _inf_.
%
% *2.* Click *Run* on the *Simulation* Tab.
%
% <<../px_sitl_run.png>>
%  
% After the simulation starts in the plant model, click *OK* in the dialog box of the first model. 
%
