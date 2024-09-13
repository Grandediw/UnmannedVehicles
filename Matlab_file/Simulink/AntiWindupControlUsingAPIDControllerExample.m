%% Anti-Windup Control Using PID Controller Block
%
% This example shows how to use anti-windup schemes to prevent integration
% wind-up in PID controllers when the actuators are saturated. The PID
% Controller block in Simulink(R) features two built-in anti-windup
% methods, |back-calculation| and |clamping|, as well as a tracking mode to
% handle more complex industrial scenarios. The PID Controller block
% supports several features that allow it to handle controller windup
% issues under commonly encountered industrial scenarios.

% Copyright 2009-2022 The MathWorks, Inc.

%%
% The plant to be controlled is a saturated first-order process with
% dead-time.

open_system('sldemo_antiwindup')

%%
% The PID Controller block has been tuned with saturation ignored using the
% Simulink(R) Control Design(TM) PID tuner.

%%
% The controlled plant is a first-order process with dead-time described
% by
%
% $$P(s)=\frac{1}{10s+1}e^{-2s}$.
% 
% The plant has known input saturation limits of |[-10, 10]|, which are
% accounted for in the Saturation block labeled |Plant Actuator|. The PID
% Controller block in Simulink features two built-in anti-windup methods
% that allow it to account for the available information about the plant
% input saturation.

%% Performance Without Using Anti-Windup
% First, examine the effect of saturation on the closed-loop when the
% saturation model is not considered by the PID Controller block.
% Simulating the model generates these results. The figure shows the setpoint
% versus measured output with no anti-windup.

open_system('sldemo_antiwindup/Scope');
open_system('sldemo_antiwindup/Scope1'); % This ensures legends get displayed
simout1= sim('sldemo_antiwindup','ReturnWorkspaceOutputs','on');
close_system('sldemo_antiwindup/Scope1');

%%
% The figure shows the controller output and saturated input with no
% anti-windup.

close_system('sldemo_antiwindup/Scope');
open_system('sldemo_antiwindup/Scope1');

%%
% These figures highlight two problems with controlling a system with input
% saturation:
%
% # When the setpoint value is |10|, the PID control signal reaches a
% steady-state at about |24|, outside the range of the actuator. The
% controller is therefore operating in a nonlinear region where increasing
% the control signal has no effect on the system output, a condition known
% as _winding up_. Note that the DC-gain of the plant is unity. Therefore,
% the controller output does not need to have a steady-state value outside
% the range of the actuator.
% # When the setpoint value becomes |5|, there is a considerable delay before
% the PID controller output returns to within the actuator range.
%
% Designing the PID controller to account for the effect of saturation
% improves its performance by allowing it to operate in the linear region
% most of the time and recover quickly from nonlinearity. You can use anti-windup
% mechanism to achieve this.

%% Configure Block for Anti-Windup Based on Back-Calculation
% The back-calculation anti-windup method uses a feedback loop to unwind
% the PID Controller block internal integrator when the controller hits
% specified saturation limits and enters nonlinear operation. To enable
% anti-windup, go to the *Output Saturation* tab in the block dialog.
% Select *Limit output* and enter the saturation limits for the plant.
% Next, from the *Anti-windup method* list, select
% |back-calculation|. Then, specify the
% *Back-calculation coefficient (Kb)*. The inverse of this gain is the time
% constant of the anti-windup loop. In this example, the back-calculation
% gain is chosen to be |1|. For more information on how to choose this value,
% see [1].
% 
% <<../blkDlgAntiWindup.png>>
%
set_param('sldemo_antiwindup/PID Controller','LimitOutput','on',...
    'UpperSaturationLimit','10','LowerSaturationLimit','-10',...
    'AntiWindupMode','back-calculation');
%%
% Once back-calculation is enabled, the block has an internal tracking loop
% that unwinds the Integrator output. This figure shows the under-mask
% view of the PID Controller block with back-calculation.

open_system('sldemo_antiwindup/PID Controller','force');
%%
% Note how quickly the PID control signal returns to the linear region and
% how fast the loop recovers from saturation.

close_system('sldemo_antiwindup/PID Controller');
close_system('sldemo_antiwindup/Scope1');
open_system('sldemo_antiwindup/Scope'); % To force data to plot on scope
simout2= sim('sldemo_antiwindup','ReturnWorkspaceOutputs','on');
close_system('sldemo_antiwindup/Scope1');

%%
% The controller output |u(t)| and the saturated input |SAT(u)| coincide with
% each other because *Limit output* is enabled.

%%
close_system('sldemo_antiwindup/Scope');
open_system('sldemo_antiwindup/Scope1');

%%
% To better visualize the effect of anti-windup, this figure illustrates the
% plant measured output |y(t)| with and without anti-windup.

close_system('sldemo_antiwindup/Scope1');
t1 = get(simout1,'tout');  y1 = get(simout1,'yout');
t2 = get(simout2,'tout');  y2 = get(simout2,'yout');
figure('Tag','sldemo_antiwindup');
plot(t1,y1,t2,y2);axis([0 t1(end) round(min([y1;y2])-2) round(max([y1;y2])+2)]);
title('Measured output');
legend('Without anti-windup','With anti-windup');

%% Configure Block for Anti-Windup Based on Integrator Clamping
% Another common anti-windup strategy is based on conditional integration.
% To enable anti-windup, in the Block Parameters dialog box, select the
% *PID Advanced* tab. Select *Limit output* and enter the saturation limits
% for the plant. Then, from the *Anti-windup method* list, select
% *clamping*.
%
% This figure shows setpoint versus measured output with clamping.

open_system('sldemo_antiwindup/Scope');
open_system('sldemo_antiwindup/Scope1'); % To force data to plot on scope
set_param('sldemo_antiwindup/PID Controller','AntiWindupMode','clamping');
sim('sldemo_antiwindup');
close_system('sldemo_antiwindup/Scope1');

%%
% This figure shows that the controller output |u(t)| and the saturated
% input |SAT(u)| coincide with each other because *Limit output* is
% enabled.

close_system('sldemo_antiwindup/Scope');
open_system('sldemo_antiwindup/Scope1');

%%
bdclose('sldemo_antiwindup')
close(findobj('Type','figure','Tag','sldemo_antiwindup'))

%%
% For more information on when to use clamping, see [1].

%% Use Tracking Mode to Handle Complex Anti-Windup Scenarios
% The anti-windup strategies discussed so far rely on built-in methods to
% process the saturation information provided to the block via its dialog.
% For those built-in techniques to work as intended, two conditions must be
% met:
%
% # The saturation limits of the plant are known and can be entered into the
% dialog of the block.
% # The PID Controller block output signal is the only signal feeding the
% actuator.
%
% These conditions may be restrictive when handling general anti-windup
% scenarios. The PID Controller block features a tracking mode that allows
% you to set up a back-calculation anti-windup loop externally. The next
% two examples show the use of tracking mode for
% anti-windup purposes:
%
% # Anti-windup for saturated actuators with cascaded dynamics
% # Anti-windup for PID control with feedforward


%% Construct Anti-Windup Scheme for Saturated Actuators with Cascaded Dynamics
% The actuator in |sldemo_antiwindupactuator| has complex dynamics. Complex
% dynamics are common when an actuator has its own closed-loop dynamics.
% The PID controller is in an outer loop and sees the actuator dynamics as
% an inner loop, which is also called cascaded saturated dynamics.

open_system('sldemo_antiwindupactuator');

%%
% A successful anti-windup strategy requires feeding back the actuator
% output to the tracking port of the PID Controller block. To configure the
% |tracking mode| of the PID Controller block, in the block Parameters
% dialog box, click the *PID Advanced* tab. Select
% *Enable tracking mode* and specify the gain |Kt|. The inverse of this
% gain is the time constant of the tracking loop. For more information on
% how to choose this gain, see [1].

%%
% The measured output of the plant |y(t)| and the controller output |u(t)|
% respond almost immediately to changes in the setpoint. Without the
% anti-windup mechanism, these responses have long delays.

open_system('sldemo_antiwindupactuator/Scope');
open_system('sldemo_antiwindupactuator/Scope1'); % This ensures legends get displayed
sim('sldemo_antiwindupactuator');
close_system('sldemo_antiwindupactuator/Scope1');

%%
close_system('sldemo_antiwindupactuator/Scope');
open_system('sldemo_antiwindupactuator/Scope1');

%%
bdclose('sldemo_antiwindupactuator')

%% Anti-Windup Scheme for PID Control with Feedforward
% In another common control configuration, the actuator receives a control
% signal that is a combination of a PID control signal and a feedforward
% control signal. Open the model |sldemo_antiwindupfeedforward|.

%%
% To accurately build a back-calculation anti-windup loop, the tracking
% signal should subtract the contribution of the feedforward signal. This
% action allows the PID Controller block to know its share of the effective
% control signal applied to the actuator.

open_system('sldemo_antiwindupfeedforward');

%%
% The feedforward gain is unity here because the plant has a DC-gain of
% |1|.

%%
% The measured output of the plant |y(t)| and the controller output |u(t)|
% respond almost immediately to changes in the setpoint. When the
% setpoint value is |10| , note how the controller output |u(t)|
% reduces to be within the range of the actuator.

open_system('sldemo_antiwindupfeedforward/Scope');
open_system('sldemo_antiwindupfeedforward/Scope1'); % This ensures legends get displayed
sim('sldemo_antiwindupfeedforward');
close_system('sldemo_antiwindupfeedforward/Scope1');
  
%%
% When the setpoint value is |10| , note how the controller output |u(t)| 
% reduces to be within the range of the actuator.
%%
close_system('sldemo_antiwindupfeedforward/Scope');
open_system('sldemo_antiwindupfeedforward/Scope1');

%%
% This figure shows the PID Controller output and feed forward input with anti-windup.
%%
close_system('sldemo_antiwindupfeedforward/Scope1');
open_system('sldemo_antiwindupfeedforward/Scope2');
