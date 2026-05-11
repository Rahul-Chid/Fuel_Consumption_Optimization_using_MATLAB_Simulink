% ----------------------------------------------------------------------------
%                                       GROUP 8
%
%                   Seminar Electromobility_Summer Semester_2023
%
%       Optimization of Fuel Consumption of Parallel HEV using Dynamic Programming
%                                       (NEDC)
%                           ------------------------------
%                                   Team Members               
%                           ------------------------------
% |->  Name: Danish Shahid Pathan
%      Matriculation Number: 425861
%      Email: kat22wed@rhrk.uni-kl.de
%
% |->  Name: Rahul Chidambaranathan
%      Matriculation Number: 425807
%      Email: tet21geh@rhrk.uni-kl.de
%
% |->  Name: Srinivas Shanmuga Sundaram
%      Matriculation Number: 425749
%      Email: paw81ner@rhrk.uni-kl.de
% ------------------------------
%
% ----------------------------------------------------------------------------


function [X, C, I, out] = DP_NEDC(inp,~)
% DP_NEDC Computes the resulting state-of-charge based on current state-
% of-charge, inputs and drive cycle demand.
% 
% Function Signature:
% [X, C, I, out] = DP_NEDC(INP,PAR)
%
% Inputs:
% INP   = input structure
% PAR   = user defined parameters
%
% Outputs:
% X     = resulting state-of-charge (battery charge percentage)
% C     = cost matrix (fuel consumption)
% I     = infeasible matrix (system infeasibility)
% out   = user defined output signals (Torque of engine and motor)

%% Initialization
% Assigning the individual speed and acceleration values
wg = inp.W{1};    % Engine speed
Ttot = inp.W{2};  % Total Torque          

% Engine Initialization
load OM_622.mat;                % Load the engine parameter
w_MGB = wg;                     % Assign Engine angular velocity
dw_MGB = inp.W{3};              % Assign Engine angular acceleration
theta_CE = 0.2;                 % Constant value (do not change)
w_CE_idle = 105;                % Engine Idle speed
T_CE_cutoff = 5;                % Engine Cutoff Torque
w_CE_upper = max(w_CE_max);     % Maximum Engine speed

% Electric Motor Initialization
load EM;                        % Load motor parameters
P_aux = 0;                      % Auxiliary power
theta_EM = 0.1;                 % Constant value (do not change)
w_EM_upper = max(w_EM_max);     % Maximum Motor speed

% Battery Initialization
load BT;                        % Load battery parameters          
Q_BT_0      = I_0 * 3600;       % Discharge current in 1 hour in Coulombs
U_BT_0      = c_BT_E1+c_BT_E3;  % Mean battery voltage in Volts
I_BT_max    = (60/t_ch)*I_0;    % Maximum charge/discharge current in Ampere
Q_BT_IC = inp.X{1}*3600*I_0;    % Initial battery charge in Coulombs

% Dynamic parameters initialization
u1 =(Ttot>0).*(inp.U{1}<1);     % Engine start-stop status
u2 = inp.U{1};                  % Split ratio


%% Load Simulink blocks
load_system('torque_split')  % Load the simulink model 'torque_split'
configObj = Simulink.ConfigSet; % Create a ConfigSet object
set_param(configObj,'StartTime','0') % Set simulation start time
set_param(configObj,'StopTime',num2str(size(Ttot,1))) % Set simulation stop time
set_param(configObj, 'SolverType', 'Fixed-step') % Set solver type as Fixed-step
set_param(configObj,'FixedStep', '1') % Set solver step size

%% Propagation
% Torque split calculations are done in the simulink model 'torque_split'
Ttot_in = Ttot; % Total torque input for simulink model

% Fixed grid parameters (don't change)
m=101;              % Number of states (grd.Nx{1})
n=20001;            % Number of control inputs (grd.Nu{1})


% Different logic for backward calculation and boundary conditions/forward propagation
if (size(u1,1)~=1)
    % Execute for backward calculation to reduce computational time

    % Controller: reshape input matrices into 1D for fewer simulink calls
    u1_in =  reshape(u1, 1,[]);  
    u2_in = reshape(u2, 1, []);  
    Q_BT_in = reshape(Q_BT_IC, 1, []);

    % Run simulation
    outputs = sim('torque_split', 'SrcWorkspace','current');

    % Reshape output variables back to 2D
    Te_out = reshape(outputs.Te_out(1,:), [m n]); % Engine torque
    Tm_out = reshape(outputs.Tm_out(1,:), [m n]); % Motor torque
    Pe_out = reshape(outputs.Pe_out(1,:), [m n]); % Fuel consumption
    I_BT_out = reshape(outputs.I_BT(1,:), [m n]); % Battery output current

    % Calculate infeasibility
    inps = reshape(~((~((Ttot_in>0)&(u1_in==0))|(Tm_out==Ttot_in))&(~(Ttot_in<=0)|(Tm_out>=(Ttot_in)))), [m n]);
    ine = reshape((outputs.I2(1,:) | outputs.I3(1,:)), [m n]);
    inm = reshape((outputs.I4(1,:) | outputs.I5(1,:)), [m n]);
    inb = reshape((outputs.I6(1,:) | outputs.I7(1,:)), [m n]);
else
    % Execute for boundary conditions check and forward propagation

    % Controller: 1D inputs, no need to reshape
    u1_in = u1;
    u2_in = u2;
    Q_BT_in = Q_BT_IC;

    % Run simulation    
    outputs = sim('torque_split', 'SrcWorkspace','current');

    % Output variables
    Te_out = outputs.Te_out(1,:); % Engine torque
    Tm_out = outputs.Tm_out(1,:); % Motor torque
    Pe_out = outputs.Pe_out(1,:); % Fuel consumption
    I_BT_out = outputs.I_BT(1,:); % Battery output current

    % Calculate infeasibility
    inps = ~((~((Ttot_in>0)&(u1_in==0))|(Tm_out==Ttot_in))&(~(Ttot_in<=0)|(Tm_out>=(Ttot_in))));
    ine = (outputs.I2(1,:) | outputs.I3(1,:)); 
    inm = (outputs.I4(1,:) | outputs.I5(1,:));
    inb = (outputs.I6(1,:) | outputs.I7(1,:));
end

% Infeasible matrix (combines infeasibility from controller, engine, motor, and battery)
I = (inps+inb+ine+inm~=0);

% Update State variable (battery state of charge)
X{1} = (Q_BT_IC - I_BT_out)/(3600*I_0);
X{1} = (conj(X{1})+X{1})/2;

% Cost matrix (fuel consumption)
C{1}  = Pe_out;

% If system is infeasible, stop and debug
if numel(find(I==0))==0
    keyboard
end

% User defined output signals
out.Te = Te_out; % Engine torque
out.Tm = Tm_out; % Motor torque
end
