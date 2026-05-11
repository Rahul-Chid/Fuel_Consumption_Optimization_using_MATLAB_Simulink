% ----------------------------------------------------------------------------
%                                       GROUP 8
%
%                   Seminar Electromobility_Summer Semester_2023
%
%       Optimization of Fuel Consumption of Parallel HEV using Dynamic Programming
%                                       (FTP-75)
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

%%                  Dynamic Programming Optimization for FTP-75 Driving Cycle

% Start stopwatch timer to measure the execution time
tic;  

% Load the driving cycle data
load FTP-75_inputs.mat

% Defining the SOC and split ratio grid for the problem
clear grd
grd.Nx{1}    = 101;         % Define the number of discrete values for SOC
grd.Xn{1}.hi = 0.95;        % Set SOC upper bound
grd.Xn{1}.lo = 0.15;        % Set SOC lower bound

grd.Nu{1}    = 10001;       % Define the number of discrete values for split ratio
grd.Un{1}.hi = 1;           % Set split-ratio upper bound
grd.Un{1}.lo = -1;          % Set split-ratio lower bound

% Set initial state of SOC
grd.X0{1} = 0.50;           % Initial SOC 

% Constraints for the final state of SOC
grd.XN{1}.hi = 0.51;        % Define the final SOC range 
grd.XN{1}.lo = 0.501;       % Define the final SOC range

% Setting up optimization parameters
clear prb

prb.W{1} = w_MGB_FTP';      % Angular speed of MGB for NEDC
prb.W{2} = T_MGB_FTP';      % Torque of MGB for NEDC
prb.W{3} = dw_MGB_FTP';     % Angular acceleration of MGB for NEDC

prb.Ts = 1;                 % Define the time-step
prb.N  = 1877*1/prb.Ts + 1; % Define the size of the cycle

% Configure options for Dynamic Programming Method
options = dpm();
options.MyInf = 1e57;
options.BoundaryMethod = 'Line';    % Use 'Line' as boundary condition.
if strcmp(options.BoundaryMethod,'Line') 
    % Below options are applicable only if 'Line' is used
    options.Iter = 9;
    options.Tol = 1e-8;
    options.FixedGrid = 0;
end

% Execute the Dynamic Programming Method
[res, dyn] = dpm(@DP_FTP_75,[],grd,prb,options);

% Calculate and print the total execution time
toc;
run_time = toc - tic;       % Execution time of the program

% --- Save the NEDC cycle results section ---
save FTP-75_outputs.mat res dyn

%% -------Execute this section to visualize the results in Simulink--------
%   Compute Torque split ratio section

load FTP-75_outputs.mat
load FTP-75_inputs.mat

% Compute the Split-ratio
u = res.Tm./T_MGB_FTP';
u(isnan(u)) = 0;            % If torque is zero, set 'u' to zero   

% Save the computed split ratio
save torque_split_FTP_75.mat u

% The following parameters help to feed the split-ratio in 
% the qss_hybrid_electric_vehicle_example.mdl file
u_in = u;
time = 0:1:1877; 
table1 = table(time', u_in');
file_name= 'torque_split_FTP_75.xlsx';
writetable(table1, file_name)
save torque_split_FTP_with_time.mat u_in time;  % Store split-ratio with corresponding time
