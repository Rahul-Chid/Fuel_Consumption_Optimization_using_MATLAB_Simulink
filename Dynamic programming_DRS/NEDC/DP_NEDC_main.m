% ----------------------------------------------------------------------------
%                                       GROUP 8
%
%                   Seminar Electromobility_Summer Semester_2023
%
%       Optimization of Fuel Consumption of Parallel HEV using Dynamic Programming 
%                                        (NEDC)
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

%%             Dynamic Programming Optimization for NEDC Driving Cycle

% Initiating stopwatch timer to measure the execution time of the program
tic;

% Loading the necessary input data for the NEDC driving cycle
load NEDC_inputs.mat

% Setting up the grid with defined boundaries and discrete values for SOC and split ratio
clear grd
grd.Nx{1}    = 101;         % Number of discrete values for State of Charge (SOC)
grd.Xn{1}.hi = 0.95;        % Upper limit for SOC
grd.Xn{1}.lo = 0.15;        % Lower limit for SOC

grd.Nu{1}    = 20001;       % Number of discrete values for split ratio
grd.Un{1}.hi = 1;           % Upper limit for split ratio
grd.Un{1}.lo = -1;          % Lower limit for split ratio

% Specifying the initial and final SOC values
grd.X0{1} = 0.50;           % Initial SOC value
grd.XN{1}.hi = 0.51;        % Upper limit for final SOC value
grd.XN{1}.lo = 0.501;       % Lower limit for final SOC value

% Defining the optimization parameters using the loaded data
clear prb
prb.W{1} = w_MGB_NEDC';     % Angular speed of Main Gear Box (MGB) throughout the NEDC
prb.W{2} = T_MGB_NEDC';     % Torque of MGB throughout the NEDC
prb.W{3} = dw_MGB_NEDC';    % Angular acceleration of MGB throughout the NEDC

prb.Ts = 1;                 % Time-step in seconds
prb.N  = 1220*1/prb.Ts + 1; % Total number of steps in the cycle

% Configuring the options for the dynamic programming method
options = dpm();
options.MyInf = 1e57;
options.BoundaryMethod = 'Line';    % Setting the boundary method to 'Line'
if strcmp(options.BoundaryMethod,'Line') 
    % Specific options for the 'Line' boundary method
    options.Iter = 9;
    options.Tol = 1e-8;
    options.FixedGrid = 0;
end

% Calling the dynamic programming function to perform the optimization
[res, dyn] = dpm(@DP_NEDC,[],grd,prb,options);

% Stopping the stopwatch timer and calculating the total runtime
toc;
run_time = toc - tic;       % Total runtime of the program

%%           Saving the Optimization Results for NEDC Driving Cycle

% Saving the results and dynamics from the optimization into a MAT-file
save NEDC_outputs.mat res dyn

%%       Visualizing the Torque Split Ratio in Simulink

% Loading the necessary data to calculate and visualize the split ratio
load NEDC_outputs.mat
load NEDC_inputs.mat

% Calculating the split ratio using the optimized torque values
u = res.Tm./T_MGB_NEDC';
u(isnan(u)) = 0;            % Setting NaN values to zero (occurring when torque is zero)

% Saving the calculatedsplit ratio into a MAT-file
save torque_split_NEDC.mat u

% Preparing the data for visualization in Simulink
u_in = u;
time = 0:1:1220; 
table1 = table(time', u_in');
file_name= 'torque_split_NEDC.xlsx';
writetable(table1, file_name)
save torque_split_NEDC_with_time.mat u_in time;  % Saving the split ratio along with the corresponding time into a MAT-file
