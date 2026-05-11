%% -------Execute this section to visualize the results in Simulink--------
%   Compute Torque split ratio section

load FTP-75_outputs.mat
load FTP-75_inputs.mat

% Compute the Split-ratio
u = res.Tm./T_MGB_FTP';
u(isnan(u)) = 0;            % If torque is zero, set 'u' to zero   

% Save the computed split ratio
save split_FTP_3.mat u

% The following parameters help to feed the split-ratio in 
% the qss_hybrid_electric_vehicle_example.mdl file
u_in = u;
time = 0:1:1877; 
table1 = table(time', u_in');
file_name= 'split_done.xlsx';
writetable(table1, file_name)
save split_done.mat u_in time;  % Store split-ratio with corresponding time