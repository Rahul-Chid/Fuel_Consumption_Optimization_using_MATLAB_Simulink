% ----------------------------------------------------------------------------
%                                       GROUP 8
%
%                   Seminar Electromobility_Summer Semester_2023
%
%      Energy Management of Parallel Mild Hybrid Electric Vehicle using Rule Based Strategy
%
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

% Function to Calculate the Torque Split Factor
function u = Controller_DRS_SS_23(input)

    % Input extraction
    w_MGB   = input(1);   % Flywheel angular velocity
    dw_MGB  = input(2);   % Flywheel angular acceleration
    T_MGB   = input(3);   % Flywheel torque
    Q_BT    = input(4);   % Current battery charge

    % Global variables
    global w_EM_max;      % Maximum motor angular velocity
    global T_EM_max;      % Maximum motor torque 
    global Q_BT_IC;       % Initial battery charge

    % Constants definitions
    theta_EM    = 0.1;    % Motor inertia
    epsilon     = 0.01;   % Small value for approximation
    u_LPS_max   = 0.06;    % Maximum torque-split factor for Load Point Shifting (LPS)
    SOC_th      = 0.37;   % Threshold value for State of Charge

    % Stop-time of the cycle calculation
    current_system = gcs;
    stop_time = get_param(current_system, 'StopTime');
    stop_time = eval(stop_time);

    % Parameters setting based on cycle's stop-time
    switch stop_time
        case 1220
            % Parameters for NEDC
            T_MGB_th = 60;         % Torque threshold
            T = 29;                % Electric Drive Torque threshold
            u_LPS_min = -0.573;    % Minimum torque-split factor
            Q_BT_min = SOC_th * Q_BT_IC;
            Q_BT_max = 0.9 * Q_BT_IC;

        case 1877
            % Parameters for FTP-75
            T_MGB_th = 100;        % Torque threshold
            T = 31;                % Electric Drive Torque threshold
            u_LPS_min = -0.192;    % Minimum torque-split factor
            Q_BT_min = SOC_th * Q_BT_IC;
            Q_BT_max = 0.9 * Q_BT_IC;

        otherwise
            error('The selected Driving Cycle is not valid for this Seminar');
    end

    % Determine the control mode
    if (T_MGB < 0)
        % Regeneration mode
        u = min((interp1(w_EM_max,-T_EM_max,w_MGB)+abs(theta_EM*dw_MGB)+epsilon)/T_MGB,1);

    elseif ((T_MGB >= T_MGB_th)  &&  (Q_BT > Q_BT_max))
        % Load Point Shifting in Motor mode
        u = min((interp1(w_EM_max,T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,u_LPS_max);

    elseif ((T_MGB > 0) && (T_MGB < T) && (Q_BT >= Q_BT_min))
        % Electric Drive mode
        u = 1;

    elseif ((T_MGB > T) && (T_MGB < T_MGB_th))
        % Load Point Shifting in Generator mode
        u = max((interp1(w_EM_max,-T_EM_max,w_MGB)+abs(theta_EM*dw_MGB)+epsilon)/T_MGB,u_LPS_min);

    else
        % Engine mode
        u = 0;
    end
end
