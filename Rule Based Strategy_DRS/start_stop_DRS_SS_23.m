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
%      Matriculation Number: xxxxxx
%      Email: xxx@rhrk.uni-kl.de
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

% Function to Determine the State of Combustion Engine
function state_CE = start_stop_DRS_SS_23(input)

    % Extract inputs
    T_MGB = input(1);    % Flywheel Torque
    u     = input(2);    % Torque-split factor

    % Calculation of state of Combustion Engine
    if (T_MGB < 0) || (u == 1)
        % If the Flywheel Torque is less than 0 or the Torque-split factor equals 1, 
        % the state of the Combustion Engine is 0 (OFF)
        state_CE = 0;
    else
        % Otherwise, the state of the Combustion Engine is 1 (ON)
        state_CE = 1;
    end
end
