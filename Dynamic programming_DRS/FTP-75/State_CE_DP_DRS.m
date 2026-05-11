function State_CE = State_CE_DP_DRS(input)
%'Engine_on_off' function defines the engine state (Either on and off)

% assigning the parameters separately
u      = input(1);       % Torque of MGB
T_MGB  = input(2);       % Split-ratio from the dynamic programming

if ((T_MGB<0) && (u==1))
    %Engine state off, i.e., for Electric drive and Regenrative braking
    State_CE = 0;
else
    %Engine state on, i.e., LPS in motor and generator mode and engine drive
    State_CE = 1;
end

%function ends
end