function V = CalculateVelocityForTime(xps, Group, StartPos, EndPos, DesiredTime, Acc)
% CalculateVelocityForTime - Computes velocity needed to move between positions in a specified time.
% Inputs:
%   xps          : XPSD_NET object (connected to controller).
%   Group        : Axis group name (e.g., 'Group1').
%   StartPos     : Start position (units as configured).
%   EndPos       : Target position (units as configured).
%   DesiredTime  : Total desired motion time (seconds).
%   Acc          : (Optional) Override acceleration (units/s²).
% Output:
%   V            : Required velocity (units/s).

    % Retrieve acceleration if not provided
    if nargin < 6 || isempty(Acc)
        [~, Acc, ~, ~] = xps.GetMoveParameters(Group);
    end

    % Total distance
    D = abs(EndPos - StartPos);
    
    % Minimum possible time (triangular profile)
    T_min = 2 * sqrt(D / Acc);
    
    % Check feasibility
    if DesiredTime < T_min
        error('Desired time (%.2f s) is too short. Minimum time: %.2f s.', DesiredTime, T_min);
    end
    
    % Solve quadratic equation: V² - (A*T)V + A*D = 0
    A = Acc;
    T = DesiredTime;
    discriminant = sqrt((A * T)^2 - 4 * A * D);
    
    % Possible solutions
    V1 = (A * T + discriminant) / 2;
    V2 = (A * T - discriminant) / 2;
    
    % Validate solutions (V > 0 and D > 2*D_acc)
    valid_V = [];
    for V_candidate = [V1, V2]
        if V_candidate <= 0
            continue;
        end
        D_acc = 0.5 * A * (V_candidate / A)^2; % Distance during acceleration
        if D > 2 * D_acc
            valid_V(end+1) = V_candidate;
        end
    end
    
    if isempty(valid_V)
        error('No valid velocity found.');
    else
        V = min(valid_V); % Use the smaller valid velocity
    end
end