function t_total = CalculateMotionTime(xps, Group, StartPos, EndPos, Vel, Acc)
% CalculateMotionTime - Estimates motion time between two positions.
% Inputs:
%   xps      : XPSD_NET object (connected to controller).
%   Group    : Axis group name (e.g., 'Group1').
%   StartPos : Starting position (units as configured).
%   EndPos   : Target position (units as configured).
%   Vel      : (Optional) Override velocity (units/s).
%   Acc      : (Optional) Override acceleration (units/s²).
% Output:
%   t_total  : Estimated motion time (seconds).

    % Retrieve velocity/acceleration if not provided
    if nargin < 5 || isempty(Vel) || isempty(Acc)
        [Vel, Acc, ~, ~] = xps.GetMoveParameters(Group);
    end

    % Total distance
    D = abs(EndPos - StartPos);
    
    % Time to accelerate/decelerate
    t_acc = Vel / Acc;                     % Acceleration time
    D_acc = 0.5 * Acc * t_acc^2;          % Distance during acceleration
    
    % Total time calculation
    if D > 2 * D_acc
        % Trapezoidal profile (accel + constant velocity + decel)
        t_total = t_acc + (D - 2 * D_acc) / Vel + t_acc;
    else
        % Triangular profile (accel + decel, no constant velocity)
        t_total = 2 * sqrt(D / Acc);
    end
end