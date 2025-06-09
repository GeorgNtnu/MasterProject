% Laster in data                                |Place from first
% element|The catual radius
% Radius
Data_1 = load('Data_1,3GHz.mat');  fc = 1.3e9; % 0.53 %  6.26
Data_2 = load('Data_3GHz.mat');   % fc = 3e9;    % 3.25 %  8.98
Data_3 = load('Data_6GHz.mat');   % fc = 6e6;    % 8.23 % 13.96

% theta = linspace(-90, 90, 1000).';

theta = Data_1.Angles.';
Data  = Data_1.LoS_array;

[Tx,Rx1,Rx2] = Calculate_PoS(theta);

% Calculate vectors
v1 = Rx1 - Tx;
v2 = Rx2 - Tx;
v3 = Rx2 - Rx1;
theta_deg1 = Calc_DoA(v3,v1);
theta_deg2 = Calc_DoA(v3,v2);

d1 = sqrt(v1(:, 1).^2 + v1(:, 2).^2);
Corrected_Data1 = Data(:, 1) .* 4*pi.*d1*fc/3e8 .* exp(1j*2*pi.*d1*fc/3e8); 
d2 = sqrt(v2(:, 1).^2 + v2(:, 2).^2);
Corrected_Data2 = Data(:, 2) .* 4*pi.*d2*fc/3e8 .* exp(1j*2*pi.*d2*fc/3e8); 



% theta_common = linspace(min([theta_deg1; theta_deg2]), max([theta_deg1; theta_deg2]), 10000);
% Corrected_Data1 = interp1(theta_deg1, Corrected_Data1, theta_common, 'spline', 'extrap');
% Corrected_Data2 = interp1(theta_deg2, Corrected_Data2, theta_common, 'spline', 'extrap');
% theta_deg1 = theta_common; theta_deg2 = theta_common;

Corrected_Data = [Corrected_Data1, Corrected_Data2];
theta_deg      = [theta_deg1, theta_deg2];

figure(1);
    subplot(2, 1, 1);
    plot(theta_deg1, abs(Corrected_Data1)); hold on;
    plot(theta_deg2, abs(Corrected_Data2)); hold off;
    legend({'LPDA_1', 'LPDA_2'});
    title('Amplitude');
    ylabel('$|A|$', 'Interpreter', 'latex');
    xlabel('Angle DoA (deg)');

    subplot(2, 1, 2);
    plot(theta_deg1, unwrap(angle(Corrected_Data1))); hold on;
    plot(theta_deg2, unwrap(angle(Corrected_Data2))); hold off;
    legend({'LPDA_1', 'LPDA_2'});
    title('Phase Unwrapped');
    ylabel('$\phi$', 'Interpreter', 'latex');
    xlabel('Angle DoA (deg)');

figure(2);
    subplot(2, 1, 1);
    plot(theta_deg, abs(Corrected_Data(:, :)./ Corrected_Data(:, 1)))
    legend({'LPDA_1', 'LPDA_2'});
    title('Amplitude Offset');
    ylabel('$\Delta |A|$', 'Interpreter', 'latex');
    xlabel('Angle DoA (deg)');

    
    subplot(2, 1, 2);
    plot(theta_deg, (angle(Corrected_Data(:, :)./ Corrected_Data(:, 1))))
    legend({'LPDA_1', 'LPDA_2'});
    title('Phase Offset');
    ylabel('$\Delta \phi$', 'Interpreter', 'latex');
    xlabel('Angle DoA (deg)');

%% Updated Plot in dB



figure(1);
subplot(2, 1, 1);
plot(theta_deg1, 20*log10(abs(Corrected_Data1))); hold on;
plot(theta_deg2, 20*log10(abs(Corrected_Data2))); hold off;
legend({'LPDA_1', 'LPDA_2'});
title('Amplitude (dB)');
ylabel('$|G(r) \cdot \epsilon|$ (dB)', 'Interpreter', 'latex');
xlabel('Angle DoA (deg)');
grid on;

subplot(2, 1, 2);
plot(theta_deg1, unwrap(angle(Corrected_Data1))); hold on;
plot(theta_deg2, unwrap(angle(Corrected_Data2))); hold off;
legend({'LPDA_1', 'LPDA_2'});
title('Phase Unwrapped');
ylabel('$\angle G(r) \cdot \epsilon$', 'Interpreter', 'latex');
xlabel('Angle DoA (deg)');
grid on;

figure(2);
subplot(2, 1, 1);
plot(theta_deg, 20*log10(abs(Corrected_Data(:, :)./ Corrected_Data(:, 1))));
legend({'LPDA_1', 'LPDA_2'});
title('Amplitude Offset (dB)');
ylabel('$\Delta |\epsilon|$ (dB)', 'Interpreter', 'latex');
xlabel('Angle DoA (deg)');
grid on;

subplot(2, 1, 2);
plot(theta_deg, angle(Corrected_Data(:, :)./ Corrected_Data(:, 1)));
legend({'LPDA_1', 'LPDA_2'});
title('Phase Offset');
ylabel('$\Delta \angle \epsilon$', 'Interpreter', 'latex');
xlabel('Angle DoA (deg)');
grid on;

%% Function


function theta_deg = Calc_DoA(v2,v1)
    % Find the perpendicular vector to v2
    v2_perp = [-v2(:, 2), v2(:, 1)];
    
    % Calculate the angle using atan2 for each pair of vectors
    theta_rad = atan2(v1(:, 2), v1(:, 1)) - atan2(v2_perp(:, 2), v2_perp(:, 1));
    
    % Convert the angle to degrees
    theta_deg = rad2deg(theta_rad);
    theta_deg = wrapTo180(wrapTo180(theta_deg)*-1 - 180);
end

function [Tx,Rx1,Rx2] = Calculate_PoS(theta)
    Tx = [0, 0];
    % Radius has to be changed to:
    % 6.26e-2 for 1.3GHz
    % 8.98e-2; for 3GHz
    % 13.96e-2 for 6GHz 

    Radius =  6.26e-2;
    % Radius =  8.98e-2; 
    % Radius =  13.96e-2;
    distance_antennas = 15.5e-2;
    phi = rad2deg(atan(distance_antennas / Radius));
    Radius2 = norm([distance_antennas, Radius]);
    CenterOfRotation = [0, 5];
    
    % Calculate Rx1 and Rx2 for all theta values
    Rx1 = [sin(deg2rad(theta)), -cos(deg2rad(theta))] * Radius + CenterOfRotation;
    Rx2 = [sin(deg2rad(theta - phi)), -cos(deg2rad(theta - phi))] * Radius2 + CenterOfRotation;
end



