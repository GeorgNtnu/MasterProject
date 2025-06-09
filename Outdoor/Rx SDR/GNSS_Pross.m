% GNSS_Pssesing
% 'C:\Users\GMELK\MATLAB Drive\GNSS\TestMappe\GNSSMesuarments.pos'
% 'C:\Users\GMELK\MATLAB Drive\GNSS\RPI4_pos\ublox_000.pos'
% Measurement = load('PostPross_Data.mat');
% LoS_Data = load("ResultFFT.mat");
[GNSS_Pos, ~, ~, ~, ~, ~, LoS_taps, Reff_Pos, indices] = MobileGNSSDataProscess(Measurement.t_saved, Measurement.Start_Time, LoS_Data.LoS_taps);
[GNSS_Pos_Static, ~, ~ , ~] = StaticGNSSDataProscess(Reff_Pos);
[Ant1, Ant2, ~] = Calc_AntennePossition(GNSS_Pos_Static);
[unit_vec,TxAnt] = CalcTxAntenna_poss(GNSS_Pos_Static,GNSS_Pos);

d1 = Ant1 - TxAnt;
d2 = Ant2 - TxAnt;

d1_length = sqrt(sum(d1.^2, 2));
d2_length = sqrt(sum(d2.^2, 2));

d = [d1_length, d2_length];
fc = 1.3e9;


LoS_taps_corr = LoS_taps.' * 4*pi.*d*fc/3e8 .* exp(1j*2*pi.*d*fc/3e8); 

Offset = LoS_taps_corr ./ LoS_taps_corr(:, 1);



%% Plot 





figure(1);
    plot3(GNSS_Pos(:, 1), GNSS_Pos(:, 2), GNSS_Pos(:, 3), '-', DisplayName='Tx GNSS'); hold on;
    plot3(GNSS_Pos_Static(:, 1), GNSS_Pos_Static(:, 2), GNSS_Pos_Static(:, 3), 'x', DisplayName='Rx GNSS');  
    title('GNSS data')
    xlabel('x (east)');
    ylabel('y (north)');
    zlabel('z (up)');
    legend; grid on;
    hold off;

% figure(2);
%     subplot(3, 1, 1);
%     plot(xEast);
% 
%     subplot(3, 1, 2);
%     plot(yNorth);
% 
%     subplot(3, 1, 3);
%     plot(zUp);


figure(3); % 2D plot
    plot(GNSS_Pos_Static(1), GNSS_Pos_Static(2), 'x', DisplayName='Rx Array GNSS'); hold on;
    plot(GNSS_Pos(:, 1), GNSS_Pos(:, 2), DisplayName='Tx GNSS');
    plot(Ant1(1), Ant1(2), '*',  DisplayName='Ant1');
    plot(Ant2(1), Ant2(2), '*',  DisplayName='Ant2');   
    % quiver(TxAnt(:,1), TxAnt(:,2), unit_vec(:,1), unit_vec(:,2),200, 'b', 'DisplayName', 'Direction Vectors');
    plot(TxAnt(:, 1), TxAnt(:, 2), 'o', MarkerSize=1, DisplayName='Tx Antenna');
    %plot(TxAnt(20000, 1), TxAnt(20000, 2), 'X')

    hold off;
    legend;

figure(4);
    subplot(2, 1, 1);
    plot(Measurement.t_saved(indices)-Measurement.t_saved(1), 20*log10(abs(Offset)));
    title('Amplitude Offset');
    ylabel('$\Delta|G\cdot\epsilon|$ (dB)', Interpreter='latex');
    xlabel('time [s]');
    legend({'LPDA_1', 'LPDA_2'})
    grid on;

    subplot(2, 1, 2);
    plot(Measurement.t_saved(indices)-Measurement.t_saved(1), (angle(Offset)), 'X');
    title('Phase Offset');
    ylabel('$\Delta\angle(G\cdot\epsilon)$', Interpreter='latex');
    xlabel('time [s]');
    legend({'LPDA_1', 'LPDA_2'})
    grid on;

figure(5);
    subplot(2, 1, 1);
    plot(Measurement.t_saved-Measurement.t_saved(1), 20*log10(abs((LoS_Data.LoS_taps))));
    title('Amplitude');
    ylabel('$|\alpha_{LoS}|$ (dB)', Interpreter='latex');
    xlabel('time [s]');
    legend({'LPDA_1', 'LPDA_2'})
    grid on;
    
    subplot(2, 1, 2);
    plot(Measurement.t_saved(1:end-1)-Measurement.t_saved(1), unwrap(angle((LoS_Data.LoS_taps(:, 1:end-1))), [], 2) );
    title('Phase');
    ylabel('$\angle\alpha_{LoS}$', Interpreter='latex');
    xlabel('time [s]');
    legend({'LPDA_1', 'LPDA_2'})
    grid on;

figure(6);
    subplot(2, 1, 1);
    plot(Measurement.t_saved(indices)-Measurement.t_saved(1), 20*log10(abs((LoS_taps_corr))));
    title('Amplitude');
    ylabel('$|G\cdot\epsilon|$ (dB)', Interpreter='latex');
    xlabel('time [s]');
    legend({'LPDA_1', 'LPDA_2'})
    grid on;
    
    subplot(2, 1, 2);
    plot(Measurement.t_saved(1:end-1)-Measurement.t_saved(1), unwrap(angle((LoS_taps_corr(1:end-1, :))), [], 1) );
    title('Phase');
    ylabel('$\angle(G\cdot\epsilon)$', Interpreter='latex');
    xlabel('time [s]');
    legend({'LPDA_1', 'LPDA_2'})
    grid on;




    

%StartTime.Format = 'HH:mm:ss.SSS';


% Plot vectors at new points
% quiver3(new_points(:,1), new_points(:,2), new_points(:,3), ...
%         unit_vecs(:,1), unit_vecs(:,2), unit_vecs(:,3), ...
%         0.1, 'b', 'DisplayName', 'Direction Vectors');


%% Functions 
function [GNSS_Pos,xEast,yNorth,zUp,Start,Stopp,LoS_taps, Reff_Pos, indices] = MobileGNSSDataProscess(t_saved, Start_Time, LoS_taps)
% GNSS_Pos is the interpolated position data
% [xEast,yNorth,zUp] is original position data 
% [Start, Stopp] is the index of [xEast,yNorth,zUp] used
% LoS_taps is the relevant taps that mached the position. 
% Reff_Pos is the refrence in the cart

% The function process the GNSS reciver data to fit with SDR samples. The
% function is hand tuned for one mesuerment and can not be used in a
% general setting, but may be tuned. 
    data = readtable('C:\Users\GMELK\MATLAB Drive\GNSS\RPI4_pos\ublox_000.pos', 'FileType', 'text');
    usefulData = data(24:end, 1:15);
    newColumnNames = {'GPST1', 'GPST2', 'latitude(deg)', 'longitude(deg)', 'height(m)', 'Q', 'ns', 'sdn(m)', 'sde(m)', 'sdu(m)', 'sdne(m)', 'sdeu(m)', 'sdun(m)', 'age(s)', 'ratio'};
    usefulData.Properties.VariableNames = newColumnNames;
    
    % Data_filterd = usefulData(usefulData.Q == 1, :);
    % ---- Må endres 
    Data_filterd = usefulData;
    Data_filterd.GPST1 = datetime(Data_filterd.GPST1, 'InputFormat', 'yyyy/MM/dd');
    combinedDateTimeStr = strcat(string(Data_filterd.GPST1), {' '}, string(Data_filterd.GPST2));
    combinedDateTime = datetime(combinedDateTimeStr, InputFormat='dd-MMM-uuuu HH:mm:ss.SSS', Locale='en_US', Format='HH:mm:ss.SSS', TimeZone='UTC');
    
    
    indices = 1:size(Data_filterd, 1);
    GNSS_Pos = [Data_filterd.("latitude(deg)")(indices), Data_filterd.("longitude(deg)")(indices), Data_filterd.("height(m)")(indices)];
    Reff_Pos =   mean(GNSS_Pos, 1); 
    [xEast,yNorth,zUp] = latlon2local(GNSS_Pos(:, 1) , GNSS_Pos(:, 2), GNSS_Pos(:, 3), Reff_Pos); % Må endres til den andre reciveren. 
    GNSS_Pos = [xEast,yNorth,zUp];
    
    t = t_saved;
    t = seconds(t - t(1)) + (Start_Time); 
    Start = 350;
    Stopp = 440;

    % t_GNSS = combinedDateTime(Start:Stopp);
    % GNSS_Pos = GNSS_Pos(Start:Stopp, :);
    % ---- Må endres
    t_GNSS = combinedDateTime(1:end);
    GNSS_Pos = GNSS_Pos(1:end, :);
    indices = (t >= t_GNSS(1) & t <= t_GNSS(end) );
    t = t(indices);
    LoS_taps = LoS_taps(:, indices);
    GNSS_Pos = interp1(t_GNSS, GNSS_Pos, t, "spline", 'extrap');
end

function [GNSS_Pos_Static, xEast,yNorth,zUp] = StaticGNSSDataProscess(Reff_Pos)
data = readtable('C:\Users\GMELK\MATLAB Drive\GNSS\TestMappe\GNSSMesuarments.pos', 'FileType', 'text'); % Read the GNSS static data 
usefulData = data(24:end, 1:15); % Has to manely adjusted to mach the actual data
newColumnNames = {'GPST1', 'GPST2', 'latitude(deg)', 'longitude(deg)', 'height(m)', 'Q', 'ns', 'sdn(m)', 'sde(m)', 'sdu(m)', 'sdne(m)', 'sdeu(m)', 'sdun(m)', 'age(s)', 'ratio'};
usefulData.Properties.VariableNames = newColumnNames;
usefulData = usefulData(usefulData.Q == 1, :);  

Static_GNSS_Pos = [usefulData.("latitude(deg)")(:), usefulData.("longitude(deg)")(:), usefulData.("height(m)")(:)];
[xEast,yNorth,zUp] = latlon2local(Static_GNSS_Pos(:, 1) , Static_GNSS_Pos(:, 2), Static_GNSS_Pos(:, 3), Reff_Pos); % Må endres til den andre reciveren. 
GNSS_Pos_Static = mean([xEast,yNorth,zUp], 1);
d = sqrt(sum(([xEast,yNorth,zUp] - GNSS_Pos_Static).^2, 2));
total_variance = var(d);
% Display the result
fprintf('Total positional variance: %.4f\n', total_variance);

end

function [Ant1, Ant2, Ant3] = Calc_AntennePossition(GNSS_Pos_Static)
% Calculate the relative position of the antenna based on the position of
% the GNSS reciver. 

    orientation = deg2rad(140);
    Ant1 = [sin(orientation), cos(orientation), 0]*(0.15 + 0.0373) + GNSS_Pos_Static + [0, 0, 0.06 - 0.13];
    orientation = orientation + pi/2;
    Ant2 = [sin(orientation), cos(orientation), 0]*0.155 + Ant1;
    orientation = orientation - pi;
    Ant3 = [sin(orientation), cos(orientation), 0]*0.155 + Ant1;
end

function [unit_vec,TxAnt] = CalcTxAntenna_poss(GNSS_Pos_Static,GNSS_Pos)
    vec = GNSS_Pos_Static - GNSS_Pos;
    unit_vec = vec ./ sqrt(sum(vec.^2, 2));
    distance = 0;
    TxAnt = GNSS_Pos + distance * unit_vec;
end



%% For å hente ut Nr of satetlitter


% NumOfSateliteTx = MobileGNSS();
% NumOfSateliteRx = StaticGNSS();
% 
% 
% 
% 
% 
% figure('Name', 'GNSS Satellite Usage', 'NumberTitle', 'off');
%     plot(NumOfSateliteTx, '-o', 'DisplayName', 'Tx GNSS', 'LineWidth', 1.5); hold on;
%     plot(NumOfSateliteRx, '-s', 'DisplayName', 'Rx GNSS', 'LineWidth', 1.5);
%     hold off;
% 
%     title('Number of Satellites Used for GNSS', 'FontSize', 14, 'FontWeight', 'bold');
%     xlabel('Sample Number', 'FontSize', 12);
%     ylabel('Number of Satellites', 'FontSize', 12);
% 
%     legend('Location', 'best');
%     grid on;
%     set(gca, 'FontSize', 11);
% 
% 
% function NumOfSatelite = MobileGNSS()
%     data = readtable('C:\Users\GMELK\MATLAB Drive\GNSS\RPI4_pos\ublox_000.pos', 'FileType', 'text');
%     usefulData = data(24:end, 1:15);
%     newColumnNames = {'GPST1', 'GPST2', 'latitude(deg)', 'longitude(deg)', 'height(m)', 'Q', 'ns', 'sdn(m)', 'sde(m)', 'sdu(m)', 'sdne(m)', 'sdeu(m)', 'sdun(m)', 'age(s)', 'ratio'};
%     usefulData.Properties.VariableNames = newColumnNames;
%     NumOfSatelite = usefulData.ns;
% end
% 
% function NumOfSateliteRx = StaticGNSS()
% data = readtable('C:\Users\GMELK\MATLAB Drive\GNSS\TestMappe\GNSSMesuarments.pos', 'FileType', 'text'); % Read the GNSS static data 
% usefulData = data(24:end, 1:15); % Has to manely adjusted to mach the actual data
% newColumnNames = {'GPST1', 'GPST2', 'latitude(deg)', 'longitude(deg)', 'height(m)', 'Q', 'ns', 'sdn(m)', 'sde(m)', 'sdu(m)', 'sdne(m)', 'sdeu(m)', 'sdun(m)', 'age(s)', 'ratio'};
% usefulData.Properties.VariableNames = newColumnNames;
% NumOfSateliteRx = usefulData.ns;
% end