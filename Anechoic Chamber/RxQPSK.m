    % Laster in instillinger
Settings = load("Setting.mat");
fc     = Settings.fc;
fs     = Settings.fs;
N      = Settings.N;
N_Pre  = Settings.N_pre;
N_QPSK = Settings.N_symb;
Preamble = Settings.Preamble;
X        = Settings.X;

StartPos = -90;
EndPos   =  90;
Time     =  30;

    % Setter opp turntabel:
asmInfo = NET.addAssembly('Newport.XPS.CommandInterface');
xps = XPSD_NET('192.168.42.2'); % Replace with your XPS IP
Group = 'Group1';
xps.GroupInitHome(Group);

fprintf('\nFlytter til startpos:\n')
xps.SetSpeed(Group, 80);  % 80.0 deg/s 
xps.MoveAbsolute(Group, StartPos); pause(2);
fprintf('Current Position: %.3f\n', xps.GetPosition(Group));

fprintf('\nSetter opp hastighet:\n')
v = CalculateVelocityForTime(xps, Group, StartPos, EndPos, Time, []); % Kalkulerer nødvendig hastighet for å gjenomføre på bestemt tid.
xps.SetSpeed(Group, v);  
t_total = CalculateMotionTime(xps, Group, StartPos, EndPos, [], []);  % Estimert tid med satt fart.

%% Setter opp Radio

fprintf('\nStarter Radio\n');
RxUSRP = comm.SDRuReceiver("Platform", "B210", "SerialNum", "30998D7");
    RxUSRP.CenterFrequency = fc;
    RxUSRP.ChannelMapping =  [1, 2];
    RxUSRP.MasterClockRate = 30e6; 
    RxUSRP.Gain = [40, 40];
    RxUSRP.DecimationFactor = RxUSRP.MasterClockRate / fs;
    RxUSRP.OutputDataType = "double";
    RxUSRP.SamplesPerFrame = N * 2;
    SamplesPerFrame = RxUSRP.SamplesPerFrame;
for i = 1:100; Samples = RxUSRP(); end

% RxSpectrum = spectrumAnalyzer(SampleRate = fs, ChannelNames= {'RF0', 'RF1'});
% RxSpectrum(Samples);



%%

% Initialization

I = [1, 1];
LoS_array = complex(zeros(1e5, 2));



disp('Start');
xps.MoveAbsolute(Group, EndPos);
tic;
while toc < Time
    % [Samples, ~, ~, timeStamps] = RxUSRP();
    Samples = RxUSRP();
    % RxSpectrum(Samples);
    % Position = xps.GetPosition(Group);

    % Frame Sync
    [idx1, ~, ~, ~] = FrameSync(Samples(:, 1), Preamble, N); % [idx1, Threhold1, c1, lags1]
    %[idx2, ~, ~, ~] = FrameSync(Samples(:, 2), Preamble, N); % [idx2, Threhold2, c2, lags2]
    % [idx2, Threhold2, c2, lags2] = FrameSync(Samples(:, 2), Preamble, N);

    % For each frame:
    if ~isempty(idx1); [LoS_array(:, 1), I(1)] = Estimate_LoS_standalone(Samples(:, 1), idx1, N, SamplesPerFrame, X, 1); end
    if ~isempty(idx1); [LoS_array(:, 2), I(2)] = Estimate_LoS_standalone(Samples(:, 2), idx1, N, SamplesPerFrame, X, 2); end
end

disp('Stopp');
Angles = linspace(-90, 90, max(I));
LoS_array = LoS_array(1:max(I), :);







%%




figure(1);
    subplot(2, 1, 1);
    plot(Angles, abs(LoS_array./LoS_array(:, 1)), 'X');
    legend({'Channel 1', 'Channel 2'});
    title('Amplitude Offset');
    ylabel('$\Delta |A|$', 'Interpreter', 'latex');
    xlabel('Angle Turntabel (deg)');

    subplot(2, 1, 2);
    plot(Angles, angle(LoS_array./LoS_array(:, 1)), 'X');
    legend({'Channel 1', 'Channel 2'});
    title('Phase Offset');
    ylabel('$\Delta \phi$', 'Interpreter', 'latex');
    xlabel('Angle Turntabel (deg)');

% figure(2);
%     plot(df(1:floor(NumFrame/2)));
%     % xline(Angle(1:NumFrame/2, 1), '--');
%     title('CFO');
%     ylabel('Hz');
% 
figure(3);
    subplot(2, 1, 1);
    plot(Angles, abs(LoS_array));
    legend({'Channel 1', 'Channel 2'});
    title('Amplitude');
    ylabel('$|A|$', 'Interpreter', 'latex');
    xlabel('[n]');

    subplot(2, 1, 2)
    plot(Angles, unwrap(angle(LoS_array)));
    legend({'Channel 1', 'Channel 2'});
    title('Phase Unwrapped');
    ylabel('$\phi$', 'Interpreter', 'latex');
    xlabel('[n]');
% 
% figure(4);
%     plot(lags1, abs(c1), 'DisplayName', 'Channel 1'); hold on;
%     plot(lags2, abs(c2), 'DisplayName', 'Channel 2');
%     yline(Threhold1, 'r--', 'DisplayName', 'Threshold Channel 1');
%     yline(Threhold2, 'b--', 'DisplayName', 'Threshold Channel 2'); hold off;
%     legend;
%     title('Cross-correlation with Preamble');
%     ylabel('$|R_{xy}(\tau)|$', 'Interpreter', 'latex');
%     xlabel('Samples [n]');
% 
% figure(5);
%     pwelch(y, [], [], [], 'centered'); hold on;
%     pwelch(ifft(X), [], [], [], "centered"); hold off;
%     title('Spectrum Rx');
%     xlabel('Frequency/$\frac{f_s}{2}$ (Hz)', 'Interpreter', 'latex');
%     ylabel('Power/Frequency (dB/Hz)');
%     legend({'Channel 1', 'Channel 2', 'Tx'});    

% save('Data_6GHz.mat', 'LoS_array', 'Angles');

%% Func

function [idx, Threhold, c, lags] = FrameSync(inputSignal, trainingSequence, N)
    [c, lags] = xcorr(inputSignal, trainingSequence);
    C = abs(c(N:end));
    Delays = lags(N:end);
    Peaks = islocalmax(C, 'MinSeparation', N, 'MinProminence', 0.8*max(C));
    Threhold = mean(C)*15;
    hit = Peaks &(C > Threhold);
    HIT = Delays(hit);
    idx = HIT(HIT > 0);
end

function LoS = SignalProcessingFFT(y, X)
    % SIGNALPROCESSINGFFT - FFT-based channel estimation
    % Inputs:
    %   y - Received signal segment
    %   X - Reference signal
    % Output:
    %   LoS - Line-of-Sight component estimate
    
    Y = fft(y);
    H = Y ./ X;
    h = ifft(H);
    LoS = max(h);
end

function frequencyOffset = estimateFrequencyOffset(trainingSequence, symbolLength)
    % Estimate frequency offset using Moose's algorithm
    % Inputs:
    %   trainingSequence - The received training sequence
    %   symbolLength - The length of half the training sequence
    % Output:
    %   frequencyOffset - The estimated frequency offset

    % Split the training sequence into two parts
    firstPart = conj(trainingSequence(1:symbolLength));
    secondPart = trainingSequence(symbolLength+1:end);

    % Calculate the product of the two parts
    product = firstPart .* secondPart;

    % Estimate the frequency offset
    frequencyOffset = 1 / (2 * pi* symbolLength) * atan(sum(imag(product))/ sum(real(product)));
end

function [LoS_array, I] = Estimate_LoS_standalone(Rx, idx, N, SamplesPerFrame, X, channel)
    % ESTIMATE_LOS_STANDALONE - Processes dual-channel input (USRP B210)
    % Inputs:
    %   channel       - Channel index (1 or 2)
    %   ... (other inputs remain the same)
    
    % Persistent storage indexed by channel (1 or 2)
    persistent LoS_array_internal buffers bufferLogics Is B_ENDs;
    
    % Initialize persistent variables for the channel
    if isempty(LoS_array_internal) || numel(LoS_array_internal) < channel
        % Initialize for the first call
        LoS_array_internal{channel} = complex(zeros(1e5, 1));  % LoS array for this channel
        buffers{channel} = complex(zeros(N, 1));                % Buffer for this channel
        bufferLogics{channel} = [false, false];                 % Buffer logic for this channel
        Is(channel) = 1;                                        % Index tracker for this channel
        B_ENDs(channel) = 0;                                    % Buffer end for this channel
    end
    
    % Extract state variables for this channel
    current_buffer = buffers{channel};
    current_bufferLogic = bufferLogics{channel};
    current_I = Is(channel);
    current_B_END = B_ENDs(channel);
    
    % Process the received signal for this channel
    if current_bufferLogic(1)
        current_buffer(current_B_END+1:end) = Rx(1:N-current_B_END);
        LoS_array_internal{channel}(current_I) = SignalProcessingFFT(current_buffer, X);
        current_I = current_I + 1;
    end
    
    END = SamplesPerFrame - idx(end);
    LEN = length(idx);
    if END < N
        LEN = LEN - 1;
        current_buffer(1:END) = Rx(idx(end)+1:end);
        current_bufferLogic(2) = true;
        current_B_END = END;
    else
        current_bufferLogic(2) = false;
    end
    
    for i = 1:LEN
        y = Rx(idx(i)+1:idx(i)+N);
        LoS_array_internal{channel}(current_I) = SignalProcessingFFT(y, X);
        current_I = current_I + 1;
    end
    
    current_bufferLogic = [current_bufferLogic(2), false];
    
    % Update persistent variables for this channel
    buffers{channel} = current_buffer;
    bufferLogics{channel} = current_bufferLogic;
    Is(channel) = current_I;
    B_ENDs(channel) = current_B_END;
    
    % Assign outputs
    LoS_array = LoS_array_internal{channel};
    %buffer = buffers{channel};
    %bufferLogic = bufferLogics{channel};
    I = Is(channel);
    %B_END = B_ENDs(channel);
end
