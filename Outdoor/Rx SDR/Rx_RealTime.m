fc = 1.3e9;
fs = 4e6;

RxUSRP = comm.SDRuReceiver("Platform", "B210", "SerialNum", "30998E2");
    RxUSRP.CenterFrequency = fc;
    RxUSRP.ChannelMapping =  [1, 2];
    RxUSRP.MasterClockRate = 20e6; 
    RxUSRP.Gain = [40, 40];
    RxUSRP.DecimationFactor = RxUSRP.MasterClockRate / fs;
    RxUSRP.OutputDataType = "single";
    %RxUSRP.TransportDataType = "int16";
    RxUSRP.SamplesPerFrame = 2^12; %1278 * 2;
    SamplesPerFrame = RxUSRP.SamplesPerFrame;


RxSpectrum = spectrumAnalyzer(SampleRate = fs, ChannelNames= {'RF0', 'RF1'});
RxScope    = timescope(SampleRate=fs, ChannelNames={'RF0', 'RF1'});

Preamble = [zadoffChuSeq(25,139); zadoffChuSeq(25,139)];
x = load('WaveForm.mat', 'TxSymb');
X = fft(x.TxSymb);
    % Testing
[Samp, ~, ~, t] = RxUSRP(); 
RxSpectrum(Samp);
RxScope(abs(Samp));

%%
disp('Oppvarming');
for i = 1:10000
    [Samp, ~, ~, t] = RxUSRP(); 
    RxSpectrum(Samp);
    RxScope(abs(Samp));
end
disp('Ferdig Oppvarming');

%% Sampling
NumberOfPacket = 1.5e6;

disp('Lag plass')
y_saved = complex(zeros(1278, 2, NumberOfPacket, 'single'));
t_saved = zeros(NumberOfPacket, 1, 1);
Packet_Nr = 1;
%%
tic;
toc;
disp('Start');
[Start_Time, Start_Time_Java, T_start] = StartTime(RxUSRP); % At the start of the loop
for i = 1:NumberOfPacket
    [Samp, ~, ~, t] = RxUSRP();
    % RxSpectrum(Samp);

    idx = FrameSync(Samp(:, 1), Preamble, 1278);
    NrOfPacketsInFrame = length(idx);
        
    for I = 1:NrOfPacketsInFrame-1
            % y = Samp(idx(I)+1:idx(I)+1278, Ch);
            y_saved(:, :, Packet_Nr) = Samp(idx(I)+1:idx(I)+1278, :); 
            t_saved(Packet_Nr) = t(idx(I));
            Packet_Nr = Packet_Nr + 1;

            if Packet_Nr >= NumberOfPacket
                disp('break1!!!!');
                break;
            end
    end

    if Packet_Nr >= NumberOfPacket
        disp('break2!!!!');
        break;
    end

end

toc;
disp('Stopp');

%% Saving Data
disp('Saving the packets data');
save('PostPross_Data.mat', "t_saved", "y_saved", "Start_Time", "Start_Time_Java", "T_start", '-v7.3');
disp('Finiched');


%% Function


function idx = FrameSync(inputSignal, trainingSequence, N)
    [c, lags] = xcorr(inputSignal, trainingSequence);
    C = abs(c(N:end));
    Delays = lags(N:end);
    Peaks = islocalmax(C, 'MinSeparation', N, 'MinProminence', 0.8*max(C));
    Threhold = mean(C)*15;
    hit = Peaks &(C > Threhold);
    HIT = Delays(hit);
    idx = HIT(HIT > 0);
end

function [Start_Time, Start_Time_Java, T_start] = StartTime(RxUSRP)
    Start_Time = datetime('now', 'Format', 'HH:mm:ss.SSS', 'TimeZone','UTC');
    Start_Time_Java = java.lang.System.currentTimeMillis();
    for P = 1:100
        [~, ~, ~, T_start] = RxUSRP();
    end
end


