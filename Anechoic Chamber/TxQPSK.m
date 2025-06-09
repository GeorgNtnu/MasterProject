% This script generate a QSPK signal save it in a file that is used for the
% transmitter (Tx). 

% Parameters
fc = 6e9;                 % Carrier wave frequency (Hz)
fs = 1e6;                 % Baseband sample rate (Hz)
Time = 1000;

Preamble = [zadoffChuSeq(25,139); zadoffChuSeq(25,139)];
N_symb = 1e3;  % Number of transmitted symbol.
N_pre  = length(Preamble);
N      = N_symb + N_pre; 
Symb   = randi([0, 3], N, 1);
TxSymb = pskmod(Symb, 4, pi/4, "gray", InputType="integer");

TxSymb(1:N_pre) = Preamble;
X = fft(TxSymb);
% save('Setting.mat', "fc", "fs", 'TxSymb', "X", 'N', 'N_symb', 'N_pre', 'Preamble');
save('WaveForm.mat', 'TxSymb');
%
TxUSRP = comm.SDRuTransmitter("Platform", "B210", "SerialNum", "30998E2");
    TxUSRP.ChannelMapping = 1;
    TxUSRP.MasterClockRate = 30e6; 
    TxUSRP.CenterFrequency = fc;
    TxUSRP.Gain = 40;
    TxUSRP.InterpolationFactor = TxUSRP.MasterClockRate / fs;
TxUSRP(TxSymb);
%%
% tic;
% while toc < Time
while true
    TxUSRP(TxSymb);
end



% figure(1);
%     pwelch(TxSymb, [], [], [], 'centered');
%     title('Spectrum Tx');
%     xlabel('Frequency/$\frac{f_s}{2}$ (Hz)', 'Interpreter', 'latex');
%     ylabel('Power/Frequency (dB/Hz)');

