% This script do the Signal Prosccesing for the Rx_PostPross.m file and
% synchronisation  with the GNSS data.  

Measurement = load('PostPross_Data.mat');
x = load('WaveForm.mat', 'TxSymb');
fc = 1.3e9;
%% Poscess Data
y = Measurement.y_saved;
t = Measurement.t_saved;
X = fft(x.TxSymb);
StartTime = Measurement.Start_Time;

%LoS_taps = zeros(2, 1.5e6);
% h_FFT  = zeros(12780, 2);
% h_apes = zeros(12780, 2);
tic
parfor i = 1:1.5e1
    for Ch = 1:2 % The channel
        % [LoS, ~, ~] = SignalProcessingFFT(y(:, Ch, i), X);
        [LoS, ~, ~] = SignalProcessingAPES(y(:, Ch, i), X);
        LoS_taps(Ch, i) = LoS;
    end
end
toc
%%
LoS_Offset = LoS_taps(:, :) ./ LoS_taps(:, 1);
save("ResultAPES.mat", "LoS_taps", "LoS_Offset");
%% Save and load data
% save("ResultAPES.mat", "LoS_taps", "LoS_Offset");
%save("ResultFFT.mat", "LoS_taps", "LoS_Offset");
% LoS_Data = load("ResultAPES.mat");
% LoS_taps = LoS_Data.LoS_taps;
% LoS_Offset = LoS_Data.LoS_Offset;

%% Plot Channel h(t)


t = linspace(0, 12780-1, 12780) / 4e6;
figure(1);
    plot(t, 20*log10(abs(h_FFT(:,1))), '-', 'Color', [0 0.4470 0.7410], 'DisplayName', 'FFT - LPDA_1'); hold on;
    plot(t, 20*log10(abs(h_apes(:,1))), '--', 'Color', [0 0.4470 0.7410], 'DisplayName', 'APES - LPDA_1');
    plot(t, 20*log10(abs(h_FFT(:,2))), '-', 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'FFT - LPDA_2');
    plot(t, 20*log10(abs(h_apes(:,2))), '--', 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'APES - LPDA_2'); hold off;
    
    % Formatting
    xlim([0, 100/4e6]);
    xlabel('Time [s] (\tau)');
    ylabel('Magnitude (dB)');
    title('Comparison of FFT and APES for h(t)');
    legend('Location', 'best');
    grid on;



figure(2);
    plot(t, (angle(h_FFT(:,1))), '-', 'Color', [0 0.4470 0.7410], 'DisplayName', 'FFT - LPDA_1'); hold on;
    plot(t, (angle(h_apes(:,1))), '--', 'Color', [0 0.4470 0.7410], 'DisplayName', 'APES - LPDA_1');
    plot(t, (angle(h_FFT(:,2))), '-', 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'FFT - LPDA_2');
    plot(t, (angle(h_apes(:,2))), '--', 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'APES - LPDA_2'); hold off;
    
    xlim([0, 100/4e6]);
    xlabel('Time [s] (\tau)');
    ylabel('Phase [rad]');
    title('Phase of FFT and APES for h(t)');
    legend('Location', 'best');
    grid on;

figure(3);
    plot(1:1.5e6, abs(LoS_Offset(:, :)))
    plot(1:1.5e6, abs(LoS_taps(:, :)))


%% Func

function [LoS, h, H] = SignalProcessingFFT(y, X)
    % SIGNALPROCESSINGFFT - FFT-based channel estimation
    % Inputs:
    %   y - Received signal segment
    %   X - Reference signal
    % Output:
    %   LoS - Line-of-Sight component estimate
    
    Y = fft(y);
    H = Y ./ X;
    h = ifft(H, 1278*10);
    LoS = max(h);
end

function [LoS, h, H] = SignalProcessingAPES(y, X)
    y = double(y);
    Y = fft(y);
    H = Y ./ X;
    h = apes(H, 320, 1278*10);
    LoS = max(h);
end

function ape = apes(x, L, Nmax)
    %  Function to compute the complex spectrum of x using the
    %  forward-and-backward APES algorithm.
    %
    %  ape = apes(x, L, Nmax);
    %
    %  x Observed 1D complex (demodulated) data;
    %  L    the number of taps of the FIR
    %    filter exploited by APES;
    %  Nmax     Number of samples of the
    %    complex spectrum evaluated in the frequency
    %      domain.
    %
    %
    %  ape Complex spectrum of x.
    %
    %  Based on the fast implementation due to Liu.
    %
    %  T. Ekman 16/8 1999
    %
    %  L = 1 renders the fourier transform  
    %  L = a quarter of the length of x is usually sensible
    %  
  x=x(:);  
  N=length(x);
  M=N-L+1;
  Y = hankel(x(1:L),x(L:N));
  %Z = conj( hankel(x(N:-1:M),x(M:-1:1)) );
  Z=rot90(Y,2);
  R = Y*Y'+Z*Z';      % Unnormailzed fb covaraince estimate
  R=(R+R')/2;
  A=chol(R);          % C in art.
  Rint12=inv(A);
  Rinty=sparse(Rint12')*Y;                % D in art.
  Rintz=sparse(Rint12')*Z;                % E in art.
  V11=zeros(1,Nmax);
  V12=zeros(1,Nmax);
  V13=zeros(1,Nmax);
  V22=zeros(1,Nmax);
  V23=zeros(1,Nmax);
  V33=zeros(1,Nmax);

  for k=1:L
      x = Rint12(:, k).'; % The B_k matrix.
      Fr=fft(x, Nmax);          % The k:th part of the b matrix.
      V11=V11+real(Fr).^2+imag(Fr).^2;     % Building || b ||^2
%
      x=Rinty(k, :);   % D_k matrix
      Fy=fft(x, Nmax);          % The k:th part of the d matrix.
      V12=V12 + Fr.*Fy;                   % Building   b^T d
      V22=V22 + real(Fy).^2+imag(Fy).^2;  % Building || d ||^2
%
      x=Rintz(k, :);   % E_k matrix
      Fz=fft(x, Nmax);          % The k:th part of the e matrix.
      V13=V13+Fr.*Fz;                    % Building   b^T e
      V23=V23+conj(Fy).*Fz;               % Building | d^H e |^2
      V33=V33+real(Fz).^2+imag(Fz).^2;   % Building || e ||^2
  end
%
  V33=M-V33;
  aQg = V12 + ( V13.*conj(V23) ) ./ V33;
  gQg = V22 +   conj(V23) .* V23 ./ V33;
  aQa = V11 +   conj(V13) .* V13 ./ V33;
  ape = aQg ./ ( (M-gQg).*aQa + conj(aQg).*aQg );
  ape=ape(:);
end

