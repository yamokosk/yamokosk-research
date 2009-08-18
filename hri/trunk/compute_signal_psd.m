function [Pxx,freq] = compute_signal_psd(signal,fs)

debug = 0;

% Preprocess signal before computing the psd
%   METHOD 1: Simply subtract the mean
%S = subtract_mean(signal);

%   METHOD 2: Subtract a linear trend from the data
%S = subtract_full_linear_trend(signal);

%   METHOD 3: Make first and last data point y=0
S = subtract_partial_linear_trend(signal);

% Filter the signal?
S = filter_signal(S, fs);

if (debug)
    disp('Plotting original and preprocessed signals.');
    figure(1); hold on;
    plot(signal,'k--');
    plot(S,'k-')
    legend('raw_data', 'pre-processed');
    ginput(1);
    close(1);
end

% Now compute the psd
N = 1024;
[Pxx, freq] = compute_psd(S, N, fs);

if (debug)
    disp('Plotting psds original and preprocessed signals.');
    
    % Compute alternate signals
    [P_orig, f_orig] = compute_psd(signal, N, fs);

    ind = min(find(freq > 25));
    figure(1)
    plot(freq(1:ind),P_orig(1:ind),'k--',freq(1:ind),Pxx(1:ind),'k-');
    legend('PSD(raw_data)','PSD(pre-processed)');
    xlabel('Freq');
    ylabel('Power/Hz');
    ginput(1);
    close(1);
end

function S = subtract_mean(signal)
S = signal - mean(signal);

function S = subtract_full_linear_trend(signal)
[numSamples,junk] = size(signal);

x = [0:1:numSamples-1]';
A = [x, ones(size(x))];
b = signal;

params = A \ b;

y = params(1)*x + params(2);
S = signal - y;

function S = subtract_partial_linear_trend(signal)
[numSamples,junk] = size(signal);

x = [0:1:numSamples-1]';
A = [1, 1; numSamples, 1];
b = [signal(1); signal(end)];
params = A \ b;

y = params(1)*x + params(2);
S = signal - y;
S = S - mean(S); % Get rid of DC term

function [Pxx, freq] = compute_psd(y, N, fs)
%[P_trend_removed,freq] = pwelch(S,[],[],N,fs);
%[Pxx,freq] = periodogram(y,[],N,fs);
[Pxx,freq] = periodogram(y,tukeywin(length(y),0.5),N,fs);
%[Pxx,freq] = periodogram(S,hamming(length(S)),N,fs);
%[Pxx,freq] = pwelch(S,300,250,N,fs);
%Pxx = P_trend_removed;

% Y = fft(S,N);
% Pxx = Y.* conj(Y) / N;
% freq = [fs*(0:N/2)/N]';
% Pxx = Pxx(1:length(freq),1)

%[Pxx, freq] = pmtm(S,3/2,N,fs);

function S = filter_signal(y, fs)

cut_off = 100; % in Hz
Wn = cut_off/(fs/2);
[b,a] = butter(6,Wn,'low');

S = filtfilt(b,a,y);