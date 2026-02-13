close all
fs = 20; % sampling rate
N = 1; % number of period

% 设置参数
params.freq = linspace(0.1, 1.0, 10);  % 频率: for linear velocity
params.a = 0.2;  % 振幅 0.1 for linear velocity
params.psi = - 2*pi + 4 * pi * rand(1, length(params.freq));

% 设置时间范围和采样率
t_end = N * 1/min(params.freq);  % N个周期
t = 0: 1/fs: t_end;        % Time vector

% 生成合成波形
u = generate_synthetic_waveform(t, params);

% 绘制波形
fig = figure;
subplot(211)
%plot(t, u);
plot(t, u, 'b', 'LineWidth', 2.0);
title('Sum-of-sinusoids signal');
xlabel('Time [s]');
ylabel('Amplitude');
grid on;
ylim([min(u) - 0.05, max(u) + 0.05])
set(gca, 'FontSize', 20)
set(gca, 'LineWidth', 2)
% Calculate FFT
Y = fft(u);
L = length(u);
P2 = abs(Y/L);
P1 = P2(1:floor(L/2+1));
P1(2:end-1) = 2*P1(2:end-1); 
f = fs*(0:(L/2))/L;
subplot(212)
stem(f,P1,'b-','Marker','none', 'LineWidth', 2.0)
title('Single-Sided Amplitude Spectrum')
xlim([0, 1.02])
ylim([0, 0.11])
xticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
xlabel('f (Hz)')
ylabel('|P1(f)|')
grid on
set(gca, 'FontSize', 20)
set(fig, 'Position', [100, 100, 600, 800]); % 增加窗口大小
set(gca, 'LineWidth', 2)

% 保存为 PNG 文件（去除边框和余白）
%print(strcat('idinput', '.png'), ...
%    '-dpng', '-r300', '-opengl', '-noui');


% 定义生成合成波形的函数
function u = generate_synthetic_waveform(t, params)
    u = zeros(size(t));
    for j = 1: length(params.freq)
        u = u + params.a * sin(2 * pi * params.freq(j) * t ...
            + params.psi(j));
    end
end
