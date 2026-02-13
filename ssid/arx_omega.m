clear; close all;
 
file_names = strcat("./data/log_20231118_", ...
    ["0036", "0034", "0033"], ...
    "_omega.csv");

Ts = 0.2; % [s] sampling period

% ARX model
na = 1;
nb = 2;
nk = 1;

% initialize coefficients matrix
A = zeros(3, na+1);
B = zeros(3, nk+nb);

for i = 1: length(file_names)
    % file name
    file_name = file_names(i);

    % read csv file
    data_mat = readmatrix(file_name);
    omega_cmd = data_mat(:, 4);
    
    % experimental velocities
    omega = data_mat(omega_cmd~=0, 5);
    t = data_mat(omega_cmd~=0, 1);
    t = t - t(1);
    omega_cmd = omega_cmd(omega_cmd~=0);
  
    % perform system identification using ARMAX model
    z = iddata(omega, omega_cmd, Ts);
    sys = arx(z, [na nb nk]); 
    A(i, :) = sys.A;
    B(i, :) = sys.B;

    % plot
    fig = figure;
    [y_id, fit_id, ~] = compare(z, sys);
    stairs(t, omega_cmd, 'k:', 'linew', 2.5)
    hold on
    plot(t, omega, 'b', 'linew', 2.5) 
    plot(t, y_id.OutputData, 'r--', 'linew', 2.5)  
    lgd = legend('ref', 'exp', ['model:', 'fit(', num2str(fit_id), '%)']);
    set(lgd, 'Location', 'northeastoutside', ...
        'NumColumns', 1, 'Orientation', 'horizontal')
    xlabel('Time [s]')
    ylabel('Angular Velocity [rad/s]')
    title(strcat('Experiment ', " ", num2str(i), " ", "for \omega"))
    set(get(gca, 'XLabel'), 'FontSize', 26) 
    set(get(gca, 'YLabel'), 'FontSize', 26) 
    set(get(gca, 'Title'), 'FontSize', 26) 
    set(gca, 'FontSize', 26)
    set(gca, 'LineWidth', 2)
    ylim([min(min(omega_cmd), min(omega)) - 0.05, ...
        max(max(omega_cmd), max(omega)) + 0.05]);
    grid on
    

    % 调整图形窗口大小以确保图例完全可见
    set(fig, 'Position', [100, 100, 1050, 600]); % 增加窗口大小

    % 保存为 PNG 文件（去除边框和余白）
    print(strcat('ssid_omega', num2str(i), '.png'), ...
        '-dpng', '-r300', '-opengl', '-noui');
end

