clear; close all;

file_names = strcat("./data/log_20231118_", ...
    ["0029", "0030", "0032"], ...
    "_v.csv");


Ts = 0.05; % [s] sampling period

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
    v_cmd = data_mat(:, 2);
    
    % experimental velocities
    v = data_mat(v_cmd~=0, 5);
    t = data_mat(v_cmd~=0, 1);
    t = t - t(1);
    v_cmd = v_cmd(v_cmd~=0);
  
    % perform system identification using ARMAX model
    v_id = resample(v,4,1);
    v_cmd_id = repelem(v_cmd, 4);
    z = iddata(v_id, v_cmd_id, Ts/4);
    sys = armax(z, [na nb 0 nk]); 
    A(i, :) = sys.A;
    B(i, :) = sys.B;

    % plot
    fig = figure;
    [y_id, fit_id, ~] = compare(z, sys);
    stairs(t, v_cmd_id, 'k:', 'linew', 2.5)
    hold on
    plot(t, v_id, 'b', 'linew', 2.5) 
    plot(t, y_id.OutputData, 'r--', 'linew', 2.5)  
    lgd = legend('CMD', 'EXP', ['ARX: ', 'fit(', num2str(fit_id), '%)']);
    set(lgd, 'Location', 'northeastoutside', ...
        'NumColumns', 1, 'Orientation', 'horizontal')
    xlabel('Time [s]')
    ylabel('Linear Velocity [m/s]')
    title(strcat('Experiment ', " ", num2str(i), " ", "for \it{v}"))
    set(get(gca, 'XLabel'), 'FontSize', 26) 
    set(get(gca, 'YLabel'), 'FontSize', 26) 
    set(get(gca, 'Title'), 'FontSize', 26) 
    set(gca, 'FontSize', 26)
    set(gca, 'LineWidth', 2)
    ylim([min(min(v_cmd), min(v)) - 0.05, ...
        max(max(v_cmd), max(v)) + 0.05]);
    grid on
    

    % 调整图形窗口大小以确保图例完全可见
    set(fig, 'Position', [100, 100, 1050, 600]); % 增加窗口大小

    % 保存为 PNG 文件（去除边框和余白）
    print(strcat('ssid_v', num2str(i), '.png'), ...
        '-dpng', '-r300', '-opengl', '-noui');
end

