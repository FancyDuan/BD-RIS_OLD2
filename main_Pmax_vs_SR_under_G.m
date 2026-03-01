%% MATLAB Code for Multi-BD-RIS-Assisted Uplink Cell-free System Sumrate Maximization
clear;
clc;
close all;
tic

%% 系统参数定义

% 定义参数结构体
params = struct();

% ==================== Network Layout ====================
params.B = 4;                                               % AP数量
params.K = 8;                                               % UE数量
params.R = 4;                                               % BD-RIS数量
params.AreaSize = 200;                                      % 区域边长 (m), 假设为正方形区域

% ==================== AP & BD-RIS Configuration ====================
params.N = 16;                                              % AP天线数      (ULA)
params.M = 32;                                              % BD-RIS元素数  (UPA)
% params.G = 1;                                               % BD-RIS元素分组数
% params.M_bar = params.M / params.G;                         % BD-RIS单组元素数

% ==================== Channel Model Paraeters ====================
params.beta_dB = -30;                                       % 参考距离为1m时的路径损耗 (dB)
params.beta = 10^(params.beta_dB / 10);
params.Rician_AP_RIS_dB = 5;                                % AP-RIS信道莱斯因子 (dB)
params.Rician_AP_RIS = 10^(params.Rician_AP_RIS_dB / 10);
params.Rician_RIS_UE_dB = 5;                                % RIS-UE信道莱斯因子 (dB)
params.Rician_RIS_UE = 10^(params.Rician_RIS_UE_dB / 10);
params.alpha_AP_RIS = 2.2;                                  % AP-RIS信道的路径损耗指数
params.alpha_RIS_UE = 2.2;                                  % RIS-UE信道的路径损耗指数

% ==================== Communication & Noise Power Parameters ====================
% params.Pmax_dBm = 20;                                       % 用户最大发射功率 (dBm)
% params.Pmax = 10^(params.Pmax_dBm / 10 - 3);                % 用户最大发射功率 (W)
params.noise_power_dBm = -80;                               % 噪声功率 (dBm)
params.noise_power = 10^(params.noise_power_dBm / 10 - 3);  % 噪声功率 (W)

%% 设置发射功率 P_T
Pmax_range_dBm = 10:5:40;                                   % 定义功率范围: 10dBm-40dBm, 步长5dBm
G_range = [1,8,32];
P_Num = length(Pmax_range_dBm);
G_Num = length(G_range);
SumRate_Record = zeros(P_Num, G_Num);                       % 预分配数组存储结果

%% 优化算法
for num_g = 1 : G_Num
    for num_p = 1 : P_Num
    
        % 重置随机种子
        rng(123);
    
        % 更新当前分组数和最大发射功率
        params.G = G_range(num_g);                          % BD-RIS元素分组数
        params.M_bar = params.M / params.G;                 % BD-RIS单组元素数
        params.Pmax_dBm = Pmax_range_dBm(num_p);
        params.Pmax = 10^(params.Pmax_dBm / 10 - 3);        % 用户最大发射功率 (W)
    
        % 初始化通信节点位置
        params = Position_Initialization(params);
        
        % 初始化信道
        params = Channel_Initialization(params);
        
        % 初始化波束赋形、功率分配和RIS相移
        params = W_P_Theta_Initialization(params);
    
        error = 100;
        iter = 1;
        obj_old = -100;
        fprintf('Initial SumRate = %4.3f\n', Calculate_SINR_SumRate(params).SumRate);
    
        while error > 1e-3 && iter <= 100
    
            fprintf('--------optimizing Theta--------\n');
            % params = Optimize_Theta_BCD_2(params);
            params = Optimize_Theta_BCD_3(params);          % 修改了欧几里得梯度的计算公式
        
            fprintf('--------optimizing P--------\n');
            params = Optimize_P_CVX_Scaled(params);
    
            fprintf('--------optimizing W--------\n');
            % params = Optimize_W_ZF_Normalized(params);
            params = Optimize_W_BCD_Normalized(params);
        
            obj_new = Calculate_SINR_SumRate(params).SumRate;
            error = abs(obj_new - obj_old);
            obj_old = obj_new;
            fprintf('iter = %2d, SumRate = %4.3f\n', iter, obj_new);
        
            iter = iter + 1;
    
        end
    
        % 记录当前 G, Pmax 下的和速率
        SumRate_Record(num_p, num_g) = Calculate_SINR_SumRate(params).SumRate;
    
    end
end

computation_time = toc;
fprintf('联合优化算法运行耗时: %.2f 秒\n', computation_time);

%% 绘制结果
figure('Color', 'w');
hold on; grid on;

% 2. 定义颜色和标记（增强区分度）
colors = lines(length(G_range));
markers = ['o', 's', '^', 'v', 'd', 'x'];

% 3. 循环绘制曲线
for g_idx = 1:length(G_range)
    plot(Pmax_range_dBm, SumRate_Record(:, g_idx), ...
        'Color', colors(g_idx, :), ...
        'Marker', markers(g_idx), ...
        'LineWidth', 1.5, ...
        'MarkerSize', 8);
end

% 4. 美化图表
xlabel('P_{max} (dBm)', 'FontSize', 12);
ylabel('Sum Rate (bps/Hz)', 'FontSize', 12);
title('Sum Rate vs. P_{max} for Different Group Sizes G', 'FontSize', 13);

% 5. 生成图例
legend_labels = arrayfun(@(g) sprintf('G = %d', g), G_range, 'UniformOutput', false);
legend(legend_labels, 'Location', 'NorthWest');

set(gca, 'FontSize', 11);
hold off;