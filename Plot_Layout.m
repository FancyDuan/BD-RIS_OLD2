function Plot_Layout(params)
% 输入参数:
%   params.AP_pos - AP坐标 (B x 2)
%   params.UE_pos - UE坐标 (K x 2)
%   params.RIS_pos - RIS坐标 (R x 2)

%% 提取坐标
AP_pos = params.AP_pos;
UE_pos = params.UE_pos;
RIS_pos = params.RIS_pos;

%% 创建图形
figure;
hold on;
grid on;
box on;

%% 定义自定义颜色
color_AP  = [0.85, 0.33, 0.10];
color_RIS = [0.00, 0.45, 0.74];
color_UE  = [0.49, 0.18, 0.56];

%% --- 绘制 AP ---
if ~isempty(AP_pos)
    plot(AP_pos(:,1), AP_pos(:,2), ...
      'p', ...
      'Color', color_AP, ...            % 设置轮廓颜色
      'MarkerFaceColor', color_AP, ...  % 设置填充颜色
      'MarkerSize', 12, ...
      'DisplayName', 'AP');
end

%% --- 绘制 BD-RIS ---
if ~isempty(RIS_pos)
    plot(RIS_pos(:,1), RIS_pos(:,2), ...
        's', ...
        'Color', color_RIS, ...           % 设置轮廓颜色
        'MarkerFaceColor', color_RIS, ... % 设置填充颜色
        'MarkerSize', 6, ...
        'DisplayName', 'BD-RIS');
end

%% --- 绘制 UE ---
if ~isempty(UE_pos)
    plot(UE_pos(:,1), UE_pos(:,2), ...
        'o', ...
        'Color', color_UE, ...            % 设置轮廓颜色
        'MarkerFaceColor', color_UE, ...  % 设置填充颜色
        'MarkerSize', 4, ...
        'DisplayName', 'UE');
end

%% 图形设置
xlabel('X (m)');
ylabel('Y (m)');
title('Communication Scene Layout');
legend('show');
axis equal;
% xlim([0, 1000]); 
% ylim([0, 1000]); 
hold off;

end