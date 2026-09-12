
% =========================================================================
% 最终定稿：数据清洗 + 带非负投影的递归最小二乘法 (RLS)
% 模型: P = tau*w + a*|w| + b*tau^2 + c
% =========================================================================
clear; clc; close all;

%% 1. 读取原始数据
filename = 'C:\Users\Murph\Desktop\data_3single.csv';
try
    opts = detectImportOptions(filename);
    opts.VariableNamingRule = 'preserve'; 
    data = readtable(filename, opts);
catch
    error('无法读取文件，请确保路径正确。');
end

P_raw = double(data{:, 2}(:));          
tau_raw = double(data{:, 3}(:));        
omega_raw = double(data{:, 4}(:));      

nan_idx = isnan(P_raw) | isnan(tau_raw) | isnan(omega_raw);
P_raw(nan_idx) = 0; tau_raw(nan_idx) = 0; omega_raw(nan_idx) = 0;

%% 2. 数据清洗（只剔除极端异常，保留真实工况）
% 物理硬阈值：保留 -20 到 100W 的数据
mask_phys = (P_raw >= -20) & (P_raw <= 100);
% 剔除矛盾点：扭矩极小但功率很大（说明数据解析错位）
mask_logic = ~(abs(tau_raw) < 0.001 & P_raw > 10);
% MAD 统计学剔除
temp_P = P_raw(mask_phys & mask_logic);
median_P = median(temp_P);
MAD = median(abs(temp_P - median_P));
threshold_upper = median_P + 3 * 1.4826 * MAD;
threshold_lower = median_P - 3 * 1.4826 * MAD;

valid_mask = mask_phys & mask_logic & (P_raw <= threshold_upper) & (P_raw >= threshold_lower);

P_meas = P_raw(valid_mask);
tau_meas = tau_raw(valid_mask);
omega_meas = omega_raw(valid_mask);
N_clean = length(P_meas);

fprintf('原始数据: %d, 清洗后: %d (剔除 %d 个极端异常点)\n', length(P_raw), N_clean, length(P_raw) - N_clean);

%% 3. 构造RLS输入与输出
Y = P_meas - tau_meas .* omega_meas;
X = [abs(omega_meas), tau_meas.^2, ones(N_clean, 1)];

%% 4. 标准RLS (无约束)
theta_uncon = zeros(3, 1); 
P_cov_uncon = 1e5 * eye(3);
lambda = 1;
for k = 1:N_clean
    phi_k = X(k, :)';
    K = (P_cov_uncon * phi_k) / (lambda + phi_k' * P_cov_uncon * phi_k);
    theta_uncon = theta_uncon + K * (Y(k) - phi_k' * theta_uncon);
    P_cov_uncon = (P_cov_uncon - K * phi_k' * P_cov_uncon) / lambda;
end

%% 5. 带非负投影的RLS (鲁棒约束)
theta_con = zeros(3, 1); 
P_cov_con = 1e5 * eye(3);
theta_history = zeros(N_clean, 3);
for k = 1:N_clean
    phi_k = X(k, :)';
    K = (P_cov_con * phi_k) / (lambda + phi_k' * P_cov_con * phi_k);
    theta_con = theta_con + K * (Y(k) - phi_k' * theta_con);
    P_cov_con = (P_cov_con - K * phi_k' * P_cov_con) / lambda;
    % 物理约束：损耗系数必须非负
    theta_con = max(theta_con, 0); 
    theta_history(k, :) = theta_con';
end

%% 6. 结果输出与对比
fprintf('\n=== 拟合结果对比 ===\n');
fprintf('无约束 RLS: a = %.4f, b = %.4f, c = %.4f\n', theta_uncon(1), theta_uncon(2), theta_uncon(3));
fprintf('非负约束 RLS: a = %.4f, b = %.4f, c = %.4f\n', theta_con(1), theta_con(2), theta_con(3));

% 使用有约束的系数进行预测
a_fit = theta_con(1); b_fit = theta_con(2); c_fit = theta_con(3);
P_pred = tau_meas.*omega_meas + a_fit*abs(omega_meas) + b_fit*tau_meas.^2 + c_fit;

residuals = P_meas - P_pred;
SSE = sum(residuals.^2);           
SST = sum((P_meas - mean(P_meas)).^2); 
R_squared = 1 - SSE/SST;
RMSE = sqrt(mean(residuals.^2));

fprintf('\n--- 非负约束 RLS 评估 ---\n');
fprintf('R-squared (R方): %.4f\n', R_squared);
fprintf('RMSE (均方根误差): %.4f W\n', RMSE);

%% 7. 绘图（高对比度配色版）
figure('Name', '最终版：清洗数据后的RLS拟合', 'Color', 'k', 'Position', [100, 100, 1400, 900]);

% 图1: 参数收敛过程
subplot(2, 2, 1);
% 使用高对比度颜色绘制参数收敛
plot(1:N_clean, theta_history(:, 1), 'c', 'LineWidth', 1.5); hold on; % 青色 a
plot(1:N_clean, theta_history(:, 2), 'y', 'LineWidth', 1.5);          % 黄色 b
plot(1:N_clean, theta_history(:, 3), 'm', 'LineWidth', 1.5);          % 品红 c
title('参数收敛过程 (非负投影RLS)', 'Color', 'w');
xlabel('迭代次数', 'Color', 'w'); ylabel('参数值', 'Color', 'w');
legend('a', 'b', 'c', 'TextColor', 'w', 'Color', 'none');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); % 坐标轴设为黑底白字
grid on;

% 图2: 时间序列对比 (重点修改部分)
subplot(2, 2, 2);
% 实际功率：亮青色实线 (高对比度)
plot(1:N_clean, P_meas, 'c-', 'LineWidth', 0.5); hold on; 
% 预测功率：黄色粗虚线 (高对比度)
plot(1:N_clean, P_pred, 'y--', 'LineWidth', 1.5);
title('实际功率与预测功率 (清洗后)', 'Color', 'w');
xlabel('样本序号', 'Color', 'w'); ylabel('功率 (W)', 'Color', 'w');
legend('实际功率(青色)', '预测功率(黄色)', 'TextColor', 'w', 'Color', 'none');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
grid on;
ylim([-10, 30]);

% 图3: 散点图
subplot(2, 2, 3);
% 散点使用亮青色填充，稍微降低透明度防止挡视线
scatter(P_meas, P_pred, 15, 'c', 'filled', 'MarkerFaceAlpha', 0.3); hold on;
% 理想对角线使用白色
plot([min(P_meas), max(P_meas)], [min(P_meas), max(P_meas)], 'w--', 'LineWidth', 2);
title('预测 vs 实际 (散点图)', 'Color', 'w');
xlabel('实际功率 (W)', 'Color', 'w'); ylabel('预测功率 (W)', 'Color', 'w');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
grid on;

% 图4: 残差直方图
subplot(2, 2, 4);
% 直方图使用黄色
histogram(residuals, 50, 'FaceColor', 'y', 'EdgeColor', 'none');
title('预测残差直方图', 'Color', 'w');
xlabel('残差 (W)', 'Color', 'w'); ylabel('频数', 'Color', 'w');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
grid on;
%% 终极诊断图：P vs tau*w 散点图
figure('Name', '数据物理相关性诊断', 'Position', [200, 200, 600, 500]);

% 绘制 实际功率 vs (扭矩*转速)
scatter(tau_meas .* omega_meas, P_meas, 10, 'filled', 'MarkerFaceAlpha', 0.3, 'MarkerEdgeColor', 'none');
hold on;
xlabel('扭矩 \times 转速 (\tau \cdot \omega) [W]');
ylabel('实际测量功率 P [W]');
title('终极诊断：实际功率 vs 理论输出功率 (\tau\omega)');
grid on;

% 画一条 y=x 的对角线
max_val = max(abs([tau_meas .* omega_meas; P_meas]));
plot([-1, max_val], [-1, max_val], 'r--', 'LineWidth', 2);
legend('实际数据点', '理想物理关系 (P = \tau\omega)');

% 计算相关系数
corr_coef = corr(tau_meas .* omega_meas, P_meas);
text(0.05, 0.95, sprintf('相关系数 R = %.4f', corr_coef), ...
    'Units', 'normalized', 'FontSize', 12, 'BackgroundColor', 'w');