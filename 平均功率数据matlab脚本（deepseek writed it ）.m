%% 读取数据、剔除长时间不动异常、计算平均值并绘图
clear; clc; close all;

% ========== 1. 读取数据 ==========
filename = 'D:\RM_projects\h7_final_4\data_powerstatic.csv';   % 文件需在当前目录，或写完整路径

% 跳过第一行表头，读取数值矩阵
data = readmatrix(filename, 'NumHeaderLines', 1);

t     = data(:, 1);   % 时间戳
power = data(:, 2);   % 功率值

% 去除可能的 NaN
valid = ~isnan(t) & ~isnan(power);
t     = t(valid);
power = power(valid);

% ========== 2. 识别长时间不动的异常数据 ==========
% 连续相同值达到或超过 threshold 个点，视为异常段
threshold = 10;      % 可根据需要调整，例如 20、30

% 标记每个连续相同值段的编号
change  = [true; diff(power) ~= 0];   % true 表示新段开始
seg_id  = cumsum(change);             % 每个点所属段编号

% 计算每段长度
seg_len = accumarray(seg_id, 1);

% 找出长度 >= threshold 的段，并展开到每个点
abnormal_seg  = seg_len >= threshold;
abnormal_mask = abnormal_seg(seg_id);

% ========== 3. 计算平均值 ==========
mean_all   = mean(power);
mean_clean = mean(power(~abnormal_mask));

fprintf('总点数: %d\n', numel(power));
fprintf('剔除异常后点数: %d\n', numel(power(~abnormal_mask)));
fprintf('全部数据平均值: %.6f\n', mean_all);
fprintf('剔除长时间不动异常后的平均值: %.6f\n', mean_clean);

% ========== 4. 绘图 ==========
figure('Position', [100 100 1200 700]);

% --- 子图1：原始数据 + 异常点标记 ---
subplot(2,1,1);
plot(t, power, 'b.', 'MarkerSize', 4); hold on;
plot(t(abnormal_mask), power(abnormal_mask), 'r.', 'MarkerSize', 6);
% 画平均值线（兼容旧版 MATLAB）
plot([min(t) max(t)], [mean_clean mean_clean], 'k--', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Power');
title('原始数据与长时间不动异常点');
legend('正常数据', '长时间不动异常', sprintf('剔除后平均值 = %.4f', mean_clean), ...
    'Location', 'best');
grid on;

% --- 子图2：剔除异常后的数据 ---
subplot(2,1,2);
plot(t(~abnormal_mask), power(~abnormal_mask), 'g.', 'MarkerSize', 4); hold on;
plot([min(t) max(t)], [mean_clean mean_clean], 'k--', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Power');
title('剔除长时间不动异常后的数据');
legend('清洗后数据', sprintf('平均值 = %.4f', mean_clean), 'Location', 'best');
grid on;

% ========== 5. 可选：保存图片 ==========
% saveas(gcf, 'power_analysis.png');
% print(gcf, 'power_analysis.png', '-dpng', '-r300');

% ========== 6. 可选：直方图对比 ==========
figure('Position', [100 100 800 500]);
histogram(power, 50, 'Normalization', 'pdf', 'FaceColor', [0.7 0.7 0.7]); hold on;
histogram(power(~abnormal_mask), 50, 'Normalization', 'pdf', 'FaceColor', [0.2 0.6 1]);
plot([mean_clean mean_clean], [0 max(ylim)], 'r--', 'LineWidth', 2);
xlabel('Power');
ylabel('Probability density');
title('原始数据与清洗后数据分布对比');
legend('原始数据', '清洗后数据', sprintf('清洗后平均值 = %.4f', mean_clean));
grid on;

% saveas(gcf, 'power_histogram.png');