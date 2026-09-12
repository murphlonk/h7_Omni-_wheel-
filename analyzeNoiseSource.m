function result = analyzeNoiseSource(filename, fs, highBand)
% ============================================================
%  高频振荡 & 噪声源分析工具
%
%  用法：
%    analyzeNoiseSource('data.csv', 1000, [200 450])  % 指定 fs 与关注频段
%    analyzeNoiseSource('data.csv', 1000)             % 自动取上半频段
%    analyzeNoiseSource('data.csv')                   % fs=1，做相对频率分析
%
%  输出：
%    图1 各通道功率谱 + 高频能量占比
%    图2 通道间相干性矩阵热图
%    图3 候选噪声源排名
%    图4 时域波形对比
%    图5 候选源与其余通道的相干谱
%
%  配色：全亮色系
% ============================================================

if nargin < 1 || isempty(filename)
    [f, p] = uigetfile({'*.csv','CSV (*.csv)'}, '选择 CSV 文件');
    if isequal(f,0), return; end
    filename = fullfile(p, f);
end
if nargin < 2 || isempty(fs), fs = 1; end

%% ---------- 1. 读取数据 ----------
try
    T = readtable(filename, 'VariableNamingRule','preserve');
    vn = T.Properties.VariableNames;
    isNum = varfun(@isnumeric, T, 'OutputFormat','uniform');
    data = double(table2array(T(:, isNum)));
    varNames = vn(isNum);
catch
    data = double(readmatrix(filename));
    varNames = arrayfun(@(q) sprintf('C%d',q), 1:size(data,2), 'UniformOutput', false);
end

[n, m] = size(data);
if n < 32, warning('样本数偏少，频谱分析可能不稳。'); end

% 去均值 + NaN 插值
data = data - mean(data, 1, 'omitnan');
for j = 1:m
    v = isnan(data(:,j));
    if any(v) && ~all(v)
        data(v,j) = interp1(find(~v), data(~v,j), find(v), 'linear', 'extrap');
    elseif all(v)
        data(:,j) = 0;
    end
end

fprintf('==== 数据 %d 行 × %d 列, fs = %g Hz ====\n', n, m, fs);

%% ---------- 2. 参数 ----------
nfft   = min(1024, 2^floor(log2(n)));
winLen = min(256, nfft);
win    = hann(winLen);
nover  = floor(winLen/2);

%% ---------- 3. Welch 功率谱 ----------
[Pxx, fAx] = pwelch(data, win, nover, nfft, fs);
fmax = fs/2;

if nargin < 3 || isempty(highBand)
    highBand = [0.50*fmax, 0.98*fmax];       % 默认上半频段
end
bandIdx = fAx >= highBand(1) & fAx <= highBand(2);
if ~any(bandIdx)
    error('高频段 [%g %g] Hz 超出 [0 %g] Hz。', highBand(1), highBand(2), fmax);
end

Ptot   = trapz(fAx, Pxx, 1);
Pband  = trapz(fAx(bandIdx), Pxx(bandIdx,:), 1);
hfRatio = Pband ./ max(Ptot, eps);

%% ---------- 4. 两两相干 ----------
C = eye(m);
LagMat = zeros(m);
for i = 1:m
    for j = i+1:m
        [Cxy, fC] = mscohere(data(:,i), data(:,j), win, nover, nfft, fs);
        bIdx = fC >= highBand(1) & fC <= highBand(2);
        if any(bIdx)
            C(i,j) = mean(Cxy(bIdx), 'omitnan');
            C(j,i) = C(i,j);
        end
        % 全带互相关滞后 (粗略传播方向)
        [xc, lg] = xcorr(data(:,j)-mean(data(:,j)), data(:,i)-mean(data(:,i)), 'coeff');
        [~, im] = max(abs(xc));
        LagMat(i,j) =  lg(im);    % i 相对 j 的滞后
        LagMat(j,i) = -lg(im);
    end
end

%% ---------- 5. 噪声源打分 ----------
% 得分 = 与其他所有通道的高频相干均值 × 高频能量加权因子
score = zeros(1,m);
for i = 1:m
    score(i) = mean(C(i, setdiff(1:m,i)), 'omitnan');
end
overallScore = score .* (0.5 + 0.5*hfRatio);
overallScore = overallScore / max(overallScore + eps);

[~, rankIdx] = sort(overallScore, 'descend');

fprintf('\n%-14s | 高频能量占比 | 高频相干均值 | 综合得分\n', '通道');
fprintf('---------------------------------------------------------\n');
for k = 1:m
    i = rankIdx(k);
    tag = '';
    if k == 1, tag = '  <== 候选噪声源'; end
    fprintf('%-14s | %12.3f | %12.3f | %8.3f%s\n', ...
        varNames{i}, hfRatio(i), score(i), overallScore(i), tag);
end

% 用滞后矩阵判断传播方向
best = rankIdx(1);
leadCount = sum(LagMat(best, :) > 0);
if leadCount >= m-1
    fprintf('\n>>> 通道 "%s" 在时序上领先几乎所有通道，进一步支持它是噪声源头。\n', varNames{best});
elseif leadCount <= 1
    fprintf('\n>>> 通道 "%s" 与多数通道无固定领先关系，可能是被污染通道。\n', varNames{best});
end

result = struct('names',{varNames},'fs',fs,'Pxx',Pxx,'fAx',fAx, ...
    'highBand',highBand,'hfRatio',hfRatio,'coherence',C, ...
    'lagMat',LagMat,'noiseScore',overallScore,'rank',rankIdx,'data',data);
assignin('base','noiseResult',result);

%% ---------- 6. 绘图 ----------
palette = brightPalette(m);

% --- 图1: PSD + 高频能量占比 ---
figure('Name','频谱总览','Color','w','Position',[80 80 1200 620]);
subplot(1,2,1); hold on;
for i = 1:m
    plot(fAx, 10*log10(Pxx(:,i)+eps), '-', 'Color', palette(i,:), 'LineWidth', 1.2);
end
yl = ylim;
patch([highBand(1) highBand(2) highBand(2) highBand(1)], ...
      [yl(1) yl(1) yl(2) yl(2)], [1.0 0.95 0.75], ...
      'EdgeColor','none','FaceAlpha',0.35);
uistack(findobj(gca,'Type','patch'),'bottom');
hold off; grid on; box off;
xlabel('频率 (Hz)'); ylabel('PSD (dB/Hz)');
title('各通道功率谱  (黄区 = 关注高频段)','FontWeight','bold');
legend(varNames,'Location','bestoutside','Box','off','FontSize',8.5);

subplot(1,2,2);
[~, sIdx] = sort(hfRatio, 'descend');
bar(hfRatio(sIdx), 'FaceColor',[0.35 0.80 1.00], 'EdgeColor','none');
grid on; box off;
set(gca,'XTick',1:m,'XTickLabel',varNames(sIdx),'FontSize',9);
xtickangle(35);
ylabel('高频能量 / 总能量');
title(sprintf('高频能量占比 (%.1f ~ %.1f Hz)', highBand(1), highBand(2)), ...
      'FontWeight','bold');

% --- 图2: 相干性热图 ---
figure('Name','通道间相干','Color','w','Position',[100 100 720 660]);
imagesc(C, [0 1]); colormap(makeBrightColormap()); colorbar; axis square;
set(gca,'XTick',1:m,'XTickLabel',varNames,'FontSize',9, ...
        'YTick',1:m,'YTickLabel',varNames);
xtickangle(35);
title(sprintf('高频段 (%.1f~%.1f Hz) 幅度平方相干', highBand(1), highBand(2)), ...
      'FontWeight','bold');
for i = 1:m
    for j = 1:m
        text(j, i, sprintf('%.2f', C(i,j)), ...
            'HorizontalAlignment','center','FontSize',7.5,'Color',[0.15 0.15 0.15]);
    end
end

% --- 图3: 噪声源排名 ---
figure('Name','候选噪声源排名','Color','w','Position',[120 80 900 560]);
[ss, ord] = sort(overallScore, 'descend');
b = bar(ss, 'FaceColor','flat', 'EdgeColor','none');
colors = repmat([0.55 0.82 1.00], m, 1);
colors(1,:) = [1.00 0.35 0.35];
b.CData = colors;
grid on; box off;
set(gca,'XTick',1:m,'XTickLabel',varNames(ord),'FontSize',9);
xtickangle(35);
ylabel('噪声源综合得分');
title('候选噪声源排名 (越靠前越可能是源头)','FontWeight','bold');
for k = 1:m
    text(k, ss(k)+0.01, sprintf('%.2f',ss(k)), ...
        'HorizontalAlignment','center','FontSize',9);
end
ylim([0 max(ss)*1.2 + 0.05]);

% --- 图4: 时域波形 ---
figure('Name','时域波形','Color','w','Position',[140 60 1200 700]);
nShow = min(800, n);
t = (0:nShow-1) / fs;
for i = 1:m
    subplot(m, 1, i);
    xn = data(1:nShow, i) / max(std(data(:,i)), eps);
    plot(t, xn, 'Color', palette(i,:), 'LineWidth', 1.0);
    grid on; box off;
    ylabel(varNames{i},'FontSize',8,'Interpreter','none');
    set(gca,'FontSize',8,'GridAlpha',0.15);
    if i == 1
        title(sprintf('归一化时域波形 (前 %d 点)', nShow),'FontWeight','bold');
    end
    if i == m, xlabel('时间 (s)'); end
end

% --- 图5: 候选源相干谱 ---
best = rankIdx(1);
figure('Name','候选源相干谱','Color','w','Position',[160 80 900 620]);
hold on;
others = setdiff(1:m, best);
for k = 1:numel(others)
    i = others(k);
    [Cxy, fC] = mscohere(data(:,best), data(:,i), win, nover, nfft, fs);
    plot(fC, Cxy, '-', 'Color', palette(i,:), 'LineWidth', 1.3);
end
yl = ylim;
patch([highBand(1) highBand(2) highBand(2) highBand(1)], ...
      [yl(1) yl(1) yl(2) yl(2)], [1.0 0.95 0.75], ...
      'EdgeColor','none','FaceAlpha',0.30);
uistack(findobj(gca,'Type','patch'),'bottom');
hold off; grid on; box off;
xlabel('频率 (Hz)'); ylabel('相干性'); ylim([0 1]);
title(sprintf('候选噪声源 "%s" 与其余通道的相干谱', varNames{best}), ...
      'FontWeight','bold');
legend(varNames(others),'Location','bestoutside','Box','off','FontSize',8.5);

fprintf('\n分析完成。结果已存到工作区变量 noiseResult。\n');
end

%% ---------- 辅助 ----------
function pal = brightPalette(m)
p = [0.20 0.70 1.00;
     1.00 0.55 0.10;
     0.30 0.90 0.45;
     1.00 0.40 0.70;
     1.00 0.90 0.20;
     0.70 0.50 1.00;
     0.20 0.90 0.90;
     1.00 0.40 0.40];
pal = repmat(p, ceil(m/size(p,1)), 1);
pal = pal(1:m, :);
end

function cmap = makeBrightColormap()
n = 64;
anchors = [1.00 1.00 1.00;    % 白
           1.00 1.00 0.55;    % 亮黄
           1.00 0.75 0.30;    % 亮橙
           1.00 0.40 0.35;    % 亮红
           0.85 0.20 0.65];   % 亮品红
x  = linspace(0,1,size(anchors,1));
xi = linspace(0,1,n);
cmap = interp1(x, anchors, xi, 'linear');
end