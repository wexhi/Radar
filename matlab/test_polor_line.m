% 构造雷达散点的极坐标参数
%rho = [1, 2, 3, 4, 5, 6, 7, 8, 9]; % 雷达散点的极径
%theta = [pi/4, pi/4, pi/4, pi/4, pi/4, pi/4, pi/4, pi/4, pi/4]; % 雷达散点的极角

rho = [1.72,  2.38,  3.79,  3.49,  5.36,  5.66,  7.07,  7.76,  8.75,  9.79,  11.26, 11.72, 12.05, 13.41, 14.79, 15.23, 16.42, 17.20, 18.18, 18.64, 20.12, 20.68, 21.44, 22.48, 24.09, 24.88, 25.42, 26.44, 27.73, 27.65, 29.31, 30.25, 31.19, 32.13, 32.52, 33.59, 34.94, 34.80, 36.02, 37.80, 38.33, 39.12, 40.68, 40.90, 42.19, 42.84, 44.14, 44.39, 45.76, 46.91, 47.30, 48.70, 49.87, 50.71, 51.33, 52.31, 53.05, 53.62, 55.17, 56.52, 56.72, 58.88, 59.06, 61.25, 61.64, 62.26, 63.00, 64.19, 64.82, 67.01, 67.37, 68.87, 69.32, 70.51, 71.55, 72.27, 72.48, 74.34, 75.08, 76.00, 76.98, 77.46, 78.95, 79.35, 80.21, 81.38, 82.35, 83.48, 84.16, 85.17, 86.15, 86.57, 88.24, 89.32, 90.26, 90.57, 91.39, 92.43, 93.04, 94.26, 95.56, 97.31, 96.28, 97.63, 98.94, 99.56, 100.09, 101.41, 102.81, 103.76, 104.47, 104.90, 102.36, 105.27, 107.17, 109.98, 109.74, 110.33, 111.33, 112.53, 114.10, 114.30, 115.84, 116.69, 117.30, 118.69, 119.14, 120.07, 121.13, 121.54, 123.16, 123.77, 124.62, 158.72, 162.52, 285.46, 285.89, 287.50, 289.10, 290.17, 289.79, 292.25, 293.04, 291.69, 292.94, 294.91, 296.08, 295.87, 297.53, 298.74, 300.05, 299.68, 301.58, 302.77, 301.69, 302.95, 306.06, 306.57, 305.20, 310.81, 313.42, 313.59, 314.80, 316.25, 316.70, 318.36, 318.65, 319.93, 320.57, 321.21, 321.81, 323.18, 324.56, 324.66, 325.65, 327.40, 327.17, 329.30, 330.08, 331.40, 331.95, 331.73, 333.77, 335.28, 334.78, 335.68, 337.35, 337.98, 337.48, 339.65, 341.66, 342.03, 341.74, 342.83, 344.77, 345.73, 345.50, 347.41, 349.39, 348.74, 350.00, 351.40, 351.14, 351.74, 354.11, 354.66, 355.88, 355.92, 356.84, 358.93, 358.51, 359.54];
theta = [399, 291, 291, 290, 290, 290, 290, 290, 290, 290, 290, 290, 290, 291, 291, 292, 292, 293, 293, 294, 296, 296, 298, 299, 300, 301, 303, 305, 307, 308, 310, 310, 313, 315, 319, 320, 324, 324, 327, 330, 332, 338, 342, 345, 348, 353, 358, 358, 365, 371, 375, 377, 384, 388, 393, 400, 406, 413, 420, 427, 433, 440, 449, 429, 450, 444, 439, 436, 101, 102, 418, 414, 413, 407, 405, 399, 398, 395, 392, 388, 386, 385, 383, 370, 364, 376, 374, 373, 373, 355, 345, 343, 342, 348, 447, 447, 445, 445, 445, 444, 444, 443, 443, 443, 443, 444, 444, 445, 445, 446, 447, 448, 448, 449, 452, 454, 454, 454, 457, 459, 462, 464, 464, 469, 472, 473, 475, 480, 485, 482, 489, 495, 499, 270, 106, 414, 415, 418, 443, 444, 443, 445, 448, 449, 449, 455, 455, 457, 458, 465, 465, 464, 472, 475, 476, 477, 482, 488, 493, 497, 482, 491, 489, 479, 460, 462, 456, 437, 429, 421, 424, 413, 402, 393, 397, 380, 376, 372, 367, 360, 357, 360, 348, 345, 342, 342, 341, 330, 331, 329, 321, 320, 319, 317, 312, 312, 313, 310, 306, 305, 305, 302, 301, 300, 299, 295, 296, 295, 295,  291, 290, 290];
rho = deg2rad(rho);
% 将极坐标转换为直角坐标
% 转换为笛卡尔坐标系
x = rho .* cos(theta);
y = rho .* sin(theta);

% 绘制散点图
scatter(x, y);
hold on;

% 使用RANSAC算法检测直线
maxDistance = 0.1; % 最大拟合误差
model = fitlineRANSAC([x', y'], maxDistance);

% 绘制检测到的直线
xlim([-10, 10]);
ylim([-10, 10]);
plot([-10, 10], model([-10, 10]), 'r', 'LineWidth', 2);

% RANSAC算法拟合直线函数
function lineModel = fitlineRANSAC(data, maxDistance)
    numPoints = size(data, 1);
    bestInlierCount = 0;
    lineModel = [];
    
    for i = 1:1000 % 迭代次数
        % 随机选择两个点作为直线模型的参数
        sampleIndices = randperm(numPoints, 2);
        samplePoints = data(sampleIndices, :);
        
        % 拟合直线
        line = polyfit(samplePoints(:, 1), samplePoints(:, 2), 1);
        
        % 计算所有点到直线的距离
        distances = abs(polyval(line, data(:, 1)) - data(:, 2));
        
        % 统计符合阈值条件的内点数量
        inlierIndices = find(distances < maxDistance);
        inlierCount = length(inlierIndices);
        
        % 更新最优模型参数
        if inlierCount > bestInlierCount
            bestInlierCount = inlierCount;
            lineModel = line;
        end
    end
end

