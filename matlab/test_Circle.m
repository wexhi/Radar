%angles = [330.29, 1.00,  1.92,  3.49,  4.03,  4.95,  5.73,  7.61,  7.96,  9.39,  9.77,  10.85, 11.61, 12.28, 13.93, 14.52, 15.37, 16.11, 17.34, 17.43, 19.21, 20.19, 20.69, 21.73, 22.81, 23.44, 24.91, 24.85, 26.23, 27.11, 27.65, 28.48, 29.83, 31.88, 31.59, 33.18, 33.61, 34.51, 35.41, 35.69, 37.38, 37.98, 39.26, 40.78, 41.19, 41.75, 43.42, 43.41, 44.03, 45.45, 46.49, 47.40, 48.19, 49.36, 50.40, 50.39, 52.86, 52.75, 54.19, 60.20, 71.92, 73.37, 73.36, 75.81, 75.42, 76.89, 78.01, 78.31, 79.53, 80.44, 81.25, 82.53, 83.11, 84.22, 84.63, 85.60, 86.55, 88.21, 88.79, 89.83, 90.77, 156.22, 156.90, 158.13, 158.80, 271.96, 278.37, 319.37, 320.45, 321.05, 321.41, 323.52, 324.16, 324.88, 325.72, 327.16, 327.30, 327.42, 329.59, 330.83, 331.49, 331.92, 333.21, 334.42, 334.44, 335.00, 337.09, 337.26, 337.96, 340.22, 341.94, 341.99, 341.93, 343.43, 344.74, 344.69, 346.33, 346.86, 348.69, 347.89, 349.58, 350.48, 351.24, 351.35, 353.81, 354.51, 355.22, 355.66, 356.38, 358.46, 358.56, 359.71, 359.52];
%distances =[276, 270, 270, 269, 269, 270, 269, 270, 269, 270, 270, 270, 270, 271, 271, 273, 273, 273, 275, 275, 277, 278, 280, 280, 282, 284, 284, 286, 287, 290, 292, 294, 295, 297, 299, 300, 305, 306, 310, 312, 317, 319, 322, 328, 328, 335, 337, 341, 348, 348, 353, 362, 366, 373, 377, 384, 389, 397, 396, 398, 398, 396, 96, 383, 385, 383, 380, 379, 376, 376, 372, 373, 372, 370, 368, 368, 336, 327, 352, 352, 248, 300, 297, 357, 395, 399, 397, 389, 380, 369, 372, 368, 356, 354, 349, 343, 335, 331, 334, 326, 323, 318, 318, 312, 310, 308, 305, 302, 300, 298, 296, 293, 291, 290, 289, 284, 283, 283, 280, 279, 280, 277, 275, 275, 275, 274, 273, 272, 272, 271, 271, 271, 271];
angles = [1.72,  2.38,  3.79,  3.49,  5.36,  5.66,  7.07,  7.76,  8.75,  9.79,  11.26, 11.72, 12.05, 13.41, 14.79, 15.23, 16.42, 17.20, 18.18, 18.64, 20.12, 20.68, 21.44, 22.48, 24.09, 24.88, 25.42, 26.44, 27.73, 27.65, 29.31, 30.25, 31.19, 32.13, 32.52, 33.59, 34.94, 34.80, 36.02, 37.80, 38.33, 39.12, 40.68, 40.90, 42.19, 42.84, 44.14, 44.39, 45.76, 46.91, 47.30, 48.70, 49.87, 50.71, 51.33, 52.31, 53.05, 53.62, 55.17, 56.52, 56.72, 58.88, 59.06, 61.25, 61.64, 62.26, 63.00, 64.19, 64.82, 67.01, 67.37, 68.87, 69.32, 70.51, 71.55, 72.27, 72.48, 74.34, 75.08, 76.00, 76.98, 77.46, 78.95, 79.35, 80.21, 81.38, 82.35, 83.48, 84.16, 85.17, 86.15, 86.57, 88.24, 89.32, 90.26, 90.57, 91.39, 92.43, 93.04, 94.26, 95.56, 97.31, 96.28, 97.63, 98.94, 99.56, 100.09, 101.41, 102.81, 103.76, 104.47, 104.90, 102.36, 105.27, 107.17, 109.98, 109.74, 110.33, 111.33, 112.53, 114.10, 114.30, 115.84, 116.69, 117.30, 118.69, 119.14, 120.07, 121.13, 121.54, 123.16, 123.77, 124.62, 158.72, 162.52, 285.46, 285.89, 287.50, 289.10, 290.17, 289.79, 292.25, 293.04, 291.69, 292.94, 294.91, 296.08, 295.87, 297.53, 298.74, 300.05, 299.68, 301.58, 302.77, 301.69, 302.95, 306.06, 306.57, 305.20, 310.81, 313.42, 313.59, 314.80, 316.25, 316.70, 318.36, 318.65, 319.93, 320.57, 321.21, 321.81, 323.18, 324.56, 324.66, 325.65, 327.40, 327.17, 329.30, 330.08, 331.40, 331.95, 331.73, 333.77, 335.28, 334.78, 335.68, 337.35, 337.98, 337.48, 339.65, 341.66, 342.03, 341.74, 342.83, 344.77, 345.73, 345.50, 347.41, 349.39, 348.74, 350.00, 351.40, 351.14, 351.74, 354.11, 354.66, 355.88, 355.92, 356.84, 358.93, 358.51, 359.54];
distances = [399, 291, 291, 290, 290, 290, 290, 290, 290, 290, 290, 290, 290, 291, 291, 292, 292, 293, 293, 294, 296, 296, 298, 299, 300, 301, 303, 305, 307, 308, 310, 310, 313, 315, 319, 320, 324, 324, 327, 330, 332, 338, 342, 345, 348, 353, 358, 358, 365, 371, 375, 377, 384, 388, 393, 400, 406, 413, 420, 427, 433, 440, 449, 429, 450, 444, 439, 436, 101, 102, 418, 414, 413, 407, 405, 399, 398, 395, 392, 388, 386, 385, 383, 370, 364, 376, 374, 373, 373, 355, 345, 343, 342, 348, 447, 447, 445, 445, 445, 444, 444, 443, 443, 443, 443, 444, 444, 445, 445, 446, 447, 448, 448, 449, 452, 454, 454, 454, 457, 459, 462, 464, 464, 469, 472, 473, 475, 480, 485, 482, 489, 495, 499, 270, 106, 414, 415, 418, 443, 444, 443, 445, 448, 449, 449, 455, 455, 457, 458, 465, 465, 464, 472, 475, 476, 477, 482, 488, 493, 497, 482, 491, 489, 479, 460, 462, 456, 437, 429, 421, 424, 413, 402, 393, 397, 380, 376, 372, 367, 360, 357, 360, 348, 345, 342, 342, 341, 330, 331, 329, 321, 320, 319, 317, 312, 312, 313, 310, 306, 305, 305, 302, 301, 300, 299, 295, 296, 295, 295,  291, 290, 290];

% 将极坐标转换为直角坐标
x = distances .* cosd(angles);
y = distances .* sind(angles);

% 合并数据
data = [x; y];

% 绘制散点图
figure;
plot(data(1,:), data(2,:), 'o');
hold on;

%% 拟合圆弧
X = data;
n = length(X(:,1));
y = ones(n,1);
b = [rand(1)*1000  rand(1)  rand(1)];
fun = @(a, X) X(:,1).^2 + X(:,2).^2 + a(1)*X(:,1) + a(2)*X(:,2) + a(3);
[a,~,~,~,~] = lsqcurvefit(fun, b, X, y);
% 圆心
X1 = -a(1)/2;
Y1 = -a(2)/2;
% 半径
R = sqrt(a(1)^2 + a(2)^2 - 4*a(3))/2;

% 绘制拟合圆弧
theta = linspace(0, 2*pi, 100);
x_arc = X1 + R*cos(theta);
y_arc = Y1 + R*sin(theta);
plot(x_arc, y_arc, 'r-', 'LineWidth', 2);

title(['Best Fit Arc: (x-', num2str(X1), ')^2 + (y-', num2str(Y1), ')^2 = ', num2str(R^2)]);

% 绘制圆心和半径
plot(X1, Y1, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot([X1 X1+R*cos(pi/4)], [Y1 Y1+R*sin(pi/4)], 'k--', 'LineWidth', 1.5);

axis equal;



