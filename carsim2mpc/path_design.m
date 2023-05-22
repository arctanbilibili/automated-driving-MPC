%% design path from points by spline
% 注意速度限制 = sqrt(μ * R * g)
map = load('map.txt');
path = load('path.txt');

gridSize = 5;
point = path*gridSize;

s = 1:1:length(point);

dt = 0.1;
%直接参数方程式插值
px_spline = spline(s, point(:,1), 1:dt:length(point));
py_spline = spline(s, point(:,2), 1:dt:length(point));

p_spline = [px_spline', py_spline'];

n = length(px_spline);
dy_dt = zeros(n, 1);
dx_dt = zeros(n, 1);
d2y_dt2 = zeros(n, 1);
d2x_dt2 = zeros(n, 1);

dx_dt(1:n-1) = diff(px_spline)/dt;
dx_dt(n) = dx_dt(n-1);

dy_dt(1:n-1) = diff(py_spline)/dt;
dy_dt(n) = dy_dt(n-1);

d2x_dt2(1:n-1) = diff(dx_dt)/dt;
d2x_dt2(n) = d2x_dt2(n-1);

d2y_dt2(1:n-1) = diff(dy_dt)/dt;
d2y_dt2(n) = d2y_dt2(n-1);

%带符号kxy
kxys = (dx_dt .* d2y_dt2 - dy_dt .* d2x_dt2) ./ (dx_dt.^2 + dy_dt.^2).^(3/2);
%不带符号kxy
kxy = abs(dx_dt .* d2y_dt2 - dy_dt .* d2x_dt2) ./ (dx_dt.^2 + dy_dt.^2).^(3/2);



%% insert yaw

yaw = zeros(length(px_spline), 1);
for i = 2:length(px_spline)-1
    x_forward = px_spline(i+1);
    x_backward = px_spline(i-1);
    y_forward = py_spline(i+1);
    y_backward = py_spline(i-1);
    yaw(i) = atan2(y_forward-y_backward, x_forward-x_backward);
end
%左右加点
yaw(1) = yaw(2);
yaw(end) = yaw(end-1);

%输出路径,变成行向量用
xr=px_spline';
yr=py_spline';
thetar=yaw;
kappar=kxys;

%显示
figure(1)
subplot(3,1,1)
title('曲率图')
hold on
plot(px_spline, py_spline,'b-');
quiver(px_spline', py_spline', -sin(yaw).*kxys, cos(yaw).*kxys);
scatter(point(:,1),point(:,2),'k*')
grid on; hold off;

arrow_scale = 0.01;
subplot(3,1,2)
title('方向图')
hold on
%plot(px_spline, py_spline,'b-');
quiver(px_spline', py_spline', cos(yaw)*arrow_scale, sin(yaw)*arrow_scale);
grid on; hold off;

subplot(3,1,3)
title('曲率数值，弯道处速度限制 <= sqrt(μ * R * g) m/s')
hold on
plot(kxy,'b-');
grid on; hold off;

%绘制圆形障碍物的地图
figure(114)
[m,n] = size(map);
xlim([-gridSize (n+1)*gridSize]);
ylim([-gridSize (m+1)*gridSize]);
hold on
for i=1:m
    for j=1:n
        if(map(i,j)==1)
            rectangle('Position', [(i-1)*gridSize-gridSize/2,(j-1)*gridSize-gridSize/2, gridSize, gridSize], 'Curvature', [1, 1], 'FaceColor', 'k');
            %rectangle('Position', [(i-1)*gridSize-gridSize/2,(j-1)*gridSize-gridSize/2, gridSize, gridSize], 'FaceColor', 'k');
        end
    end
end
plot(px_spline, py_spline,'r-');
hold off