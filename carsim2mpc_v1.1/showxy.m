%绘制圆形障碍物的地图
figure(115)
[m,n] = size(map);
xlim([-gridSize (n+1)*gridSize]);
ylim([-gridSize (m+1)*gridSize]);
hold on
for i=1:m
    for j=1:n
        if(map(i,j)==1)
            rectangle('Position', [(i-1)*gridSize-gridSize/2,(j-1)*gridSize-gridSize/2, gridSize, gridSize], 'Curvature', [1, 1], 'FaceColor', 'k');%显示圆形障碍物
            %rectangle('Position', [(i-1)*gridSize-gridSize/2,(j-1)*gridSize-gridSize/2, gridSize, gridSize], 'FaceColor', 'k'); %显示矩形障碍物
        end
    end
end
plot(px_spline(1:5:end), py_spline(1:5:end),'r.');
plot(xy.signals(1).values,xy.signals(2).values,'b-');
title('红点设定轨迹 - 蓝线为实际轨迹')
hold off
