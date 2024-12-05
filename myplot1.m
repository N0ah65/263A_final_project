function [] = myplot1(points)
points=points;
figure(1);
hold on;
    for i = 1:size(points, 1)-1
        plot3(points(i:i+1, 1), points(i:i+1, 2), points(i:i+1, 3), 'b', 'LineWidth', 2);
    end
end