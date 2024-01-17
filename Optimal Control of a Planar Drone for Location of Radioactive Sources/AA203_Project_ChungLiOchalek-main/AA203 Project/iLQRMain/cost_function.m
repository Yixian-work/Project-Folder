function cost = cost_function(y, z, y_ans, z_ans, k, obstacles)
    cost = [];
    for i = 1:length(y)
        cost = [cost; getReadingObstacle(y(i), z(i), y_ans, z_ans, k, obstacles)];
    end
end