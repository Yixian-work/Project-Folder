n = 100;
m = 100;
mea_evi_grid = zeros(n, m);
y_1 = 0.258.*n;
x_1 = 0.148.*m;
k_1 = 100;
y_2 = 0.788.*n;
x_2 = 0.699.*m;
k_2 = 150;
y_3 = 0.522.*n;
x_3 = 0.387.*m;
k_3 = 150;

for i = 1:size(mea_evi_grid, 1)
    for j = 1:size(mea_evi_grid, 2)
        mea_evi_grid(i, j) = k_1./((x_1 - i).^2 + (y_1 - j).^2) + k_2./((x_2 - i).^2 + (y_2 - j).^2) + k_3./((x_3 - i).^2 + (y_3 - j).^2);
    end
end
mea_evi_grid(:, 1) = -1;
mea_evi_grid(1, :) = -1;
mea_evi_grid(end, :) = -1;
mea_evi_grid(:, end) = -1;
mea_evi_grid(50:60, 10:30) = -1;
mea_evi_grid(50:80, 50:60) = -1;

log_mea_grid = mea_evi_grid;
for i = 1:size(mea_evi_grid, 1)
    for j = 1:size(mea_evi_grid, 2)
        if log_mea_grid(i, j) ~= -1
            log_mea_grid(i, j) = log(log_mea_grid(i, j));
        else
            log_mea_grid(i, j) = 8;
        end
    end
end

figure();
heatmap(log_mea_grid);

cat_grid = make_cat_grid(mea_evi_grid);
figure();
heatmap(cat_grid);