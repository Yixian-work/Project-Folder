function cat_grid = make_cat_grid(mea_evi_grid)
% evi_grid: a matrix that is the evidence grid of radiation measurements.
% If it's a wall or if the measurement is unknown, the value is -1.
    if all(all(mea_evi_grid == -1)) % Edge case for no measurements taken yet
        cat_grid = mea_evi_grid;
    else
        cat_grid = (mea_evi_grid == -1).*(-1);
        cat = 0; % Number of categories
        while any(any(cat_grid == 0)) % Check if categorization is done
            cat = cat + 1;
            max_mea = max(mea_evi_grid(cat_grid == 0), [], 'all');
            max_mea_idx = find(mea_evi_grid == max_mea, 1);
            cat_grid(max_mea_idx) = 0.5; % 0.5 means the grid is "to be checked"
            while any(any(cat_grid == 0.5))
                to_be_checked_idx = find(cat_grid == 0.5, 1);
                cat_grid(to_be_checked_idx) = cat;
                [to_be_checked_y, to_be_checked_x] = ind2sub(size(cat_grid), to_be_checked_idx);
                cat_grid = cat_grid_update(cat_grid, mea_evi_grid, to_be_checked_y, to_be_checked_x, to_be_checked_y - 1, to_be_checked_x);
                cat_grid = cat_grid_update(cat_grid, mea_evi_grid, to_be_checked_y, to_be_checked_x, to_be_checked_y + 1, to_be_checked_x);
                cat_grid = cat_grid_update(cat_grid, mea_evi_grid, to_be_checked_y, to_be_checked_x, to_be_checked_y, to_be_checked_x - 1);
                cat_grid = cat_grid_update(cat_grid, mea_evi_grid, to_be_checked_y, to_be_checked_x, to_be_checked_y, to_be_checked_x + 1);
            end
        end
    end
end

function cat_grid_out = cat_grid_update(cat_grid_in, mea_evi_grid, to_be_checked_y, to_be_checked_x, next_check_y, next_check_x)
    cat_grid_out = cat_grid_in;
    if next_check_y > 0 && next_check_y <= size(cat_grid_in, 1) && next_check_x > 0 && next_check_x <= size(cat_grid_in, 2) % Check if indexing is valid
        if cat_grid_in(next_check_y, next_check_x) == 0 && mea_evi_grid(next_check_y, next_check_x) <= mea_evi_grid(to_be_checked_y, to_be_checked_x)
            cat_grid_out(next_check_y, next_check_x) = 0.5;
        end
    end
end