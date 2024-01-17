function is_source = check_source(row, col, mea_evi_grid)
% Check whether (row, col) is a source
    is_source = false;
    if check_in_bound(row - 1, col, mea_evi_grid) && mea_evi_grid(row - 1, col) < mea_evi_grid(row, col)
        if check_in_bound(row + 1, col, mea_evi_grid) && mea_evi_grid(row + 1, col) < mea_evi_grid(row, col)
            if check_in_bound(row, col - 1, mea_evi_grid) && mea_evi_grid(row, col - 1) < mea_evi_grid(row, col)
                if check_in_bound(row, col + 1, mea_evi_grid) && mea_evi_grid(row, col + 1) < mea_evi_grid(row, col)
                    is_source = true;
                end
            end
        end
    end
end

function is_in_bound = check_in_bound(row, col, mea_evi_grid)
    if row > 0 && row < size(mea_evi_grid, 1) && col > 0 && col < size(mea_evi_grid, 2) && mea_evi_grid(row, col) ~= -1
        is_in_bound = true;
    else
        is_in_bound = false;
    end
end

