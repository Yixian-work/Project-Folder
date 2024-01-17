function [next_row, next_col] = gradient_step(mea_evi_grid, occ_evi_grid, cur_row, cur_col, cat_grid, cat)
% Gives the next grid (in terms of row and col) to go to based on the
% current row and col, the evidence grid AS A MATRIX (mea_evi_grid), and the gradient
% that the robot is following (cat)

    cost_list = [0, 0, 0, 0]; % Up, down, left, right
    next_row_list = [cur_row - 1, cur_row + 1, cur_row, cur_row];
    next_col_list = [cur_col, cur_col, cur_col - 1, cur_col + 1];

    for i = 1:length(cost_list)
        next_row_can = next_row_list(i);
        next_col_can = next_col_list(i);
        if check_valid_grid(next_row_can, next_col_can, occ_evi_grid) % Check that it's not out of bounds and not an obstacle
            if cat_grid(next_row_can, next_col_can) ~= -1 && cat_grid(next_row_can, next_col_can) ~= cat
                cost_list(i) = inf; % Never travel to a grid that's not part of the gradient the robot's following
            else
                if mea_evi_grid(next_row_can, next_col_can) ~= -1 % If there's measurement in the candidate, use it to calculate gradient directly
                    cost_list(i) = mea_evi_grid(cur_row, cur_col) - mea_evi_grid(next_row_can, next_col_can);
                else % Otherwise, check if the opposite grid has measurement, and use the negative of that as our gradient
                    if mod(i, 2) == 0
                        opp_row = next_row_list(i - 1);
                        opp_col = next_col_list(i - 1);
                    else
                        opp_row = next_row_list(i + 1);
                        opp_col = next_col_list(i + 1);
                    end
                    if check_valid_grid(opp_row, opp_col, occ_evi_grid) && mea_evi_grid(opp_row, opp_col)~= -1
                        cost_list(i) = mea_evi_grid(opp_row, opp_col) - mea_evi_grid(cur_row, cur_col);
                    end
                end
            end
        else
            cost_list(i) = inf; % Never travel to a grid that's out of bounds / an obstacle
        end
    end
    [~, best_action_state] = min(cost_list);
    next_row = next_row_list(best_action_state);
    next_col = next_col_list(best_action_state);
end

function is_valid_grid = check_valid_grid(next_row, next_col, occ_evi_grid)
    if next_row > 0 && next_row < size(occ_evi_grid, 1) && next_col > 0 && next_col < size(occ_evi_grid, 2) && occ_evi_grid(next_row, next_col) ~= 1
        is_valid_grid = true;
    else
        is_valid_grid = false;
    end
end