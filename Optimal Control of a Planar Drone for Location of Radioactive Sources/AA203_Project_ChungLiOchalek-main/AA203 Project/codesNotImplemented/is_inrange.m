function [in_range] = is_inrange(x, r, ctr)
%% Given a circular obstacle, check if the given state is within the obstacle or not.
    dist = norm(x-ctr);
    if dist <= r
        in_range = true;
    else
        in_range = false;
    end
end