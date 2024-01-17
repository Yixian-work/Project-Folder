function [target_id] = binary_search(numbers, target)
% numbers: must be monotonic
assert(all(diff(numbers)>0))

if target <= numbers(1)
    target_id = 1;
    return
elseif target >= numbers(end)
    target_id = length(numbers);
    return
end

i_s = 0;
i_b = length(numbers);

while i_s+1 < i_b
    i_m = int32((i_s+i_b)/2);
    if numbers(i_m) >= target
        i_b = i_m;
    else
        i_s = i_m;
    end
end
target_id = i_b;
end