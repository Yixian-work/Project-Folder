function[p] = build_logic(s)
    %%%%UNUSED%%%%
    
    % s: input (1*9): flattened 3*3 grid space nearby surrounding measurement y
    % p: output: the probability of measurement given s
    if any(s) == 1
        p = 0.95;
    else
        p = 0.05;
    end
    return

    if s(5) == 1
        p = 0.99;
    elseif any([s(2) s(4) s(6) s(8)] == 1)
        p = 0.05;
    elseif any([s(1) s(3) s(7) s(9)] == 1)
        p = 0.025;
    else 
        p = 0;
    end
end
