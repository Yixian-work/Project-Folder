function[MI] = calculate_MI(x_r, source_map, p_y_s)
    p_s = calculate_prior(x_r, source_map);
    H_s = -dot(p_s,log_0(p_s));
    p_y0 = dot(1.-p_y_s, p_s);
    p_y1 = dot(p_y_s, p_s);
    p_s_y0 = calculate_posterior(x_r, 0, source_map, p_y_s);
    p_s_y1 = calculate_posterior(x_r, 1, source_map, p_y_s);
    H_s_y = -(p_y0 * dot(p_s_y0, log_0(p_s_y0)) + p_y1 * dot(p_s_y1, log_0(p_s_y1)));
    MI = H_s - H_s_y;
end

function val = log_0(x)
    x(x==0) = 1;
    val = log2(x);
end
