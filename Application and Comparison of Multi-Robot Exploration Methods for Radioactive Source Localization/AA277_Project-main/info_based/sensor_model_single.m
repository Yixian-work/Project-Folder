function[p_y_s] = sensor_model_single()
    % get sensor params
    sensor_params;

    p_y_s = zeros(1,sensor_num_states);
    p_y_s(1) = sensor_prob_0;
    p_y_s(2:end) = sensor_prob_1;
    %for num = 1:2^9
    %    binstr = dec2bin(num-1, 9);
    %    s = (binstr == '1');
    %    p_y_s(num) = build_logic(s);
    %end
end
