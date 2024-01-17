clear
addpath(genpath('../'))
% single_robot exploration demo
% Initialization
MAX_TIME_STEP = 200;
[occ_map, source_map, x_rs, source] = init_env_multi_random();

plt = Plotter();

p_y_s = sensor_model_single(); 

for t = 1:MAX_TIME_STEP
    for i=1:size(x_rs,2)
        x_r = x_rs(:,i);
        y = generate_measurement_single(x_r, source);
        posterior = calculate_posterior(x_r, y, source_map, p_y_s);
        [source_map] = update_map(x_r, posterior, source_map);
        plt.info_based_fig(source_map,occ_map,x_rs,source);
        a_r = choose_action(x_r, occ_map, source_map, p_y_s); 
        x_r = x_r + a_r;
        x_rs(:,i) = x_r
    end
    plt.savegif('test.gif')
end
