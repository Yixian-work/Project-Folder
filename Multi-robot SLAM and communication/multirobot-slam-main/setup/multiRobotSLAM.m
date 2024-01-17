classdef multiRobotSLAM < handle
    % private member variables
    properties (Access = private)
        map_2d_row % number of rows in 2d map
        map_2d_col % number of cols in 2d map
        n_robots % number of robots
        true_map % array of true map using occupancy representation
        dwrobots = driveWheelRobot(zeros(3,1), zeros(3,1), zeros(3,3)) % array of robots
        delta_t % time step
    end
    
    % public member functions
    methods
        function obj = multiRobotSLAM(true_map, cc)
            % constructor takes in a true_map and number of robots
            assert(isinteger(cc.n_robots));
            validateattributes(true_map, {'binaryOccupancyMap', 'occupancyMap'}, {});
            validateattributes(cc, {'constantsCollection'}, {});
            
            rng(12312);
            
            map_class = class(true_map);
            
            obj.map_2d_row = true_map.GridSize(1);
            obj.map_2d_col = true_map.GridSize(2);
            obj.n_robots = cc.n_robots;
            obj.true_map = true_map.copy();
            obj.delta_t = cc.delta_t;
            % initialize robots
            obj.dwrobots(1:cc.n_robots) = driveWheelRobot(zeros(3,1), zeros(3,1), zeros(3,3));
            init_states = [[3,3,-1];[10,4,0.5];[17,16,1]];
            for i_r = 1:length(obj.dwrobots)
                obj.dwrobots(i_r) = driveWheelRobot(init_states(i_r,:)', [5;5;0], cc.process_noise);
                % initialize estimated map
                obj.dwrobots(i_r).set_map(eval(strcat(map_class, "(rand(true_map.GridSize), true_map.Resolution)")));
            end
        end
        
        function map = get_true_map(obj)
            % return true map
            map = obj.true_map.copy();
        end
        
        function robot = get_robot(obj, i_robot)
            % return robot at index i_robot
            robot = obj.dwrobots(i_robot);
        end
        
        function plot_all_state_history(obj, savefig)
            % plot all state history for all robots
            figure
            worldAx = subplot(1,1,1);
            show(obj.get_true_map(), 'Parent', worldAx);
            hold on
            for i_r = 1:length(obj.dwrobots)
                true_state_history = obj.dwrobots(i_r).get_true_state_history();
                plot(true_state_history(:,1), true_state_history(:,2), '-', 'DisplayName', strcat("robot ", num2str(i_r)))
            end
            hold off
            axis equal
            legend('Location', 'bestoutside')
            plotfixer
            
            if savefig
                saveas(gcf, './plots/all_state_history.png')
            end
        end
        
        function all_systems_evolve(obj)
            % evolve all robots for one time step delta_t
            [measurements] = obj.get_all_measurements();
            for i_r = 1:length(obj.dwrobots)
                measurement = measurements{i_r};
                want_to_go = obj.dwrobots(i_r).where_to_go(measurement);
                control_input = obj.dwrobots(i_r).get_control_input(want_to_go, 1, 1);
                obj.dwrobots(i_r).system_evolve(obj.delta_t, control_input);
            end
        end
        
        function [measurements] = get_all_measurements(obj)
            % return all lidar measurements
            % measurements: cell array of lidarData class
            true_map = obj.get_true_map();
            measurements = cell(length(obj.dwrobots), 1);
            for i_r = 1:length(obj.dwrobots)
                measurement = obj.dwrobots(i_r).measure(true_map);
                measurements{i_r, 1} = measurement;
            end
        end
        
        function log_probs = log_prob_of_measurements(obj, measurements)
            % return the log likelihood of observing 'measurements' at the
            % estimated map and pose
            % measurements: cell array of lidarData
            validateattributes(measurements, {'cell'}, {})
            validateattributes(measurements{1}, {'lidarData'}, {})
            log_probs = zeros(length(measurements), 1);
            for i_r = 1:length(obj.dwrobots)
                log_probs(i_r) = log_probs(i_r) + obj.dwrobots(i_r).log_prob_of_measurement(measurements{i_r});
            end
        end
        
        function network_communicate(obj, d_threshold)
            % robots communicate with each other
            % exchange map information if within certain distance
            % d_threshold: distance threshold for communication
            error("not implemented")
            pairwise_d = obj.get_all_distances();
            adj_mat = pairwise_d <= d_threshold;
            for i_r = 1:size(adj_mat, 1)
                new_map = zeros(size(obj.dwrobots(i_r).get_map()));
                count = 1;
                for j_r = 1:size(adj_mat, 2)
                    % if within distance d_threshold then accumulate map
                    if adj_mat(i_r, j_r)
                        map_from_others = obj.dwrobots(j_r).get_map();
                        new_map = new_map*((count-1)/count)+map_from_others/count;
                        count = count + 1;
                    end
                end
                obj.dwrobots(i_r).set_map(new_map);
            end
        end
        
        function debug(obj)
            % print some information to help debugging
            for i_r = 1:length(obj.dwrobots)
                robot = obj.dwrobots(i_r);
                r_state = robot.get_state();
                r_x = r_state(1);
                r_y = r_state(2);
                r_angle = r_state(3, 1);
                fprintf("robot %d\n\testimated state (%.3f, %.3f), %.3f rad", i_r, r_x, r_y, r_angle)
                r_true_state = robot.get_true_state();
                r_x = r_true_state(1);
                r_y = r_true_state(2);
                r_angle = r_true_state(3, 1);
                fprintf("\ttrue state (%.3f, %.3f), %.3f rad\n", r_x, r_y, r_angle)
            end
        end
    end
    
    % private member functions
    methods (Access = private)
        function pairwise_d = get_all_distances(obj)
            % helper function to compute the pairwise distance
            all_positions = zeros(length(obj.dwrobots), 2);
            for i_r = 1:length(obj.dwrobots)
                robot = obj.dwrobots(i_r);
                r_state = robot.get_state();
                r_position = r_state(1:2, 1);
                all_positions(i_r, :) = r_position;
            end
            pairwise_d = pdist2(all_positions, all_positions);
        end
    end
end