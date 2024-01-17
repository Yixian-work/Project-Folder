classdef driveWheelRobot < handle
    properties (Access = private)
        true_state % true state in 2D map, shape 3x1
        state % estimated state in 2D map, shape 3x1
        rangefinder = rangeFinderSensor % range finder sensor
        controller = driveWheelRobotController % controller
        map % array of estimated map using occupancy representation
        odometry % odometry measurement
        true_state_history % robot state history
        process_noise = eye(3) % process noise
    end
    
    methods (Access = private)
        function set_true_state(obj, new_true_state)
            % update true state
            % private function
            obj.true_state = new_true_state;
        end
        
        function update_odometry(obj)
            % update odometry measurement
        end
    end
    
    methods
        function obj = driveWheelRobot(init_state, init_guess, process_noise)
            % constructor for robot
            assert(1 == size(init_state, 2));
            assert(1 == size(init_guess, 2));
            obj.true_state = init_state;
            obj.state = init_guess;
            obj.true_state_history = obj.true_state';
            obj.process_noise = process_noise;
        end
        
        function state = get_state(obj)
            % get state
            state = obj.state;
        end

        function true_state = get_true_state(obj)
            % get true state
            true_state = obj.true_state;
        end
        
        function map = get_map(obj)
            % return estimated map
            map = obj.map;
        end
        
        function set_map(obj, map)
            % update estimated map
            obj.map = map;
        end
        
        function [true_state_history] = get_true_state_history(obj)
            true_state_history = obj.true_state_history;
        end
        
        function plot_true_state_history(obj)
            figure
            plot(obj.true_state_history(:, 1), obj.true_state_history(:, 2), '-o')
            plotfixer
        end
        
        function new_true_state = system_evolve_noise_less(obj, delta_t, control_input)
            % evolve robot state for one time step without noise
            %   INPUTS:
            %   delta_t: time step
            %   control_input: control inputs
            %       control_input(1): linear velocity of robot's COM
            %       control_input(2): angular velocity w.r.t robot's COM
            
            % update odometry reading
            obj.update_odometry()
            curr_true_state = obj.get_true_state();
            theta = curr_true_state(3);  % current angle
            speed = control_input(1);  % control - speed
            phi = control_input(2);  % control - angular velocity

            % set new state
            new_true_state = curr_true_state + delta_t * [speed*cos(theta), speed*sin(theta), phi]';
        end
        
        function new_true_state = system_evolve(obj, delta_t, control_input)
            % evolve robot state for one time step with noise
            %   INPUTS:
            %   delta_t: time step
            %   control_input: control inputs
            %       control_input(1): linear velocity of robot's COM
            %       control_input(2): angular velocity w.r.t robot's COM
            new_true_state = obj.system_evolve_noise_less(delta_t, control_input);
            % CHANGE this line for additive noise
            dim_state = numel(new_true_state);
            assert(dim_state == size(obj.process_noise, 1));
            assert(dim_state == size(obj.process_noise, 2));
            new_true_state = new_true_state + mvnrnd(zeros(1, dim_state), 0.1*obj.process_noise)';
            
            obj.set_true_state(new_true_state);
            obj.true_state_history = [obj.true_state_history; new_true_state'];
            
            % use column vector
            assert(1 == size(obj.true_state, 2));
        end
        
        function [measurement] = measure(obj, map)
            %   INPUTS:
            %   map - Ground truth map, specified as an occupancyMap (Navigation Toolbox) 
            %   or a binaryOccupancyMap (Navigation Toolbox) object. For the occupancyMap 
            %   input, the range-bearing sensor considers a cell as occupied and returns a 
            %   range reading if the occupancy probability of the cell is greater than 
            %   the value specified by the OccupiedThreshold (Navigation Toolbox) 
            %   property of the occupancy map.
            % 
            %   Outputs: returns the range and angle readings from the 2-D pose information and the ground-truth map.
            %   ranges - R-by-N real-valued matrix. 
            %       N (here is 1) is the number of poses for which the sensor is simulated, 
            %       and R is the number of sensor readings per pose of the sensor. 
            %       R is same as the value of the sensor's NumReadings property.
            %   angles - Angle readings, specified as an R-by-1 real-valued vector. 
            %       R is the number of sensor readings per pose of the sensor. 
            %       R is same as the value of the NumReadings property.
            [ranges, angles] = obj.rangefinder.measure(obj.get_true_state(), map);
            measurement = lidarData(ranges, angles);
        end
        
        function log_prob = log_prob_of_measurement(obj, measurement)
            % measurement: lidarData object
            validateattributes(measurement, {'lidarData'}, {});
            m_ranges = measurement.ranges;
            m_angles = measurement.angles;
            % get estimated state
            est_state = obj.get_state();
            % % robot angle
            % r_angle = est_state(3);
            % index = binary_search(m_angles, r_angle);
            % % rotate robot frame
            % m_ranges = [m_ranges(1:index-1); m_ranges(index:length(m_ranges))];
            % get estimated map
            est_map = obj.get_map();
            [est_ranges, ~] = obj.rangefinder.measure(est_state, est_map);
            select_not_nan = (~isnan(m_ranges)) & (~isnan(est_ranges));
            % only keep not nan
            est_ranges = est_ranges(select_not_nan);
            m_ranges = m_ranges(select_not_nan);
            % accumulate log probabilities
            log_prob = 0.;
            for i = 1:length(est_ranges)
                log_prob = log_prob - normlike([est_ranges(i), 1], m_ranges(i));
            end
        end
        
        function control_input = get_control_input(obj, newPos, kv, kphi)
            % get control input from current state and map
            curr_state = obj.get_state(); % the current state
            % curr_map = obj.get_map(); % the estimated map
            control_input = obj.controller.get_control_input(curr_state, newPos, kv, kphi);
        end

        function want_to_to = where_to_go(obj, measurement)
            % return [x; y] which is where the robot wants to go
            want_to_to = obj.controller.where_to_go(obj.get_state(), measurement);
        end
    end
end