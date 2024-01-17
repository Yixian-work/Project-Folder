classdef driveWheelRobotController
    properties(Access=private)
        obs_cost
        stay_cost
        N
        X
        Y
    end

    methods(Access=private)
        function Z = add_point(obj, Z, x, y, kernel, N)
            % helper function for controller
            x = min(N,max(1,round(x)));
            y = min(N,max(1,round(y)));
            Z_new = zeros(N,N); Z_new(x,y) = 1;
            Z_new = conv2(Z_new, kernel, 'same');
            Z = max(Z,Z_new);
        end
    end

    methods
        function obj = driveWheelRobotController()
            % constructor for controller
            N = 200;
            x = linspace(0, 25, N);
            y = linspace(0, 25, N);
            [obj.X, obj.Y] = meshgrid(x,y);
            obj.obs_cost = 0 .* obj.X + 0 .* obj.Y;
            obj.stay_cost = 0 .* obj.X + 0 .* obj.Y;
            obj.N = N;
        end

        function [new_state] = where_to_go(obj, state, lidar_data)
            % where the robot wants to go next
            validateattributes(lidar_data, {'lidarData'}, {});
            ranges = lidar_data.ranges;
            angles = lidar_data.angles;

            obstacles = [ranges.*cos(angles+state(3))+state(1),ranges.*sin(angles+state(3))+state(2)];
            
            ti = linspace(0, 2*pi,25);
            step = 0.5;
            r = 1;
            gau_kernel = fspecial('gaussian',round(obj.N/10),5)*10000;
            
            for i = 1:size(obstacles,1)
                obj.obs_cost = obj.add_point(obj.obs_cost, obstacles(i,1)*obj.N/25, obstacles(i,2)*obj.N/25, gau_kernel, obj.N);
            end
            
            new_stay_cost = (3*exp(-(obj.X-state(1)).^2./r).*exp(-(obj.Y-state(2)).^2./r))';
            obj.stay_cost = obj.stay_cost*0.999 + new_stay_cost;
            Z = obj.obs_cost+obj.stay_cost;
            nearby_X = cos(ti).*step+state(1); nearby_Y = sin(ti).*step+state(2);
            legal = intersect(find(nearby_X>0 & nearby_X < 25),find(nearby_Y>0 & nearby_Y < 25));
            nearby_X = nearby_X(legal);nearby_Y = nearby_Y(legal);
            nearby_potential = interp2(obj.Y',obj.X',Z',nearby_X',nearby_Y');
            [min_pot, min_idx] = min(nearby_potential);
            dx = nearby_X(min_idx)-state(1);
            dy = nearby_Y(min_idx)-state(2);

            new_state = zeros(2, 1);
            new_state(1) = state(1)+dx;
            new_state(2) = state(2)+dy;
        end
        
        function control_input = get_control_input(obj, state, newPos, kv, kphi)
                % This function is used for the pointRobotDynamic, where the second input
                % is phi (angular velocity of the point robot). We utilized a simple
                % porpotional control policy to generate a policy at each state state and
                % desired position.
                % We suppose the robot could only move forward.
                % INPUT: state: Current position and direction (3*1 vector)
                %        newPos: Desired Position (2*1 vector, independent of theta)
                %        kv: veloctiy gain tuable (Scalar)
                %        kphi: portional angular velocity gain (Scalar)
                % OUTPUT: v: linear velocity
                %         phi: angular velocity
                assert(3 == size(state, 1))
                assert(1 == size(state, 2))
                assert(2 == size(newPos, 1))
                assert(1 == size(newPos, 2))
                validateattributes(newPos, {'double'}, {});
                validateattributes(kv, {'double'}, {});
                validateattributes(kphi, {'double'}, {});

                if abs(wrapToPi(atan2(newPos(2)-state(2), newPos(1)-state(1)) - state(3))) > pi/2
                    v = 0;
                else
                    v = kv/(abs(wrapToPi(atan2(newPos(2)-state(2), newPos(1)-state(1)) - state(3))) + kv);
                end
                phi = wrapToPi(atan2(newPos(2)-state(2), newPos(1)-state(1)) - state(3))*kphi;
                control_input = [reshape(v,[],1); reshape(phi,[],1)];
        end
    end
end