classdef rangeFinderSensor
    properties (Access=private)
        lidar = rangeSensor % a rangeSensor object from Navigation Toolbox
    end
    methods
        function obj = rangeFinderSensor()
            obj.lidar.Range = [0 5]; % TODO(@kexinweng) TBD
            obj.lidar.RangeNoise = 0; % TODO(@kexinweng) TBD
        end
        
        function [ranges, angles] = measure(obj, state, map)
            %   INPUTS:
            %   state - sensor (vehicle) pose [x; y; theta]
            %       use TRUE state
            %   map - Ground truth map, specified as an occupancyMap (Navigation Toolbox) 
            %       or a binaryOccupancyMap (Navigation Toolbox) object. For the occupancyMap 
            %       input, the range-bearing sensor considers a cell as occupied and returns a 
            %       range reading if the occupancy probability of the cell is greater than 
            %       the value specified by the OccupiedThreshold (Navigation Toolbox) 
            %       property of the occupancy map.
            %
            %   Outputs: returns the range and angle readings from the 2-D pose information and the ground-truth map.
            %   ranges - R-by-N real-valued matrix. 
            %       N (here is 1) is the number of poses for which the sensor is simulated, 
            %       and R is the number of sensor readings per pose of the sensor. 
            %       R is same as the value of the sensor's NumReadings property.
            %   angles - Angle readings, specified as an R-by-1 real-valued vector. 
            %       R is the number of sensor readings per pose of the sensor. 
            %       R is same as the value of the NumReadings property.

            % create sensor
            assert(1 == size(state, 2)); % state must be a column vector 
            pos = state';
            [ranges, angles] = obj.lidar(pos, map);
        end
    end
end