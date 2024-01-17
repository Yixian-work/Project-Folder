classdef lidarData < handle
    properties
        ranges
        angles
    end

    methods
        function obj = lidarData(ranges, angles)
            % constructor
            obj.ranges = ranges;
            obj.angles = angles;
        end
    end
end