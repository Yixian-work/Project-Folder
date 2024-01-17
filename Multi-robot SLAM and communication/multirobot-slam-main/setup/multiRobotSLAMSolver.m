classdef multiRobotSLAMSolver < handle
    properties
        
    end
    
    properties (Access=private)
        algo_name % name of algorithm specified by user
    end
    
    methods
        function solve(obj, mrs, algo_name)
            % top level function for running algorithms specified by
            % algo_name
            
            % check that mrs is an multiRobotSLAM object
            validateattributes(mrs, {'multiRobotSLAM'}, {});
            % check that algo_name is a string
            validateattributes(algo_name, {'string'}, {});
            
            obj.algo_name = algo_name;
            
            switch algo_name
                case "EKF"
                    disp("running EKF...")
                    obj.run_EKF(mrs)
                otherwise
                    error(strcat("Algorithm ", algo_name, " not implemented!"));
            end
        end
    end
    
    methods (Access=private)
        function run_EKF(obj, mrs)
            
        end
    end
end