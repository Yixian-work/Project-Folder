classdef Plotter < handle & matlab.System
    properties
        images = {};
    end

    methods
        function [frame] = info_based_fig(obj,source_map,occ_map,x_rs,source)
            frame = draw_maps(source_map,occ_map,x_rs,source);
            obj.images{end+1} = frame2im(frame);
        end

        function [frame] = plot_frontiers(obj,map_plot)
            frame = plot_frontier(map_plot);
            obj.images{end+1} = frame2im(frame);
        end


        function [] = savegif(obj,filename)
            images = obj.images;
            delay = 1/6;
            nImages = length(images);
            for i=1:nImages
                [A,map] = rgb2ind(images{i},256);
                if i==1
                    imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',delay);
                else
                    imwrite(A,map,filename,'gif','WriteMode','Append','DelayTime',delay);
                end
            end
        end
    end
end
