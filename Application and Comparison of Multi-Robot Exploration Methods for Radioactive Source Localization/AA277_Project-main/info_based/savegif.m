function [] = savegif(images,filename)
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
