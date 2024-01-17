function [frame] = plot_frontier(map_plot)
    heatmap(map_plot);
    cmap = [0.5, 0.5, 0.5; 1, 1, 1; 0, 0, 0; 0.466, 0.674, 0.188; 1, 0, 0; 0 1 0];
    colormap(cmap);
    caxis([-1 4])
    Ax = gca;
    Ax.XDisplayLabels = nan(size(Ax.XDisplayData));
    Ax.YDisplayLabels = nan(size(Ax.YDisplayData));
    grid off
    colorbar('off') 
    drawnow
   
    frame = getframe(gcf)
end