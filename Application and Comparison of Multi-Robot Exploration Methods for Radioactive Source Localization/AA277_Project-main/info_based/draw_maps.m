function [frame] = draw_maps(source_map,occ_map,x_rs,source)
    clf;
    subplot(1,2,1);
    %show(source_map); hold on;
    imagesc(flip(source_map.occupancyMatrix),[0 1]); hold on;
    set(gca,'YDir','normal')
    cb = colorbar;
    cb.Label.String ='P(s)';
    title('Posterior')
    robot_c = [0.466 0.674 0.188];
    scatter(x_rs(1,:),x_rs(2,:),50,robot_c,'filled');
    axis square
    xlabel('x (meters)')
    ylabel('y (meters)')
    ax = gca;
    ax.XTick = [0:6:30];
    ax.XTickLabel={'0','2','4','6','8','10'};
    ax.YTick = [0:6:30];
    ax.YTickLabel={'0','2','4','6','8','10'}

    
    subplot(1,2,2);
    title('Map')
    show(occ_map); hold on;
    scatter(x_rs(1,:),x_rs(2,:),50,robot_c,'filled');
    scatter(source(1,:),source(2,:),50,'r','*','LineWidth',2);
    legend('robots','sources')
    ax = gca;
    ax.XTick = [0:6:30];
    ax.XTickLabel={'0','2','4','6','8','10'};
    ax.YTick = [0:6:30];
    ax.YTickLabel={'0','2','4','6','8','10'}
    drawnow
    frame = getframe(gcf);
