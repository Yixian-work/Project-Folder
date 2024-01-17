% plot(y_trajectory, z_trajectory,'b','linewidth',1.2)
% hold on
% plot(y_history(1:end-1), z_history(1:end-1),'ob','linewidth',1)
% plot(5,5,'xr','linewidth',1.5)
% plot(y_guess_history,z_guess_history,'dg','linewidth',1)

% th = 0:0.1:2*pi;
% circle1X = obstacles(1,1) + obstacles(1,3)*cos(th);
% circle1Y = obstacles(1,2) + obstacles(1,3)*sin(th);
% plot(circle1X, circle1Y)

% xlim([0 6])
% ylim([0 7])
% xlabel("y location (m)")
% ylabel("z location (m)")
% legend("Trajectory", "Measurement taken spot","Source","Guess location","Obstacles")
% title("Trajectory with obstacles - state focused")

plot(t, F_data)
hold on

title("")