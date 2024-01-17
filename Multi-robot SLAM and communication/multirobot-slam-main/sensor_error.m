clear all; close all; clc;

x1 = [-0.5,0,0.2,0.5,1,2];
y1 =  -2.*x1 + 5;
x2 = -x1;
y2 = 1.5.*x2;
x = [x1,x2]; y = [y1,y2];
[theta, rho] = cart2pol(x,y);
figure(1);
polarscatter(theta,rho);
figure(2);
scatter(theta*180/pi,rho);
xlabel('$$\theta,\circ$$','interpreter','latex');
ylabel('$$r,m$$','interpreter','latex');
p = 0.9;
r = sqrt(-2 * log(1-p));
t = linspace(0,2*pi,1000);
w = [cos(t).*r;sin(t).*r]';
for i = 1:size(x,2)
    sig = [2*pi/180,0;
           0,rho(i)*0.01];
    mu = [theta(i),rho(i)];
    X_ellipse = w * sqrtm(sig) + mu;
    figure(1)
    hold on;
    polarplot(X_ellipse(:,1),X_ellipse(:,2));
    figure(2)
    hold on;
    plot(X_ellipse(:,1)*180/pi,X_ellipse(:,2));
end
figure(1)
title('Error Ellipses for $$\sigma^2_{\theta}=2.0 ^{\circ}, \sigma^2_{r_i}=0.01r_i$$ projected in Cartesian space','interpreter','latex');
figure(2)
title('Error Ellipses for $$\sigma^2_{\theta}=2.0 ^{\circ}, \sigma^2_{r_i}=0.01r_i$$ in polar space','interpreter','latex');