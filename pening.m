clear all;clc
dt = 0.01; %10ms
% dt = .05;
% %%Kandidat delta_theta dan delta_vel
n = 2; %pilih jumlah kandidat
%Posisi Target 
X_c = -4;
X_r = 5;
Y_c = 6;
Y_r = -2;
theta_c = 0; %%hadap utara
% theta_r = theta_c+CalcHeading(X_c,Y_c,X_r,Y_r);
theta_r = CalcHeading(X_c,Y_c,X_r,Y_r);
X_obs = [0 -2 -4 8];
Y_obs = [1 3 -2 0];
% w = zeros(3,1);
% v = zeros(3,1);
v = 0;
w = 0;
acc = 0.1;
wheel_radius = 0.5;
for i = 1:5000
    theta_r(i) = CalcHeading(X_c(i),Y_c(i),X_r,Y_r); %%relatif ke arah Timur (searah sumbu X)
    theta_error(i) = theta_r(i)-theta_c(i);
    delta_error(:,i) = [theta_c(i):theta_error(i)/n:theta_r(i)];
    rho(i) = sqrt((X_r-X_c(i))^2+(Y_r-Y_c(i))^2);
    rho_obs(:,i)=sqrt((X_obs(:,1)-X_c(i))^2+(Y_obs(:,1)-Y_c(i))^2);
    v(i+1) = .1*rho(i)*cos(theta_error(i));
    w(i+1) = theta_error(i)+.1*sin(theta_error(i))*cos(theta_error(i));
%     v(:,i) = rho(i).*cos(delta_error(:,i));
%     w(:,i) = delta_error(:,i)+(cos(delta_error(:,i)).*sin(delta_error(:,i)));
%     v(i+1) = max(rho(i)*cos(delta_error(i)));
%     w(i+1) = max(delta_error(:,i)+sin(delta_error(:,i)).*cos(delta_error(:,i)));
    theta_c(i+1) = theta_c(i) + w(i)*dt;
    X_c(i+1) = X_c(i)+v(i)*cos(theta_c(i))*dt;
    Y_c(i+1) = Y_c(i)+v(i)*sin(theta_c(i))*dt;
%     end
end
%% %% Plot
figure(1);clf
plot(X_obs,Y_obs,'ob','LineWidth',1);hold on;
plot(X_r,Y_r,'*k','LineWidth',10);hold on;
plot(X_c,Y_c,'r','LineWidth',1);hold on;
xlim([-10 10])
ylim([-10 10])
xlabel('X axis')
ylabel('Y axis')
grid on
