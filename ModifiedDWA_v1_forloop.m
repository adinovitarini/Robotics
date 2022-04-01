clear all;clc
goal = [-0.2 0.6];
dt = 0.01; %10ms
% %%Kandidat delta_theta dan delta_vel
n = 2; %pilih jumlah kandidat
%Posisi Target 
X_r = goal(1);
Y_r = goal(2);
X_c = 1;
Y_c = -0.4;
% X_obs = [0 -.2 -.14 .8];
% Y_obs = [1 .3 -0.2 0];
theta_c = 0; %%hadap utara
theta_r = CalcThetaR(X_c,Y_c,X_r,Y_r);
pose = [X_c;Y_c;theta_c];
acc_x = -0.1245;
acc_y = 0.3639;
w = zeros(2*n,1);
v = zeros(2*n,1);
delta_theta = zeros(n,1);
% disp('------------------------------------------------------------------')
% disp('Data setiap iterasi')
% disp('------------------------------------------------------------------')
% disp('|i|theta_r|theta_error|delta_theta|G_heading|')
disp('------------------------------------------------------------------')
disp('|i|X_c|Y_c|theta_c|X_r|Y_r|theta_r|v')
disp('------------------------------------------------------------------')
epsilon = 0.01;
ex = norm(X_r-X_c);
ey = norm(Y_r-Y_c);
% while ex>=1&&ey>=1
for i = 1:50   
    theta_r(i) = CalcThetaR(X_c(i),Y_c(i),X_r,Y_r); %%relatif ke arah Timur (searah sumbu X)
    theta_error(i) = theta_r(i)-theta_c(i);    
    %Candidate Heading 
    candidate_theta_pos(1:n,i) = [theta_error(i)/100:theta_error(i)/n:theta_error(i)]';
    candidate_theta_neg(1:n,i) = [-(theta_error(i)/100):-theta_error(i)/n:-theta_error(i)]';
    delta_theta(1:2*n,i) = [candidate_theta_pos(:,i);candidate_theta_neg(:,i)];
    G_heading(:,i) = theta_error(i)+delta_theta(:,i);
    %Calculate distance 
    rho(i) = sqrt((X_r-X_c(i)).^2+(Y_r-Y_c(i)).^2);
    G_dist(:,i) = rho(i);
    %Candidate Velocity 
    v(:,i) = rho(i).*cos(delta_theta(:,i));
    candidate_v_pos(1:n,i) = [v(i)/100:v(i)/n:v(i)]';
    candidate_v_neg(1:n,i) = [-(v(i)/100):-v(i)/n:-v(i)]';
    delta_v(1:2*n,i) = [candidate_v_pos(:,i);candidate_theta_neg(:,i)];
    V_max(i) = max(v(i)+delta_v(:,i));
    acc(i) = min((V_max(i)-v(i))/dt);
    %next velocity 
    v(i+1) = v(i)+0.1*dt;
    if v(i+1)>V_max(i)
        G_vel(i) = 0 ;
        v(i+1) = v(i)+0.1*dt;
    else
        G_vel(i) = v(i);
        v(i+1) = v(i);
    end
    G(i) = G_heading(i)+G_dist(i)+G_vel(i);
    w(i+1) = w(i)-theta_r(i)*dt;
    theta_c(i+1) = theta_c(i)+w(i+1)*dt;
    X_c(i+1) = X_c(i)+v(i+1)*cos(theta_c(:,i+1))*dt;
    Y_c(i+1) = Y_c(i)+v(i+1)*sin(theta_c(:,i+1))*dt;
    ex(i) = norm(X_r-X_c);
    ey(i) = norm(Y_r-Y_c);
%     fprintf('\n|%d|%0.2f|%0.2f|[%0.2f;%0.2f;%0.2f;%0.2f]|[%0.2f;%0.2f;%0.2f;%0.2f]|\n',i,theta_r(i),theta_error(i),delta_theta(:,i),G_heading(:,i))
    fprintf('\n|%d|%0.2f|%0.2f|%0.2f|%0.2f|%0.2f|%0.2f|%0.2f|\n',i,X_c(i),Y_c(i),theta_c(i),X_r,Y_r,theta_r(i),v(i))
end
%% Plot
figure(1);clf
% plot(X_obs,Y_obs,'ob','LineWidth',1);hold on;
plot(X_r,Y_r,'*k','LineWidth',10);hold on;
plot(X_c,Y_c,'or','LineWidth',1);hold on;
% xlim([-.5 1.2])
% ylim([-.5 1.2])
xlabel('X axis')
ylabel('Y axis')
grid on
%% 
function theta_r = CalcThetaR(X_c,Y_c,X_r,Y_r)
if X_c<=0 && Y_c<=0
    theta_temp = (atan2(Y_r-Y_c,X_r-X_c))/pi*180;
    theta_r = -90-theta_temp;
else
    theta_temp = (atan2(Y_r-Y_c,X_r-X_c))/pi*180;
    theta_r = 90-theta_temp;
end
end
