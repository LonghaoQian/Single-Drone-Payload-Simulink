close all;
% Save simulink data
% Pull ScopeData into more useable variables
timedata = X.time;
N = length(timedata );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vbx = reshape(VB.signals.values(1,:,:),N,1);
Vby = reshape(VB.signals.values(2,:,:),N,1);
Vbz = reshape(VB.signals.values(3,:,:),N,1);
VIV = [Vbx Vby Vbz];
xdata = reshape(X.signals.values(1,:,:),N,1);
ydata = reshape(X.signals.values(2,:,:),N,1);
zdata = reshape(X.signals.values(3,:,:),N,1);
xdata_norm = reshape(X_norm.signals.values(1,:,:),N,1);
ydata_norm = reshape(X_norm.signals.values(2,:,:),N,1);
zdata_norm = reshape(X_norm.signals.values(3,:,:),N,1);
e_v_data = reshape(e_v,N,1);
e_x_data = reshape(e_x,N,1);
w_error_data = reshape(W_error.signals.values,N,1);
w_hat_data = reshape(W_hat.signals.values,N,1);
w_data = reshape(W.signals.values,N,1);
rVII = [xdata ydata zdata];
phidata = Euler.signals.values(:,1);
thetadata = Euler.signals.values(:,2);
psidata = Euler.signals.values(:,3);
OmegaVI = [phidata thetadata psidata];
Ydata = Y.signals.values;
Ydata_norm = Y_norm .signals.values;
Ndata = length(xdata);% data length
rPV = reshape(Y.signals.values,3,Ndata)';
IRBdata = IRB.signals.values;
CIV = IRBdata;

omegaVIV = reshape(omega.signals.values,3,Ndata)';
VPV = reshape(y_dot.signals.values,2,Ndata)';
F = reshape(F_L.signals.values,3,Ndata)';
M = reshape(Mq_b.signals.values,3,Ndata)';
% Calculate positional coordinates for payload from angles
PLxdata = zeros(Ndata,1);
PLydata = zeros(Ndata,1);
PLzdata = zeros(Ndata,1);
F_norm = zeros(Ndata,1);
M_norm = zeros(Ndata,1);
% calculate payload position
for i = 1:length(xdata)
    YI = reshape(X.signals.values(:,:,i),3,1)+reshape(Ydata(:,:,i),3,1);
    PLxdata(i) = YI(1);
    PLydata(i) = YI(2);
    PLzdata(i) = YI(3);
    F_norm(i) = norm(F(i,:));
    M_norm(i) = norm(M(i,:));
end
% calculate payload position without UDE
PLxdata_norm = zeros(Ndata,1);
PLydata_norm = zeros(Ndata,1);
PLzdata_norm = zeros(Ndata,1);
for i = 1:length(xdata)
    YI_norm = reshape(X_norm.signals.values(:,:,i),3,1)+reshape(Ydata_norm (:,:,i),3,1);
    PLxdata_norm(i) = YI_norm(1);
    PLydata_norm(i) = YI_norm(2);
    PLzdata_norm(i) = YI_norm(3);
end


%--------------------------------------------------------------------------
% plot xy movement
%--------------------------------------------------------------------------
figure(1)
subplot(2,1,1)
plot(timedata,e_v_data,'LineWidth',2);
ylabel('Velocity Error $||e_{v,i}||$(m/s)','Interpreter','latex')
hold on
text(10,9,'ZONE A')
text(38,9,'ZONE B')
text(70,9,'ZONE C')
yyaxis right
plot(timedata,e_x_data,'--r','LineWidth',1);

plot([27 27],[0 20],'--k','LineWidth',1)
plot([60 60],[0 20],'--k','LineWidth',1)

grid on;
xlabel('Time (s)');ylabel('Position Error $||e_{x,i}||$ (m)','Interpreter','latex');title('Path Error');legend({'Velocity Error $||e_{v,i}||$','Position Error $||e_{x,i}||$'},'Interpreter','latex')
subplot(2,1,2)
plot(timedata,w_hat_data ,'-b','LineWidth',1);
hold on
ylabel('Estimated Disturbance $||\hat{W}||(N)$','Interpreter','latex')
text(10,19,'ZONE A')
text(38,19,'ZONE B')
text(72,19,'ZONE C')

axis([0 100 0 20])
yyaxis right
% plot(timedata,w_error_data ,'-k','LineWidth',1);
% ylabel('Uncertainty Estimation $||\tilde{W}||$(N)','Interpreter','latex')
plot(timedata,w_data ,'-.r','LineWidth',2);
plot([27 27],[0 20],'--k','LineWidth',1)
plot([60 60],[0 20],'--k','LineWidth',1)
ylabel('Real Uncertainty $||W||$ (N)','Interpreter','latex')
grid on;
xlabel('Time (s)');title('Estimated Disturbance')
legend({'Estimated Disturbance $||\hat{W}||$','Distrubance $||W||$'},'Interpreter','latex')
axis([0 100 0 20])

% Specify the position and the size of the rectangle
x_r = 67; y_r = 12; w_r = 6; h_r = 21;
rectangle('Position', [x_r-w_r/2, y_r-h_r/2, w_r, h_r], ...
'EdgeColor', [0, 0, 0], 'LineWidth',1.1);

% Specify the position and the size of the second box and thus add a second axis for plotting
x_a = 0.8; y_a = 0.28; w_a = 0.08; h_a = 0.12;
ax = axes('Units', 'Normalized', ...
'Position', [x_a, y_a, w_a, h_a], ...
'Box', 'on', ...
'LineWidth', 2, ...
'Color', [1, 1, 1]);
hold on;
plot(timedata,w_hat_data ,'-b','LineWidth',1);
plot(timedata,w_data ,'-.r','LineWidth',2);

axis([64 70 0 60]);

%--------------------------------------------------------------------------
% plot payload trajectory in body frame
%--------------------------------------------------------------------------
figure(2)
subplot(2,1,1)
plot(timedata,reshape(v_p,N,1),'-b','LineWidth',2);

ylabel('Payload Velocity $||v_p||$(m/s)','Interpreter','latex')
hold on
text(10,5,'ZONE A')
text(38,5,'ZONE B')
text(70,5,'ZONE C')

yyaxis right

plot(timedata,reshape(r,N,1),'-.r','LineWidth',1);
plot([27 27],[0 2],'--k','LineWidth',1)
plot([60 60],[0 2],'--k','LineWidth',1)
ylabel('Payload Position $||r||$ (m)','Interpreter','latex');
grid on
title('Payload XZ Trajectory in I Frame');legend({'Payload Velocity $||v_p||$','Payload Position $||r||$'},'Interpreter','latex')
xlabel('Time (s)')


subplot(2,1,2)
plot(timedata,reshape(omega_error,N,1),'-b','LineWidth',2);
ylabel('Angluar Rate Error $||\tilde{\omega}||(rad/s)$','Interpreter','latex')
hold on
yyaxis right

plot(timedata,reshape(attitude_error,N,1),'-.r','LineWidth',1);

ylabel('Attitude Tracking Error $||e_R||$','Interpreter','latex');
grid on
xlabel('Time (s)')
title('Quadrotor Attitude Tracking Error');legend({'Angluar Rate Error $||\tilde{\omega}||$','Attitude Tracking Error $||e_R||$'},'Interpreter','latex')

%--------------------------------------------------------------------------
% plot quad-rotor velocity in body frame
%--------------------------------------------------------------------------
figure(3)

hold on
plot(timedata, F_norm,'-b','LineWidth',1);
ylabel('Fan Lift $f$ (N)','Interpreter','latex')
yyaxis right
plot(timedata, M_norm,'-.r','LineWidth',2);
ylabel('Fan Torque $||\tau||(N \cdot m)$','Interpreter','latex')
% plot(timedata,velocity_cmdx,'-.r','LineWidth',2)
grid on;
% legend('Vehicle Velocity','Command Velocity')
xlabel('Time (s)');ylabel('Fan Torque $||\tau||(N \cdot m)$');
title('Fan Lift and Torque');
legend({'Fan Lift $f$ (N)','Fan Torque $||\tau||(N \cdot m)$'},'Interpreter','latex')

figure(4)
plot3(xdata,ydata,zdata,'-b','LineWidth',2)
hold on
plot3(PLxdata,PLydata,PLzdata,'-r','LineWidth',1)
% plot3(xdata_norm,ydata_norm,zdata_norm,'-k','LineWidth',1)
% plot3(PLxdata_norm,PLydata_norm,PLzdata_norm,'-g','LineWidth',1)
N_p = max(size(waypoint_v));

plot3(waypoint_p(1,:),waypoint_p(2,:),waypoint_p(3,:),'--k','LineWidth',2)
plot3(waypoint_p(1,:),waypoint_p(2,:),waypoint_p(3,:),'ok','LineWidth',3)
X_r = waypoint_p(:,end) + linspace(0,180,10).*waypoint_n(:,end) ;
plot3(X_r(1,:),X_r(2,:),X_r(3,:),'--k','LineWidth',2)

text(waypoint_p(1,1)+5,waypoint_p(2,1),waypoint_p(3,1)-5,'P_1');
text(waypoint_p(1,2)+5,waypoint_p(2,2),waypoint_p(3,2)-5,'P_2');
text(waypoint_p(1,3)+5,waypoint_p(2,3),waypoint_p(3,3)-5,'P_3');
text(waypoint_p(1,4)+5,waypoint_p(2,4),waypoint_p(3,4)-5,'P_4');
text(waypoint_p(1,5)+5,waypoint_p(2,5),waypoint_p(3,5)-5,'P_5');
%%%%%%%%%%%%%%%%%%% wind zone %%%%%%%%%%%%%%%%%%%%%
a = 150;
b = 30;
l1 = 50;

ver1 = mid_point1 + [n(1:2)*a/2;b/2];
ver2 = mid_point1 + [-n(1:2)*a/2;b/2];
ver3 = mid_point1 + [-n(1:2)*a/2;-b/2];
ver4 = mid_point1 + [n(1:2)*a/2;-b/2];

ver5 = mid_point2 + [n(1:2)*a/2;b/2];
ver6 = mid_point2 + [-n(1:2)*a/2;b/2];
ver7 = mid_point2 + [-n(1:2)*a/2;-b/2];
ver8 = mid_point2 + [n(1:2)*a/2;-b/2];
X1 = [ver1(1) ver2(1) ver3(1) ver4(1)];
Y1 = [ver1(2) ver2(2) ver3(2) ver4(2)];
Z1 = [ver1(3) ver2(3) ver3(3) ver4(3)];
X2 = [ver5(1) ver6(1) ver7(1) ver8(1)];
Y2 = [ver5(2) ver6(2) ver7(2) ver8(2)];
Z2 = [ver5(3) ver6(3) ver7(3) ver8(3)];
patch(X1,Y1,Z1,'blue')
alpha(.2)
patch(X2,Y2,Z2,'blue')
alpha(.3)

XX = mid_point*ones(1,20) + n_*linspace(-aa/2,aa/2,20);

W1 = sin_W*n*sin(linspace(-aa/2,aa/2,20)*lambda_x)+delta_W*ones(1,20);

quiver3(XX(1,:),XX(2,:),XX(3,:),W1(1,:),W1(2,:),W1(3,:));

axis equal
grid on
legend('Quadrotor Trajectory','Payload Trajectory','Reference Path','Waypoints')
xlabel('Inertial Position x(m)')
ylabel('Inertial Position y(m)')
zlabel('Inertial Position z(m)')
set(gca, 'YDir', 'reverse')
set(gca, 'ZDir', 'reverse')
text(xdata(1)+5,ydata(1),zdata(1)+10,'$\uparrow$ Starting Point','Interpreter','latex')
%text(xdata_norm(2500)-5,ydata_norm(2500)-120,zdata_norm(2500)+15,' Controller without the estimator $\rightarrow$','Interpreter','latex')
%text(xdata(5700)+5,ydata(5700)+20,zdata(5700),'$\leftarrow$ Controller with the estimator ','Interpreter','latex')
text(waypoint_p(1,4)+10,waypoint_p(2,4)-20,waypoint_p(3,4)-15,'ZONE B: Changing Wind Field','Interpreter','latex')
text(waypoint_p(1,2)+60,waypoint_p(2,2)+10,waypoint_p(3,2)-15,'ZONE A: Constant Wind Field','Interpreter','latex')
text(waypoint_p(1,5)-120,waypoint_p(2,5)+20,waypoint_p(3,5)+5,'ZONE C: Strong Gust','Interpreter','latex')
text(waypoint_p(1,5)-80,waypoint_p(2,5),waypoint_p(3,5)+5,'$\leftarrow$ Path Deviation by Gust','Interpreter','latex')
axis([-100 120 0 270 -20 30])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
