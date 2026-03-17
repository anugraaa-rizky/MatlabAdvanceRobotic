function robotic_leader_follower_3d()

clc; clear; close all;

%% SIMULATION PARAMETERS
map_size = 100;
dt = 0.05;
steps = 900;

v = 8;
d = 5;
side = 4;
obs_r = 1;
safe_dist = 8;
safe_robot = 4;
k_rep_robot = 25;
k_rep_obs = 20;

%% PID PARAMETER
Kp = 2.5;
Ki = 0.01;
Kd = 0.5;
v_f = 8;      % kecepatan follower
Kp_f = 2.0;   % gain heading follower

e_prev = 0;
e_int = 0;

%% INITIAL STATES
state1 = [12;55;0];
state2 = [8;52;0];
state3 = [8;58;0];

traj1=[]; traj2=[]; traj3=[];

%% OBSTACLES
obs=[30 35;30 22;40 40;50 40;60 20;62 6];

%% WAYPOINTS (lintasan kuning)
wp=[
25 50   % start (biru kiri)

35 45
35 33
25 27
15 20
25 17
35 13
40 27
45 35
50 50
60 55
65 45
50 25
50 15
60 13
70 13


80 13   % goal (biru kanan)
];

wp_id=1;
mode=0;

%% FIGURE
figure(1); clf
set(gcf,'color','w')
%% GIF SETTING
gif_filename = 'leader_follower_banj.gif';
delay = 0.05; % waktu antar frame

view(3)
axis([0 map_size 0 map_size 0 10])
grid on
hold on
axis equal

xlabel('X');ylabel('Y');zlabel('Z')
title('3D Leader Follower Navigation')

%% GROUND
[gx,gy]=meshgrid(0:10:map_size);
gz=zeros(size(gx));
mesh(gx,gy,gz,'facealpha',0.1,'edgecolor',[0.8 0.8 0.8]);

%% MAIN LOOP
for k=1:steps

tx=wp(wp_id,1); ty=wp(wp_id,2);

if norm([tx-state1(1) ty-state1(2)])<3 && wp_id<size(wp,1)
    wp_id=wp_id+1;
end

dir_wp=[tx-state1(1); ty-state1(2)];
dir_wp=dir_wp/norm(dir_wp);

vx=dir_wp(1)*15;
vy=dir_wp(2)*15;

%% OBSTACLE AVOIDANCE
near_obs=false;
for i=1:size(obs,1)
    dx=state1(1)-obs(i,1);
    dy=state1(2)-obs(i,2);
    dist=sqrt(dx^2+dy^2);
    dist = max(dist, 0.5);

    if dist<safe_dist
        rep=(1/dist-1/safe_dist);
        vx=vx+rep*dx*15;
        vy=vy+rep*dy*15;
        near_obs=true;
    end
end


%% ===== PID CONTROL =====
theta_target = atan2(vy,vx);
e = angdiff(state1(3), theta_target);

e_int = e_int + e*dt;
e_der = (e - e_prev)/dt;

omega = Kp*e + Ki*e_int + Kd*e_der;

state1(3) = state1(3) + omega*dt;
e_prev = e;

state1(1)=state1(1)+v*cos(state1(3))*dt;
state1(2)=state1(2)+v*sin(state1(3))*dt;

%% FORMATION
if ~near_obs
    target2=[state1(1)-d*cos(state1(3))+side*sin(state1(3));
             state1(2)-d*sin(state1(3))-side*cos(state1(3))];
    target3=[state1(1)-d*cos(state1(3))-side*sin(state1(3));
             state1(2)-d*sin(state1(3))+side*cos(state1(3))];
else
    target2=[state1(1)-d*cos(state1(3));
             state1(2)-d*sin(state1(3))];
    target3=[state1(1)-2*d*cos(state1(3));
             state1(2)-2*d*sin(state1(3))];
end

%% ===== FOLLOWER KINEMATIC FIX =====

%% ===== FOLLOWER WITH COLLISION AVOIDANCE =====

% ======================
% FOLLOWER 2
% ======================
F_att2 = target2 - state2(1:2); % tarik ke target
F_rep2 = [0;0];

% --- hindari obstacle ---
for i=1:size(obs,1)
    dvec = state2(1:2) - obs(i,:)';
    dist = norm(dvec);

    if dist < safe_dist
        F_rep2 = F_rep2 + k_rep_obs * (1/dist - 1/safe_dist) * (dvec/(dist^3));
    end
end

% --- hindari robot lain ---
d12 = state2(1:2) - state1(1:2);
dist12 = norm(d12);
dist12 = max(dist12, 0.3);  % 🔥 anti divide by zero

if dist12 < safe_robot
    F_rep2 = F_rep2 + k_rep_robot * (d12/(dist12^3));
end

d23 = state2(1:2) - state3(1:2);
dist23 = norm(d23);
dist23 = max(dist23, 0.3);  % 🔥 anti divide by zero

if dist23 < safe_robot
    F_rep2 = F_rep2 + k_rep_robot * (d23/(dist23^3));
end

F_total2 = F_att2 + F_rep2;

theta_target2 = atan2(F_total2(2), F_total2(1));
e2 = angdiff(state2(3), theta_target2);

omega2 = Kp_f * e2;
omega2 = max(min(omega2,2),-2);

state2(3) = state2(3) + omega2 * dt;
state2(1) = state2(1) + v_f*cos(state2(3))*dt;
state2(2) = state2(2) + v_f*sin(state2(3))*dt;


% ======================
% FOLLOWER 3
% ======================
F_att3 = target3 - state3(1:2);

F_rep3 = [0;0];

% obstacle
for i=1:size(obs,1)
    dvec = state3(1:2) - obs(i,:)';
    dist = norm(dvec);

    if dist < safe_dist
        F_rep3 = F_rep3 + k_rep_obs * (1/dist - 1/safe_dist) * (dvec/(dist^3));
    end
end

% robot lain
d31 = state3(1:2) - state1(1:2);
dist31 = norm(d31);
dist31 = max(dist31, 0.3);  % 🔥 anti divide by zero

if dist31 < safe_robot
    F_rep3 = F_rep3 + k_rep_robot * (d31/(norm(d31)^3));
end

d32 = state3(1:2) - state2(1:2);
dist32 = norm(d32);
dist32 = max(dist32, 0.3);  % 🔥 anti divide by zero

if dist32 < safe_robot
    F_rep3 = F_rep3 + k_rep_robot * (d32/(norm(d32)^3));
end

F_total3 = F_att3 + F_rep3;

theta_target3 = atan2(F_total3(2), F_total3(1));
e3 = angdiff(state3(3), theta_target3);

omega3 = Kp_f * e3;
omega3 = max(min(omega3,2),-2);

state3(3) = state3(3) + omega3 * dt;
state3(1) = state3(1) + v_f*cos(state3(3))*dt;
state3(2) = state3(2) + v_f*sin(state3(3))*dt;

traj1=[traj1;state1(1:2)'];
traj2=[traj2;state2(1:2)'];
traj3=[traj3;state3(1:2)'];

%% RENDER
cla
drawEnvironment(obs,obs_r,wp,wp_id)

plot3(traj1(:,1),traj1(:,2),zeros(size(traj1,1),1),'b--','linewidth',1.5)
plot3(traj2(:,1),traj2(:,2),zeros(size(traj2,1),1),'r--','linewidth',1.5)
plot3(traj3(:,1),traj3(:,2),zeros(size(traj3,1),1),'g--','linewidth',1.5)

drawRobot3D(state1(1),state1(2),state1(3),'b')
drawRobot3D(state2(1),state2(2),state2(3),'r')
drawRobot3D(state3(1),state3(2),state3(3),'g')

drawnow
%% ===== SAVE TO GIF =====
frame = getframe(gcf);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,256);

if k == 1
    imwrite(imind,cm,gif_filename,'gif','Loopcount',inf,'DelayTime',delay);
else
    imwrite(imind,cm,gif_filename,'gif','WriteMode','append','DelayTime',delay);
end
end
end

%% =========================
%% ROBOT 3D
%% =========================
function drawRobot3D(x,y,th,color)

bodyLen = 2;
bodyWidth = 1.2;
bodyHeight = 0.5;

wheelRad = 0.4;
wheelThick = 0.2;

R = [cos(th) -sin(th) 0;
     sin(th)  cos(th) 0;
     0 0 1];

X = [-bodyLen/2 bodyLen/2 bodyLen/2 -bodyLen/2;
     -bodyLen/2 bodyLen/2 bodyLen/2 -bodyLen/2];

Y = [-bodyWidth/2 -bodyWidth/2 bodyWidth/2 bodyWidth/2;
     -bodyWidth/2 -bodyWidth/2 bodyWidth/2 bodyWidth/2];

Z = [0 0 0 0;
     bodyHeight bodyHeight bodyHeight bodyHeight];

for i=1:numel(X)
    p = R*[X(i); Y(i); Z(i)];
    X(i)=p(1)+x;
    Y(i)=p(2)+y;
    Z(i)=p(3);
end

surf(X,Y,Z,'FaceColor',color,'EdgeColor','none');

drawWheel3D(x,y,th, bodyWidth/2 + wheelThick/2, wheelRad, wheelThick)
drawWheel3D(x,y,th,-bodyWidth/2 - wheelThick/2, wheelRad, wheelThick)

end

%% =========================
%% WHEEL
%% =========================
function drawWheel3D(x,y,th,offsetY,r,h)

[z_cyl,x_cyl,y_cyl] = cylinder(r,20);
y_cyl = y_cyl*h - h/2 + offsetY;
z_cyl = z_cyl + r - 0.1;

R = [cos(th) -sin(th) 0;
     sin(th)  cos(th) 0;
     0 0 1];

for i=1:numel(x_cyl)
    p = R*[x_cyl(i); y_cyl(i); z_cyl(i)];
    x_cyl(i)=p(1)+x;
    y_cyl(i)=p(2)+y;
    z_cyl(i)=p(3);
end

surf(x_cyl,y_cyl,z_cyl,'FaceColor','k','EdgeColor','none');

end

%% DRAW ENVIRONMENT
function drawEnvironment(obs,r,wp,current_wp)

[ocx,ocy,ocz]=cylinder(r,20);
ocz=ocz*5;

for i=1:size(obs,1)
surf(ocx+obs(i,1),ocy+obs(i,2),ocz,'facecolor',[0.4 0.4 0.4],'edgecolor','none')
end

plot3(wp(:,1),wp(:,2),zeros(size(wp,1),1),'kx','markersize',10,'linewidth',2)

if current_wp<=size(wp,1)
plot3(wp(current_wp,1),wp(current_wp,2),0,'mo','markersize',12,'linewidth',2)
end

end


%% ANGLE DIFFERENCE
function d=angdiff(a,b)
d=atan2(sin(b-a),cos(b-a));
end