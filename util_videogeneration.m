%--------------------------------------------------------------------------
% attitude & position graphical plotting using hgtransform updates
%--------------------------------------------------------------------------
load rotor_payload_v01.mat;
anim_fig = figure('Units','normalized','OuterPosition',[0.1 0 0.8 1]);
ax = axes('XLim',[-5 5],'YLim',[-5 10],'ZLim',[15 25],'DataAspectRatioMode','manual');
% axis equal;
xlabel('x axis (m)');ylabel('y axis (m)');zlabel('z axis (m)');
view(3)
grid on

% quadrotor geometry variables (for plotting only)
wd = 0.04; % m, arm width 
th = 0.02; % m, arm thickness 
l = 0.4; % m, arm length
% note that arm length l is already specified

r_rotor = 0.3; % rotor radius 
r_offset = r_rotor/sqrt(2); % x, y coordinate (for quad in X-config)
r_pl = 0.1; % totally cosmetic, payload dimensions are physically irrelevant to model

% matrix of vertices for the quadcopter & payload geometry
vertices_front = [0.5*sqrt(2)*(wd), 0, 0;                           % 1
                  0.5*sqrt(2)*(l+wd/2), 0.5*sqrt(2)*(l-wd/2), 0;    % 2
                  0.5*sqrt(2)*(l-wd/2), 0.5*sqrt(2)*(l+wd/2), 0;    % 3
                  0, 0.5*sqrt(2)*(wd), 0;                           % 4
                  -0.5*sqrt(2)*(l-wd/2), 0.5*sqrt(2)*(l+wd/2), 0;   % 5
                  -0.5*sqrt(2)*(l+wd/2), 0.5*sqrt(2)*(l-wd/2), 0;   % 6
                  -0.5*sqrt(2)*(wd), 0, 0; % end of lower index     % 7
                  0.5*sqrt(2)*(wd), 0, th; % start of upper index   % 8
                  0.5*sqrt(2)*(l+wd/2), 0.5*sqrt(2)*(l-wd/2), th;   % 9
                  0.5*sqrt(2)*(l-wd/2), 0.5*sqrt(2)*(l+wd/2), th;   % 10
                  0, 0.5*sqrt(2)*(wd), th;                          % 11
                  -0.5*sqrt(2)*(l-wd/2), 0.5*sqrt(2)*(l+wd/2), th;  % 12
                  -0.5*sqrt(2)*(l+wd/2), 0.5*sqrt(2)*(l-wd/2), th;  % 13
                  -0.5*sqrt(2)*(wd), 0, th];                        % 14
                  
vertices_back = vertices_front * [-1 0 0; 0 -1 0; 0 0 1];

vertices_pl = [r_pl, r_pl, -L-r_pl;     % 1
               -r_pl, r_pl, -L-r_pl;    % 2
               -r_pl, -r_pl, -L-r_pl;   % 3
               r_pl, -r_pl, -L-r_pl;    % 4
               r_pl, r_pl, -L+r_pl;     % 5
               -r_pl, r_pl, -L+r_pl;    % 6
               -r_pl, -r_pl, -L+r_pl;   % 7
               r_pl, -r_pl, -L+r_pl];   % 8

% matrix specifying vertices to connect, and in what order, to make faces
faces_front = [1,2,9,8;
               2,3,10,9;
               3,4,11,10;
               4,5,12,11;
               5,6,13,12;
               6,7,14,13;
               1,2,3,4,;
               8,9,10,11;
               4,5,6,7;
               11,12,13,14;
               1,4,7,1;
               8,11,14,8];
               
faces_back = faces_front;

faces_pl = [1,2,3,4;
            1,2,6,5;
            1,4,8,5;
            2,3,7,6;
            3,4,8,7;
            5,6,7,8];

% coordinates of rotor circles
rotor_pos = [l-r_offset,l-r_offset,r_offset,r_offset;
             -l,-l,r_offset,r_offset
             l-r_offset,-l,r_offset,r_offset;
             -l,l-r_offset,r_offset,r_offset];
     
% use patch to create quadcopter arms
% rectangle with Curvature = [1 1] generates a circle
arm_obj(1) = rectangle('Position',rotor_pos(1,:),'Curvature',[1 1],'FaceColor','blue');
arm_obj(2) = rectangle('Position',rotor_pos(2,:),'Curvature',[1 1],'FaceColor','red');
arm_obj(3) = rectangle('Position',rotor_pos(3,:),'Curvature',[1 1],'FaceColor','red');
arm_obj(4) = rectangle('Position',rotor_pos(4,:),'Curvature',[1 1],'FaceColor','blue');
arm_obj(5) = patch('Faces',faces_front,'Vertices',vertices_front,'FaceColor','blue');
arm_obj(6) = patch('Faces',faces_back,'Vertices',vertices_back,'FaceColor','red');
% create separate object for payload and cable
box_obj(1) = patch('Faces',faces_pl,'Vertices',vertices_pl,'FaceColor','cyan');
box_obj(2) = line([0,0],[0,0],[0,-L]);
    
% Create a transform object and parent the geometry objects to it. 
% Quadrotor object
quad_obj = hgtransform('Parent',ax);
set(arm_obj,'Parent',quad_obj)
% Payload object
payload_obj = hgtransform('Parent',ax);
set(box_obj,'Parent',payload_obj);

% Create a text object to display the time during simulation
timestamp = text(0,2.5,3.5,['t = ' num2str(0.0)],'FontSize',20);

% Graphics update loop
for i = 1:length(phidata)
    % quadcopter rotation and translation matrix
    Rquad_rotate = makehgtform('xrotate',phidata(i),'yrotate',-thetadata(i),'zrotate',-psidata(i));
    Rquad_trans = makehgtform('translate',[xdata(i) -ydata(i) -zdata(i)]);   
    Rquad_matrix = Rquad_trans*Rquad_rotate;
    % payload rotation and translation matrix
    Rpl_rotate = makehgtform('xrotate',0,'yrotate',0); % Note the minus axdata is from the definition of how the payload angles add on to the x and y payload coordinates
    Rpl_trans = makehgtform('translate',[PLxdata(i) -PLydata(i) -PLzdata(i)]);
    Rpl_matrix = Rpl_trans*Rpl_rotate;
    
    % update objects with new rotation matrix values and sim time
    set(quad_obj,'Matrix',Rquad_matrix);
    set(payload_obj,'Matrix',Rpl_matrix);
    set(timestamp,'String',['t = ' num2str(X.time(i))]);
    
%     if i ~= 1
%         delete(quad_trajectory);
%         delete(payload_trajectory);
%     end
%     quad_trajectory = plot3(xdata(1:i),ydata(1:i),zdata(1:i));
%     payload_trajectory = plot3(PLxdata(1:i),PLydata(1:i),PLzdata(1:i));
    
    drawnow % visually update the window on every iteration
    % comment out the following line if you don't need to create a new
    % video
    %movie_array(i) = getframe(anim_fig);
end
% Video file creation (only if you need to create a new video file)
% Comment this section out to run the plotting code faster. The getframe
% and video writing process significantly slows the run speed.

% Video frames are stored in test_movie matrix
% Change first parameter of VideoWriter to the filename you want

v = VideoWriter('sim_10secarc_iso.mp4','MPEG-4');
v.FrameRate = 30; % control playback speed of video
open(v);
writeVideo(v, movie_array);
close(v);