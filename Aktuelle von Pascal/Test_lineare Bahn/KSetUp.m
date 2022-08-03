clc 
clear all


robot = rigidBodyTree;
robot.DataFormat = 'row';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Roboter bauen%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 1: Base]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
jnt1.PositionLimits = [deg2rad(-185),deg2rad(185)];
jnt1.HomePosition = deg2rad(0);
phi1 = 0;
tform = [cos(phi1)   0   sin(phi1)       0;
         0           1   0               0;
         -sin(phi1)  0   cos(phi1)  0.5225;
         0           0   0               1];
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

addBody(robot,body1,'base');

%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 2: Schulter]%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = deg2rad(0);
jnt2.PositionLimits = [deg2rad(-130), deg2rad(14)];

phi2 = (pi/2)*3;
% um x drehen
tform2 =    [1   0           0               0.5;
             0   cos(-phi2)  -sin(-phi2)       0;
             0   sin(-phi2)  cos(-phi2)   0.5225;
             0   0           0                 1];

setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1');

%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 3: Ellenbogen]%%%%%%%%%%%%%%%%%%%%%%%%%%
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.PositionLimits = [deg2rad(-100), deg2rad(144)];
jnt3.HomePosition = deg2rad(0);
phi3 = 0;

tform3 =    [1   0           0               1.3;
             0   cos(-phi3)  -sin(-phi3)       0;
             0   sin(-phi3)  cos(-phi3)        0;
             0   0           0                 1];


setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2');

%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 4: Rot Unterarm]%%%%%%%%%%%%%%%%%%%%%%%%%%
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.PositionLimits = [deg2rad(-350), deg2rad(350)];
jnt4.HomePosition = 0;
phi4 = pi/2;

tform4_1 =   [cos(phi4)  0   sin(phi4)   0.712;
             0           1   0             0;
             -sin(phi4)  0   cos(phi4)     0;
             0           0   0             1];

tform4_2 =   [cos(pi/2)  -sin(pi/2)   0    0;
             sin(pi/2)   cos(pi/2)    0    0;
             0           0            1    0;
             0           0            0    1];
tform4 = tform4_1*tform4_2;

setFixedTransform(jnt4,tform4);
body4.Joint = jnt4;
addBody(robot,body4,'body3');

%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 5: Handgelenk]%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
jnt5.PositionLimits = [deg2rad(-120), deg2rad(120)];
jnt5.HomePosition = deg2rad(0);
phi5 = pi/2;
tform5_1 =    [1  0           0                0;
             0  cos(phi5)   -sin(phi5)       0;
             0  sin(phi5)   cos(phi5)    0.313;
             0  0           0                1];
tform5_2 =   [cos(pi)  0   sin(pi)     0;
             0         1   0           0;
             -sin(pi)  0   cos(pi)     0;
             0         0   0           1];

tform5_3 =  [cos(pi/2)   -sin(pi/2)   0    0;
             sin(pi/2)   cos(pi/2)    0    0;
             0           0            1    0;
             0           0            0    1];
 tform5 = tform5_1*tform5_2*tform5_3;

setFixedTransform(jnt5,tform5);
body5.Joint = jnt5;
addBody(robot,body5,'body4');

%%%%%%%%%%%%%%%%%%%%%%%%[Körper 6: Rot Handgelenk]%%%%%%%%%%%%%%%%%%%%%%%%%
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
jnt6.PositionLimits = [deg2rad(-350),deg2rad(350)];
jnt6.HomePosition = 0;

phi6 = -(pi/2)*3;
tform6_1 =   [cos(phi6)  0   sin(phi6)    0.29;
             0           1   0               0;
             -sin(phi6)  0   cos(phi6)       0;
             0           0   0               1];

tform6_2 =  [cos(-pi/2)   -sin(-pi/2)   0    0;
             sin(-pi/2)   cos(-pi/2)    0    0;
             0           0            1    0;
             0           0            0    1];
tform6 = tform6_1*tform6_2;

setFixedTransform(jnt6,tform6);
body6.Joint = jnt6;
addBody(robot,body6,'body5');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%[Körper 7: Kelle]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bodyEndEffector = rigidBody('endeffector');
R7 = RotationUmZ(deg2rad(90))*RotationUmY(deg2rad(-45))*RotationUmX(deg2rad(-90));
V7 = [0 0 0.450]; % 0.45 ist die Länge des Stabs der Kelle
tform7 = [R7    V7';
          0 0 0 1];
setFixedTransform(bodyEndEffector.Joint,tform7);
addBody(robot,bodyEndEffector,'body6');

%wichtig für Laufbahn
robot.DataFormat = 'row';

%%Roboter Anzeigen
%homeConfiguration(robot) = deg2rad([183.97 -71.08 -108.01 62.40 -58.35 -75.11]);
KUKA = show(robot);
KUKA.XLim = [-4, 4];
KUKA.YLim = [-4, 4];
KUKA.ZLim = [0,4.5];

show(robot, deg2rad([0 0 0 0 0 0]));
hold on

% %before this you have plotted a surface in axis hAx
% axis(hAx,'equal')
% %Get X, Y and Z data for plotting the axes...
% X_range = hAx.XLim(2) - hAx.XLim(1);
% X_start = hAx.XLim(1);
% X_delta = X_range/20;
% 
% Y_delta = (hAx.YLim(2) - hAx.YLim(1))/20;
% Y_start = hAx.YLim(1);
% 
% Z_delta = (hAx.ZLim(2) - hAx.ZLim(1))/20;
% Z_start = hAx.ZLim(1);
% 
% X_Line = line(hAx,[X_start+X_delta X_start+X_delta*5],[Y_start+Y_delta Y_start+Y_delta],[Z_start+Z_delta Z_start+Z_delta]); % x Line
% Y_Line = line(hAx,[X_start+X_delta X_start+X_delta],[Y_start+Y_delta Y_start+Y_delta*5],[Z_start+Z_delta Z_start+Z_delta]); % Y Line
% Z_Line = line(hAx,[X_start+X_delta X_start+X_delta],[Y_start+Y_delta Y_start+Y_delta],[Z_start+Z_delta Z_start+Z_delta*5]); %Z Line
% X_text = text(hAx,X_start+X_delta*6,Y_start+Y_delta,Z_start+Z_delta,'x');
% Y_text = text(hAx,X_start+X_delta,Y_start+Y_delta*6,Z_start+Z_delta,'y');
% Z_text = text(hAx,X_start+X_delta,Y_start+Y_delta,Z_start+Z_delta*6,'z');