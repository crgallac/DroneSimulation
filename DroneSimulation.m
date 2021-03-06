% Simulation Options

dt=2e-3;
t_haplet=dt;
scale=1; 
offset=0;


wallThickness=0; 
% Device parameters
r_EE=.2;
l=scale*5;
L=scale*6.5;
d=scale*1.78;
d1=scale*3.0; 
angle_d1=0;
orgn_d1=[0;0; 8.6;];
R1=[0 -1 0; 0 0 1; -1 0 0;];
Rphi=[cos(angle_d1) -sin(angle_d1) 0;sin(angle_d1) cos(angle_d1) 0; 0 0 1];


% Cdvw=[-1 0 0; 0 0 1; 0 1 0;]; 
Cuav_d=[1 0 0; 0 1 0; 0 0 1;]; 
Cvw_uav=[-1 0 0; 0 0 1; 0 1 0;]; %[0 0 -1; 0 1 0; 1 0 0;]*
originOffset= [-.5 6 9.6]; %in centimeters
%Demo Params
% redBallRadius= 2.5; 
radiusAvatar=.2;
navSphere=1; 
Obstacles=[0 5 -10 0 500 -11 2 -6 500 -11 4];
