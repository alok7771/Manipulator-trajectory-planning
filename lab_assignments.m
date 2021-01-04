clear all
close all
clc
%% start and stop positions in workspace
p_start = [10,10,0];    % Insert a starting point [3x1] vector
p_stop = [15,15,15];   % Insert a finish point [3x1] vector

%% ARM Lengths
%Lenghts are defined (in cm)as :[L1,L2,L3,L4,L5,L6]
Lc=[17,17,7,4,4,9];
%%
%Values for theta in figure 2 are defined in a column vector:
% theta = [0;-(pi/2);-(pi/2);0;0]; %Since the DH parameters have
%been taken from figure 2, this values will appear as offsets.

theta= [0;-(pi/2);-(pi/2);0;0];
%% Denavit-Hartenberg parameters
% Column vector for Denavit-Hartenberg parameters
DHtheta =[0;0;0;0;0;0];
DHalpha=[0;(pi/2);0;(pi/2);(pi/2);(-pi/2)];
DHa=[0;0;Lc(2);Lc(4);0;0];
DHd=[Lc(1);0;0;Lc(3)+Lc(5);0;Lc(6)];
% Configuration of figure 2 offset:
offset = [0;(pi/2);(pi/2);(pi/2);0];  %offset(3) and (4) are pi/2 because in figure 2 theta2=-pi/2 and theta3=-pi/2.
%offset 4 is pi/2 because it's the value found by hand calculations.

Rev=0;%The joint type is defined by sigma which can have 0 or 1 as value. The following variables will be used:
Pri=1;

%% Link Definition using DH parameters
%Defining the links:
L(1) = Link([DHtheta(1), DHd(1), DHa(1), DHalpha(1), Rev, offset(1)], 'modified');
L(2) = Link([DHtheta(2), DHd(2), DHa(2), DHalpha(2), Rev, offset(2)], 'modified');
L(3) = Link([DHtheta(3), DHd(3), DHa(3), DHalpha(3), Rev, offset(3)], 'modified');
L(4) = Link([DHtheta(4), DHd(4), DHa(4), DHalpha(4), Rev, offset(4)], 'modified');
L(5) = Link([DHtheta(5), DHd(5), DHa(5), DHalpha(5), Rev, offset(5)], 'modified');

%% Transformation matrices
T01 = round([cos(DHtheta(1)),-sin(DHtheta(1)),0,0;     sin(DHtheta(1)),cos(DHtheta(1)),0,0;   0,0,1,Lc(1);   0,0,0,1]);
T12 = round([-sin(DHtheta(2)),-cos(DHtheta(2)),0,0;     0,0,-1,0;   cos(DHtheta(2)),-sin(DHtheta(2)),0,0;   0,0,0,1]);
T23 = round([-sin(DHtheta(3)),-cos(DHtheta(3)),0,Lc(2);    cos(DHtheta(3)),-sin(DHtheta(3)),0,0; 0,0,1,0;   0,0,0,1]);
T34 = round([-sin(DHtheta(4)),-cos(DHtheta(4)),0,4; 0,0,-1,-(Lc(3)+Lc(5));   cos(DHtheta(4)),-sin(DHtheta(4)),0,0;   0,0,0,1]);
T45 = round([cos(DHtheta(5)),-sin(DHtheta(5)),0,0;  0,0,-1,0;   sin(DHtheta(5)),cos(DHtheta(5)),0,0;   0,0,0,1]);
T56 = round([0,-1,0,0;   0,0,1,Lc(6);   -1,0,0,0;   0,0,0,1]);

%To get T06:
T02 = round(T01*T12);
T03 = round(T02*T23);
T24 = round(T23*T34);
T46 = round(T45*T56);
T04 = round(T02*T24);
T05 = round(T04*T45);
T06 = round(T04*T46)
%% plotting the arm
%Creation of the arm.
pArb=SerialLink(L, 'name', 'Robotic arm')
%Plot of the arm
pArb.plotopt={'workspace',[-55 55 -55 55 -55 55]};
pArb.qlim(2,:)=[0, 10];
pArb.tool = T56;

figure(1)
pArb.plot([offset']) %Zero configuration
figure(2)
pArb.teach
%% 2.2(a)Working further on lab assignment

T_start=transl(p_start);
T_stop=transl(p_stop);

[q_1] = pArb.ikine(T_start,'mask', [1 1 1 0 0 0] ); 	
[q_2] = pArb.ikine(T_stop,'mask', [1 1 1 0 0 0] );    
% [q_1] = ik(T_start); 	
% [q_2] = ik(T_stop);      

% make transformation matrices
T_1 = pArb.fkine(q_1);    % Create a transformation matix of the start poses
T_2 = pArb.fkine(q_2);    % Create a transformation matix of the end poses 
t = 0:0.05:2;
T_ctraj = ctraj(T_1, T_2, length(t));
p_E_ctraj = transl(T_ctraj);
disp('trajectory positions by ctraj')
disp(p_E_ctraj)
q_ctraj = pArb.ikine(T_ctraj, 'mask', [1 1 1 0 0 0]);
disp('trajectory positions by ctraj')
disp(q_ctraj);

%% 2.2(b) smooth trajectory using joint space

% initial and final values of thetas
qin=q_ctraj(1,:); % initial joint angles
qfin=q_ctraj(length(t),:); % final joint angles
[q, qd, qdd] = jtraj(qin, qfin, t);
pArb.plot(q)
%% 2.2(c) plotting trajectory of end effector in Joint and Cartesian space
figure(2)
T_traj = pArb.fkine(q);
p_E = transl(T_traj);
figure(3)
pArb.plot(q_ctraj);
