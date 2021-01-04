%% 4a and 4b
%Xavier Cremades s3649512
%Alok Ranjan s3816494

close all
clear
clc

%% ARM Lengths
%Lenghts are defined (in cm)as :[L1,L2,L3,L4,L5,L6]
Lc=[17,17,7,4,4,9];
%%

%Values for theta in figure 2 are defined in a column vector:
% theta = [0;-(pi/2);-(pi/2);0;0]; %Since the DH parameters have
%been taken from figure 2, this values will appear as offsets.

theta= [0;-(pi/2);-(pi/2);0;0];

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

%Link Definition
%Defining the links:
L(1) = Link([DHtheta(1), DHd(1), DHa(1), DHalpha(1), Rev, offset(1)], 'modified');
L(2) = Link([DHtheta(2), DHd(2), DHa(2), DHalpha(2), Rev, offset(2)], 'modified');
L(3) = Link([DHtheta(3), DHd(3), DHa(3), DHalpha(3), Rev, offset(3)], 'modified');
L(4) = Link([DHtheta(4), DHd(4), DHa(4), DHalpha(4), Rev, offset(4)], 'modified');
L(5) = Link([DHtheta(5), DHd(5), DHa(5), DHalpha(5), Rev, offset(5)], 'modified');


%Transformation matrices
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

%Creation of the arm.
pArb=SerialLink(L, 'name', 'Robotic arm')
%Plot of the arm
pArb.plotopt={'workspace',[-55 55 -55 55 -55 55]};
pArb.qlim(2,:)=[0, 10];
pArb.tool = T56;

figure(1)
pArb.plot([offset']) %Zero configuration
figure(2)
qr=[pi/2 , -pi/2 , -pi/2, 0, 0];% random configuration
pArb.plot(qr) 
figure(3)
pArb.teach
T = pArb.fkine(qr)
P = transl(T);
J0 = pArb.jacob0([0 , 0 , 0, 0, 0]);
%% From 1(b)
% P06 from our implementation of qr
%  [0   21  -3]'
%% From 3(a)
% Jacobian in zero frame from our calculations
% [  0, -37, -20, 0,  0]
% [ -4,   0,   0, 0,  9]
% [  0,  -4,  -4, 0,  0]
% [  0,   0,   0, 0, -1]
% [  0,  -1,  -1, 0,  0]
% [  1,   0,   0, 1,  0]
disp('The value of 0P6 obtained from pArb in random configuration: ')
disp(P)
disp('The value of Jacobian in zero frame for pArb is: ')
disp(J0)
