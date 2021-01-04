%% script for Question 1(b) and 3(a)
%Xavier Cremades s3649512
%Alok Ranjan s3816494


clear all; clc;
syms t1 t2 t3 t4 t5 

%% lengts in centimeters
l1 = 17;l2 = 17;l3 = 7;l4 = 4;l5 = 4;l6 = 9;
%% Theta angles (in radian)
q=[0,0,0,0,0];

%% For forward kinematics matrix calculations
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Question 1 b  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
T01 = [cos(t1),-sin(t1),0,0;sin(t1),cos(t1),0,0;0,0,1,l1;0,0,0,1];
T12 = [-sin(t2),-cos(t2),0,0;0,0,-1,0;cos(t2),-sin(t2),0,0;0,0,0,1];
T23 = [-sin(t3),-cos(t3),0,l2;cos(t3),-sin(t3),0,0;0,0,1,0;0,0,0,1];
T34 = [-sin(t4),-cos(t4),0,l4;0,0,-1,-(l3+l5);cos(t4),-sin(t4),0,0; 0,0,0,1];
T45 = [cos(t5),-sin(t5),0,0;0,0,-1,0;sin(t5),cos(t5),0,0;0,0,0,1];
T56 = [0,-1,0,0;0,0,1,l6;-1,0,0,0;0,0,0,1];
T06= T01*T12*T23*T34*T45*T56;
P06=T06(:,4);
disp('Transformation matrix from frame 1 to frame 0: T01')
disp(T01)
disp('Transformation matrix from frame 2 to frame 1: T12')
disp(T12) 
disp('Transformation matrix from frame 3 to frame 2: T23')
disp(T23)
disp('Transformation matrix from frame 4 to frame 3: T34')
disp(T34)
disp('Transformation matrix from frame 5 to frame 4: T45')
disp(T45)
disp('Transformation matrix from frame 6 to frame 5: T56')
disp(T56)
disp('Transformation matrix from frame 6 to frame 0: T06')
disp(T06)
disp('Vector P06')
disp(simplify(P06))

T06n=subs(T06,[t1,t2,t3,t4,t5],[q]);
disp('Numerical value of transformation matrix from frame 6 to frame 0: T06')
disp(T06n)

%% For inverse kinematics matrics calculation
P06IK=subs(P06,[t4,t5],[0,0]);
disp('putting t4=0 and t5=0 in P06 leads to (For Inverse Kinematics)')
disp(P06IK)
 
%% Jacobians related to Linear Velocities
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Question 3 a  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
X=T06;
J1 = [simplify(diff(X(1,4),t1));simplify(diff(X(2,4),t1));simplify(diff(X(3,4),t1))];
J2 = [simplify(diff(X(1,4),t2));simplify(diff(X(2,4),t2));simplify(diff(X(3,4),t2))];
J3 = [simplify(diff(X(1,4),t3));simplify(diff(X(2,4),t3));simplify(diff(X(3,4),t3))];
J4 = [simplify(diff(X(1,4),t4));simplify(diff(X(2,4),t4));simplify(diff(X(3,4),t4))];
J5 = [simplify(diff(X(1,4),t5));simplify(diff(X(2,4),t5));simplify(diff(X(3,4),t5))];
disp('Jacobians related to Linear Velocities at joint1: J1')
disp(simplify(J1))
disp('Jacobians related to Linear Velocities at joint2: J2')
disp(J2)
disp('Jacobians related to Linear Velocities at joint3: J3')
disp(J3)
disp('Jacobians related to Linear Velocities at joint4: J4')
disp(J4)
disp('Jacobians related to Linear Velocities at joint5: J5')
disp(J5)


%% Jacobians related to Angular Velocities

T01 = T01;
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
Z01 = [T01(1,3);T01(2,3);T01(3,3)];
Z02 = [T02(1,3);T02(2,3);T02(3,3)];
Z03 = [T03(1,3);T03(2,3);T03(3,3)];
Z04 = [T04(1,3);T04(2,3);T04(3,3)];
Z05 = [T05(1,3);T05(2,3);T05(3,3)];
Z = [Z01,Z02,Z03,Z04,Z05];
w=simplify(Z);

disp('Jacobians related to Angular Velocities')
disp(w)

%% Numerical value of jacobians

Jvw=[J1,J2,J3,J4,J5;w];
J=subs(Jvw,[t1,t2,t3,t4,t5],[q]);
disp('Numerical value of Jacobian matrix substituting thetas')
disp(J)
