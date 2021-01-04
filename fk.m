function [p,R]=fk(q)

%% 4c
%Xavier Cremades s3649512
%Alok Ranjan s3816494

%%  Forward kinematics
%   The function does the caclutaion of the Forward kinematics of the robotic arm.
%   q is an input and represents the configuration of the arm, size= [5x1].
%   values in q should be in radians
%   p is an output and it is the translational vector of out T06 matrix
%   Here, 0=base frame, 6= end effector.
%   R is an output and it is the rotational matrix of T06
%%

%Lengths in centimeter.[L1,L2,L3,L4,L5,L6]
L=[17;17;7;4;4;9]; 
% We have used the T matrices from our 1b calculation as with this it will
% be faster to calculate everything rather than using a generalized
% function.
T01 = [cos(q(1)),-sin(q(1)),0,0;sin(q(1)),cos(q(1)),0,0;0,0,1,L(1);0,0,0,1];
T12 = [-sin(q(2)),-cos(q(2)),0,0;0,0,-1,0;cos(q(2)),-sin(q(2)),0,0;0,0,0,1];
T23 = [-sin(q(3)),-cos(q(3)),0,L(2);cos(q(3)),-sin(q(3)),0,0;0,0,1,0;0,0,0,1];
T34 = [-sin(q(4)),-cos(q(4)),0,L(4);0,0,-1,-(L(3)+L(5));cos(q(4)),-sin(q(4)),0,0; 0,0,0,1];
T45 = [cos(q(5)),-sin(q(5)),0,0;0,0,-1,0;sin(q(5)),cos(q(5)),0,0;0,0,0,1];
T56 = [0,-1,0,0;0,0,1,L(6);-1,0,0,0;0,0,0,1];
T06 = T01*T12*T23*T34*T45*T56;
    
 % R matrix and p vector of T06:
 R = [T06(1:3,1:3)]; % [3x3] R matrix
 p = [T06(1:3,4)]; % [3x1] p vector

end