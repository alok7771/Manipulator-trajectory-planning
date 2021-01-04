clear all, 
clc,

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

%% Q4d joint space, input is in theta vectors

q1 = [0, 0, 0, 0, 0]; % starting pose
q2 = [0, -pi/2, -pi/2, 0, 0]; % end pose
t = 0:0.05:2;
[q, qd, qdd] = jtraj(q1, q2, t);
pArb.plot(q)
T_traj = pArb.fkine(q);
p_E = transl(T_traj);

% plot of joint trajectories

figure(3)

% joints positions
subplot(1, 3, 1)
plot(t, q(:, 1), 'LineWidth', 3); 
hold on;
plot(t, q(:, 2), 'LineWidth', 3);
plot(t, q(:, 3), 'LineWidth', 3);
plot(t, q(:, 4), 'LineWidth', 3);
plot(t, q(:, 5), 'LineWidth', 3);
grid on;
title('Joints Position')
xlabel('Time[s]') 
ylabel('State[rad]') 
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5');
hold off

% joints velocities
subplot(1, 3, 2)
plot(t, qd(:, 1), 'LineWidth', 3); 
hold on;
plot(t, qd(:, 2), 'LineWidth', 3);
plot(t, qd(:, 3), 'LineWidth', 3);
plot(t, qd(:, 4), 'LineWidth', 3);
plot(t, qd(:, 5), 'LineWidth', 3);
grid on;
title('Joints Velocities')
xlabel('Time[s]') 
ylabel('Velocity[rad/s]') 
legend('qd_1', 'qd_2', 'qd_3', 'qd_4', 'qd_5');
hold off

%end-effector position
subplot(1, 3, 3)
plot(t, p_E(:, 1), 'LineWidth', 3); 
hold on;
plot(t, p_E(:, 2), 'LineWidth', 3); 
plot(t, p_E(:, 3), 'LineWidth', 3); 
grid on;
title('End-Eff. Pos')
xlabel('Time[s]') 
ylabel('Position[cm]') 
legend('x_E', 'y_E', 'z_E');
hold off

%% Q4d Cartesian space, input is in theta vectors

q1 = [0, 0, 0, 0, 0];
q2 = [0, -pi/2, -pi/2, 0, 0];
T_1 = pArb.fkine(q1);  % start 
T_1
disp('*************************')
T_2 = pArb.fkine(q2);   % end
T_2
t = 0:0.05:2;

T_ctraj = ctraj(T_1, T_2, length(t));
p_E_ctraj = transl(T_ctraj);

disp(p_E_ctraj)

q_ctraj = pArb.ikine(T_ctraj, 'mask', [1 0 1 0 0 1]);


disp(q_ctraj);

figure(4)
pArb.plot(q_ctraj);

% plot the joint trajectories

figure(5)

%joints positions
subplot(1, 2, 1)
plot(t, q_ctraj(:, 1), 'LineWidth', 3); 
hold on;
plot(t, q_ctraj(:, 2), 'LineWidth', 3);
plot(t, q_ctraj(:, 3), 'LineWidth', 3);
plot(t, q_ctraj(:, 4), 'LineWidth', 3);
plot(t, q_ctraj(:, 5), 'LineWidth', 3);
grid on;
title('Joints Position')
xlabel('Time[s]') 
ylabel('State[rad]') 
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5');

%end-effector positions
subplot(1, 2, 2)
plot(t, p_E_ctraj(:, 1), 'LineWidth', 3); 
hold on;
plot(t, p_E_ctraj(:, 2), 'LineWidth', 3); 
plot(t, p_E_ctraj(:, 3), 'LineWidth', 3); 
grid on;
title('End-Eff. Pos')
xlabel('Time[s]') 
ylabel('Position[cm]') 
legend('x_E', 'y_E', 'z_E');
hold off

% plot in 2-D of the end-effector position

figure(6)
plot(p_E_ctraj(:, 1), p_E_ctraj(:, 3), 'LineWidth', 3)
hold on
scatter(0, 0, 'filled', 'MarkerEdgeColor', 'r', 'SizeData', 200,...
    'MarkerFaceColor', 'r') % plot the base of the arm
scatter(p_E_ctraj(1, 1), p_E_ctraj(1, 3), 'MarkerEdgeColor', 'b', 'SizeData', 100)
scatter(p_E_ctraj(end, 1), p_E_ctraj(end, 3), 'MarkerEdgeColor', 'g', 'SizeData', 100)
legend('E-E trajectory', 'Arm base', 'Start', 'Finish')
title('End-Effector Trajectory');
grid on;
xlabel('X [cm]') 
ylabel('Z [cm]')
axis equal;
hold off

%% Q4e Cartesian space, input is in terms of position vectors


p_start = [10,10,0];    % Insert a starting point [3x1] vector
p_stop = [15,15,15];   % Insert a finish point [3x1] vector


T_start=transl(p_start);
T_stop=transl(p_stop);


[q_1] = pArb.ikine(T_start,'mask', [1 1 1 0 0 0] ); 	
[q_2] = pArb.ikine(T_stop,'mask', [1 1 1 0 0 0] );      
% make transformation matrices
T_1 = pArb.fkine(q_1);    % Create a transformation matix of the start poses
T_2 = pArb.fkine(q_2);    % Create a transformation matix of the end poses 

t = 0:0.05:2;

T_ctraj = ctraj(T_1, T_2, length(t));
p_E_ctraj = transl(T_ctraj);

disp(p_E_ctraj)

q_ctraj = pArb.ikine(T_ctraj, 'mask', [1 1 1 0 0 0]);


disp(q_ctraj);

figure(7)
pArb.plot(q_ctraj);

% plot the joint trajectories

figure(8)

%joints positions
subplot(1, 2, 1)
plot(t, q_ctraj(:, 1), 'LineWidth', 3); 
hold on;
plot(t, q_ctraj(:, 2), 'LineWidth', 3);
plot(t, q_ctraj(:, 3), 'LineWidth', 3);
plot(t, q_ctraj(:, 4), 'LineWidth', 3);
plot(t, q_ctraj(:, 5), 'LineWidth', 3);
grid on;
title('Joints Position')
xlabel('Time[s]') 
ylabel('State[rad]') 
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5');

%end-effector positions
subplot(1, 2, 2)
plot(t, p_E_ctraj(:, 1), 'LineWidth', 3); 
hold on;
plot(t, p_E_ctraj(:, 2), 'LineWidth', 3); 
plot(t, p_E_ctraj(:, 3), 'LineWidth', 3); 
grid on;
title('End-Eff. Pos')
xlabel('Time[s]') 
ylabel('Position[cm]') 
legend('x_E', 'y_E', 'z_E');
hold off

%% plot in 2-D of the end-effector position
%XZ co-ordinate
figure(9)
plot(p_E_ctraj(:, 1), p_E_ctraj(:, 3), 'LineWidth', 3)
hold on
scatter(0, 0, 'filled', 'MarkerEdgeColor', 'r', 'SizeData', 200,...
    'MarkerFaceColor', 'r') % plot the base of the arm
scatter(p_E_ctraj(1, 1), p_E_ctraj(1, 3), 'MarkerEdgeColor', 'b', 'SizeData', 100)
scatter(p_E_ctraj(end, 1), p_E_ctraj(end, 3), 'MarkerEdgeColor', 'g', 'SizeData', 100)
legend('E-E trajectory', 'Arm base', 'Start', 'Finish')
title('End-Effector Trajectory');
grid on;
xlabel('X [cm]') 
ylabel('Z [cm]')
axis equal;
hold off

%XZ co-ordinate
figure(10)
plot(p_E_ctraj(:, 1), p_E_ctraj(:, 2), 'LineWidth', 3)
hold on
scatter(0, 0, 'filled', 'MarkerEdgeColor', 'r', 'SizeData', 200,...
    'MarkerFaceColor', 'r') % plot the base of the arm
scatter(p_E_ctraj(1, 1), p_E_ctraj(1, 3), 'MarkerEdgeColor', 'b', 'SizeData', 100)
scatter(p_E_ctraj(end, 1), p_E_ctraj(end, 3), 'MarkerEdgeColor', 'g', 'SizeData', 100)
legend('E-E trajectory', 'Arm base', 'Start', 'Finish')
title('End-Effector Trajectory');
grid on;
xlabel('X [cm]') 
ylabel('Y [cm]')
axis equal;
hold off