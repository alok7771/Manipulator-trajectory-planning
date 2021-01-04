function [q] = ik(p)
syms t1 t2 t3 t4 t5
l1 = 17;l2 = 17;l3 = 7;l4 = 4;l5 = 4;l6 = 9; t4=0; t5=0;
T01 = [cos(t1),-sin(t1),0,0;sin(t1),cos(t1),0,0;0,0,1,l1;0,0,0,1];
T12 = [-sin(t2),-cos(t2),0,0;0,0,-1,0;cos(t2),-sin(t2),0,0;0,0,0,1];
T23 = [-sin(t3),-cos(t3),0,l2;cos(t3),-sin(t3),0,0;0,0,1,0;0,0,0,1];
T34 = [-sin(t4),-cos(t4),0,l4;0,0,-1,-(l3+l5);cos(t4),-sin(t4),0,0; 0,0,0,1];
T45 = [cos(t5),-sin(t5),0,0;0,0,-1,0;sin(t5),cos(t5),0,0;0,0,0,1];
T56 = [0,-1,0,0;0,0,1,l6;-1,0,0,0;0,0,0,1];
T06= T01*T12*T23*T34*T45*T56;
eq1=simplify(T06(1,4))==p(1);
eq2=simplify(T06(2,4))==p(2);
eq3=simplify(T06(3,4))==p(3);
% eq1=simplify(T06(1,4));
% eq2=simplify(T06(2,4));
% eq3=simplify(T06(3,4));
th=solve([eq1 eq2 eq3],[t1 t2 t3]);
q=[th.t1 th.t2 th.t3 0 0];
% q=wrapTo2Pi(q);
end