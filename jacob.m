%% jacobian
for i= 1:length(t)
qold=arb.getpos();
J0=pArb.jacob0(qold);
Jvw=pinv(J0);
v=[1 0 0 0 0 0]';% velocity matrix
qvel=(Jvw*v)';
dt=0.05;
K=10;
qnew=qold+K*dt*qvel;
arb.setpos(qnew)
end


%% archive
% for i=1:length(t)
% J0=pArb.jacob0(q(i,:));
% J(i)={J0};
% Jn=cell2mat(J(i));
% Jvw=pinv(Jn);
% v=[1 0 0 0 0 0]';% velocity matrix
% qvel=(Jvw*v)';
% qold=q(i);
% dt=0.05;
% K=10;
% qnew(i,:)=qold+K*dt*qvel;
% end
