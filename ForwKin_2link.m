function [pos1,pos2]=ForwKin_2link(L1,L2,q)

q1 = q(1);
q2 = q(2);


x1=L1*cos(q1);
y1=L1*sin(q1);

x2=x1+L2*cos(q1+q2);
y2=y1+L2*sin(q1+q2);


pos1=[x1,y1];
pos2=[x2,y2];

