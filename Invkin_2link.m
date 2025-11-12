function theta = Invkin_2link(L1,L2,coor,elbow)
if nargin < 4
    elbow = true; %true: elbow up, false: elbow down
end

% Desired end effector position
x=coor(1);
y=coor(2);

r = sqrt(x^2+y^2);

if (L1+L2<r)
    theta(1)=atan2(y,x);
    theta(2)=0;
    fprintf('%f,%f 超出workspace 太远\n',x,y);
    return;
elseif(abs(L1-L2)>r)
    theta(1)=atan2(y,x);
    theta(2)=pi;
    fprintf('%f,%f 超出workspace 太近\n',x,y);
    return;
end

q2=abs(acos((r^2-L1^2-L2^2)/(2*L1*L2)));

if elbow
    o=atan2(L2*sin(q2),L1+L2*cos(q2));
    q1=atan2(y,x)-o;
else
    q2=-q2;
    o=atan2(L2*sin(abs(q2)),L1+L2*cos(q2));
    q1=atan2(y,x)+o;
end

theta(1)=q1;
theta(2)=q2;
end