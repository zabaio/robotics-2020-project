sympref('FloatingPointOutput',true);
syms q1(t) q2(t) q3(t);

Lbx = 0.057188;
Lby = 0.0059831;
Lbz = 0.13343;

L1=0.18967;
L2=0.21050;
L3=0.1588;

A0b = [0 1 0 Lbx; -1 0 0 Lby; 0 0 1 Lbz; 0 0 0 1];
A10 = [cos(q1(t)) -sin(q1(t)) 0 0; sin(q1(t)) cos(q1(t)) 0 0; 0 0 1 L1; 0 0 0 1]*[1 0 0 0; 0 0 1 0; 0 -1 0 0; 0 0 0 1];
A21 = [sin(q2(t)) cos(q2(t)) 0 0; -cos(q2(t)) sin(q2(t)) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 L2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
A32 = [cos(q3(t)) -sin(q3(t)) 0 0; sin(q3(t)) cos(q3(t)) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 L3; 0 1 0 0; 0 0 1 0; 0 0 0 1];

A1b = A0b*A10;
A2b = A1b*A21;
A3b = A2b*A32;

Pos1b = A1b(1:3,4);
Pos2b = A2b(1:3,4);
Pos3b = A3b(1:3,4);

Vel3b = diff(Pos3b,t);

%{
For debugging, use this on command windows.

q1 = 0;
q2 = 0;
q3 = 0;

Pos1b = [0.0572 
         0.0060 
         0.3231];

Pos2b = [0.2105*sin(q1)*sin(q2) + 0.0572
        -0.2105*cos(q1)*sin(q2) + 0.0060 
         0.2105*cos(q2)         + 0.3231];

Pos3b = [0.2105*sin(q1)*sin(q2)         + 0.1588*cos(q2)*sin(q1)*sin(q3) + 0.1588*cos(q3)*sin(q1)*sin(q2) + 0.0572
        -0.1588*cos(q1)*cos(q2)*sin(q3) - 0.1588*cos(q1)*cos(q3)*sin(q2) - 0.2105*cos(q1)*sin(q2)         + 0.0060 
         0.2105*cos(q2)                 + 0.1588*cos(q2)*cos(q3)         - 0.1588*sin(q2)*sin(q3)         + 0.3231];
%} 