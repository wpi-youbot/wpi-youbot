    L1 = 150;
    L2 = 471/2;
    alpha = pi/4;
    r = 145/2;


%Vo = [0; 1; 0]
Vo = [5; 1; 0]
Vo = [1; 5; 0]
%Vo = [5; -1; 0]
J = 1/r * [1  1/tan(alpha) -((L1*tan(alpha) + L2)/tan(alpha));
           1 -1/tan(alpha) ((L1*tan(alpha) + L2)/tan(alpha));
           1 -1/tan(alpha) -((L1*tan(alpha) + L2)/tan(alpha));
           1  1/tan(alpha) ((L1*tan(alpha) + L2)/tan(alpha))]


Vw = J * Vo
%len = Vw * r
