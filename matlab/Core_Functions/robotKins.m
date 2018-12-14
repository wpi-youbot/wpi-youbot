function Vw = robotKins(Vo,L1,L2,r)
    % Swap Vo x and y directions to match the world coordinates
    Vo = [Vo(2); Vo(1); Vo(3)];

    alpha = pi/4;
    ta=1/tan(alpha);
    Lta=((L1*tan(alpha) + L2)/tan(alpha));

    J = 1/r * [1  ta -Lta;
               1 -ta  Lta;
               1 -ta -Lta;
               1  ta  Lta];

    Vw = J * Vo;
    len = Vw*r;
end
