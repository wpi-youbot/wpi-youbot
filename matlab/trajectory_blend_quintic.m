function [a] = trajectory_blend_quintic(x,x_d,x_dd,tf,afigure)
%This function blends quintic continuous polynomial between given path's
%via points.
%The inputs are via points vector, blend velocities vector, blend
%accelerations vector, time span, and logial operator to plot or not.
%It returns a matrix that includes the cofficients of each segment
%polynomial in a row order.

j=size(x,2);
a=zeros((j-1),6);
t=linspace(0,tf,j);
step=0.01;
no_points=((t(2)-t(1))/0.01)+1;
t_interval=zeros((j-1),no_points);
for ii=1:(j-1)
    tr=t(ii+1)-t(ii) %time of the segment referenced to the initial time of the segment
    %M is the polynomial matrix for position, velocity, acceleration for
    %initial and final conditions of the segment
    M = [1   0   0     0      0        0;
         0   1   0     0      0        0;
         0   0   2     0      0        0;
         1   tr tr^2  tr^3   tr^4     tr^5;
         0   1  2*tr 3*tr^2 4*tr^3   5*tr^4;
         0   0   2    6*tr  12*tr^2  20*tr^3];
   %b is the initial and final conditions of the segment vector
    b = [x(ii);x_d(ii);x_dd(ii);x(ii+1);x_d(ii+1);x_dd(ii+1)];
   %a is the polynomial coffecients matrix
    a(ii,:)=(M\b)';
  
    t_interval(ii,:)=t(ii):0.01:t(ii+1);
    td=t_interval(ii,:)-t(ii);
    
    if afigure==1
        figure(1);
        hold on
        plot(t_interval(ii,:),a(ii,1)+a(ii,2)*td+a(ii,3)*td.^2+a(ii,4)*td.^3+a(ii,5)*td.^4+a(ii,6)*td.^5,'k','LineWidth',1.5);
        title('Position')
        grid on

        figure(2)
        hold on

        plot(t_interval(ii,:),a(ii,2)+ 2*a(ii,3)*td+3*a(ii,4)*td.^2+4*a(ii,5)*td.^3+5*a(ii,6)*td.^4,'k','LineWidth',1.5);
        title('Velocity')
        grid on

        figure(3);
        hold on
        plot(t_interval(ii,:), 2*a(ii,3) +6*a(ii,4)*td+12*a(ii,5)*td.^2+20*a(ii,6)*td.^3,'k','LineWidth',1.5);
        title('Acceleration')
        grid on
    end
    
end

end

