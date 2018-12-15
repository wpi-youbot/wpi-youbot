function [a] = trajectory_blend_quintic(x,tf,afigure)
%This function blends quintic continuous polynomial between given path's
%via points.
%The inputs are via points vector, blend velocities vector, blend
%accelerations vector, time span, and logial operator to plot or not.
%It returns a matrix that includes the cofficients of each segment
%polynomial in a row order.

%% Initialization
j=size(x,2);
a=zeros((j-1),6);           % Initiating the cofficients matrix.
t=linspace(0,tf,j);         %Segmenting the time span into equal spans along each polynomial
step=0.003                  %Time step used for generating points along the polynomial
x_d=zeros(1,j)              %Initiating the Velocities vector.
x_dd=zeros(1,j)             %Initiating the accelerations vector.

%% Velocities Constraints Automatic Generating
for ii=1:(j-2)
    dk1=(x(ii+1)-x(ii))/(t(ii+1)-t(ii))
    dk2=(x(ii+2)-x(ii+1))/(t(ii+2)-t(ii+1))
    if sign(dk1)~=sign(dk2)
        x_d(ii+1)=0;
    else
        x_d(ii+1)=(dk1+dk2/2)
    end
end

%% Accelerations Constraints Automatic Generating
for ii=1:(j-2)
    dk1=(x_d(ii+1)-x_d(ii))/(t(ii+1)-t(ii))
    dk2=(x_d(ii+2)-x_d(ii+1))/(t(ii+2)-t(ii+1))
    if sign(dk1)~=sign(dk2)
        x_dd(ii+1)=0;
    else
        x_dd(ii+1)=(dk1+dk2/2)
    end
end

%% Calculating the polynomial cofficients at each pair of via points by iterations
for ii=1:(j-1)
    tr=t(ii+1)-t(ii); %time of the segment referenced to the initial time of the segment
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
    a(ii,:)=(M\b)';       %a=inv(M)*b
   
    t_interval=t(ii):step:t(ii+1);
    td=t_interval-t(ii);
%% Plotting the trajectories   
    if afigure==1
        figure(2);
        hold on
        plot(t_interval,a(ii,1)+a(ii,2)*td+a(ii,3)*td.^2+a(ii,4)*td.^3+a(ii,5)*td.^4+a(ii,6)*td.^5,'k','LineWidth',2.5);
        

        figure(3)
        hold on
        plot(t_interval,a(ii,2)+ 2*a(ii,3)*td+3*a(ii,4)*td.^2+4*a(ii,5)*td.^3+5*a(ii,6)*td.^4,'k','LineWidth',2.5);
        

        figure(4);
        hold on
        plot(t_interval, 2*a(ii,3) +6*a(ii,4)*td+12*a(ii,5)*td.^2+20*a(ii,6)*td.^3,'k','LineWidth',2.5);
        
    end
    if afigure==2
        figure(2);
        hold on
        plot(t_interval,a(ii,1)+a(ii,2)*td+a(ii,3)*td.^2+a(ii,4)*td.^3+a(ii,5)*td.^4+a(ii,6)*td.^5,'b','LineWidth',2.5);
        title('Position Trajectory')
        xlabel('Time','FontSize',12,'FontWeight','bold','Color','r')
        ylabel('Position (X,Y) Value in m','FontSize',12,'FontWeight','bold','Color','r')
       
        grid on

        figure(3)
        hold on
        plot(t_interval,a(ii,2)+ 2*a(ii,3)*td+3*a(ii,4)*td.^2+4*a(ii,5)*td.^3+5*a(ii,6)*td.^4,'b','LineWidth',2.5);
        title('Velocity Trajectory')
        xlabel('Time','FontSize',12,'FontWeight','bold','Color','r')
        ylabel('Velocity (X,Y) Value in m/s','FontSize',12,'FontWeight','bold','Color','r')
     
        grid on

        figure(4);
        hold on
        plot(t_interval, 2*a(ii,3) +6*a(ii,4)*td+12*a(ii,5)*td.^2+20*a(ii,6)*td.^3,'b','LineWidth',2.5);
        title('Acceleration Trajectory')
        xlabel('Time','FontSize',12,'FontWeight','bold','Color','r')
        ylabel('Acceleration (X,Y) Value in m/s2','FontSize',12,'FontWeight','bold','Color','r')
     
        grid on
    end
    
end

end

