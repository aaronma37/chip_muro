

%% Quadcopter Simulations
% clear
% clc
close all
% mydatae=load('mydatae.mat')
fileID = fopen('step.txt','r');
count=1;



% while 1==1
%     check=fscanf(fileID, '%f');
%     if isempty(check)
%         empty=1
%         break;
%     end
%     timeMeasurement(count)=check;
%     scanner=fscanf(fileID, ',%f');
%     uTestData(count)=scanner(2);
%     yTestData(count)=scanner(1);
% 
%     count=count+1;
% end
% uTestData=uTestData-.01;
% timeMeasurement=(timeMeasurement-timeMeasurement(1))/1e9;
% time=0:.02:130;
% uTestData=interp1(timeMeasurement,uTestData,time,'linear','extrap');
% yTestData=interp1(timeMeasurement,yTestData,time,'linear','extrap');
% time
fclose(fileID)

x(1,1)=-1;
x(1,2)=-1;
x(1,3)=-.01;
v(1,1)=0;
v(1,2)=0;
v(1,3)=0;
t(1)=0;
k=-10;
kp=-100;
B(1)=0;
B(2)=0;
F(2)=0;
F(1)=0;
kd=-5;
T=50;
uRecord(1)=0;
maxVelFactor=1.25;
for i=1:1:800;
    
    %% Control Law
    s=1.5*x(i,1)+v(i,1);
    u(1)=k*s;
    
    if (i>1)
    u(2)=kp*x(i,2)+kd*(x(i,2)-x(i-1,2))*50;
    else
    u(2)=kp*x(i,2);
    end
    if u(1)>1;
        u(1)=1;
                
    else if u(1)<-1
       u(1)=-1; 
           
        end
    end
    
    if (u(1)*maxVelFactor<v(i,1) && u(1)>0)
                u(1)=0;
    end
    if (u(1)*maxVelFactor>v(i,1) && u(1)<0)
            u(1)=0;
    end
    
    if u(2)>1;
        u(2)=1;
        if (u(2)*maxVelFactor<v(i,2))
                u(2)=0;
            end
    else if u(2)<-1
       u(2)=-1; 
       if (u(2)*maxVelFactor>v(i,2))
                u(2)=0;
            end
        end
    end
    uRecord(i+1)=u(1);
    %% Simulate
    
    
    %% QSMC
    x(i+1,1)=x(i,1)+v(i,1)/T;
    v(i+1,1)=v(i,1)+u(1)/T;
    
%     if v(i+1,1) > u(1)*maxVelFactor && v(i+1,1)-v(i,1)>0
%         v(i+1,1)=v(i,1);
%     else
%         v(i+1,1)=v(i,1)+u(1)/T;
%     end
%     if v(i+1,1) < u(1)*maxVelFactor && v(i+1,1)-v(i,1) <0 
%         v(i+1,1)=v(i,1);
%     else
%         v(i+1,1)=v(i,1)+u(1)/T;
%     end
    
    %% PID
    x(i+1,2)=x(i,2)+v(i,2)/T;
    v(i+1,2)=v(i,2)+.55*u(2)/T;
%     if v(i+1,2) > u(2)*maxVelFactor && v(i+1,2)-v(i,2) >0 
%         v(i+1,2)=v(i,2);
%     else
%         v(i+1,2)=v(i,2)+u(2)/T;
%     end
%     if v(i+1,2)<-u(2)*maxVelFactor && v(i+1,2)-v(i,2) <0 
%         v(i+1,2)=v(i,2);
%     else
%         v(i+1,2)=v(i,2)+u(2)/T;
%     end
    
    %% Model Validation

    
    
    x(i+1,3)=x(i,3)+1*v(i,3)/T;
if t(i)<4.5
    a=.05;
    if (a*maxVelFactor<v(i,3))
        a=0;
    end
else
    a=-.05;
    if (a*maxVelFactor>v(i,3))
        a=0;
    end
end
v(i+1,3)=v(i,3)+.6*a/T;
    
    
    t(i+1)=i*.02;
    end
figure(1)
for i=1:2
    plot(t,x(:,i))
    hold on;
    plot(t,v(:,i),'r')
    
end
plot(t,uRecord)
legend('SMC','SMC','PID','PID')

grid on
figure(2)
plot(x(:,1),v(:,1))

figure(3)
plot(t, mydatae.u(i))
hold on
plot(t, x(:,3));
t=1:length(mydatav.y)
plot(t/50,mydatav.y);




