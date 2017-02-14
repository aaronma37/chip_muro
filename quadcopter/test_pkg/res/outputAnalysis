%%time,field.linear.x,field.linear.y,field.linear.z,field.angular.x,field.angular.y,field.angular.z
clear
clc
count=1;
fclose('all');
fileID = fopen('input.txt');
figure('units','normalized','outerposition',[0 0 1 1])
timeMeasurement(1)=0;
while 1==1
    check=fscanf(fileID, '%f');
    if isempty(check)
        empty=1
        break;
    end
    timeMeasurement(count)=check;
%     fscanf(fileID, '%f');
%      fscanf(fileID, '%f,')
    scanner=fscanf(fileID, ',%f');
    x(count)=scanner(1);
    y(count)=scanner(2);
%     q1(count)=scanner(6);
%     q2(count)=scanner(7);
%     [r1(count) r2(count) r3(count)] = quat2angle([(q2(count)) 0 0 q1(count)]);
%     r1(count)=r1(count)+3.14;
    count=count+1;
end
timeMeasurement

hold on;
title('Quadcopter Dynamics')



plot(timeMeasurement,x)

