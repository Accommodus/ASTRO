%test code plotting

% live_csv_plot.m
% This script reads a CSV file every 5 seconds and updates a plot
% with the next row of data.

clear;
clc; 
close all;

%Initials
filename = 'Couting.csv';   % CSV file path
refresh_interval = 0;    % seconds between updates
data = readmatrix(filename);
[numRows, numCols] = size(data);


%Extract telemetry 
x = data(:,2); %time
y = data(:,4:6); %position
z = data(:, 11:13); %control input
v = data(:, 7:9); %velocity

tol_pos = 1e-4; %tol for stopping (km)
tol_vel =  1e-4; %gentle docking tolerance (km/s)

%Initial position plot
figure('Name', 'Real Time Plot', 'NumberTitle', 'off');
subplot(2,1,1);
posPlot = plot(x(1), y(1,1:3),'linewidth',2);
xlabel('Index (# count)'); ylabel('Position (km)');
title('Real Time Position Plot');
grid on; box on
set(gca,'fontweight','bold');

subplot(2,1,2)
ctrlPlot = plot(x(1), z(1,1:3),'linewidth',2);
xlabel('Index (# count)'); ylabel('Control Input (Delta V)');
title('Real Time Control Input Plot');
grid on; box on
set(gca,'fontweight','bold');

%Update loop (real time)
[newRows, ~] = size(data);
row = 1;
k = 1; %index
while true
    if row <= numRows
        for i = 1:3
            set(posPlot(i), 'XData', x(1:row), 'YData', y(1:row,i));
            set(ctrlPlot(i), 'XData', x(1:row), 'YData', z(1:row,i));
        end
        drawnow;
        row = row + 1;
    else
        %Re-read CSV in case it’s being appended
        data = readmatrix(filename);
        [numRows, numCols] = size(data);
        x = data(:,2);
        y = data(:,4:6);
        z = data(:,11:13);
    end
    k = k+1;
    disp(norm(y(k,:)))
    if norm(y(k,:)) < tol_pos && norm(v(k,:)) < tol_vel
        break;
    end
    pause(refresh_interval);
end



