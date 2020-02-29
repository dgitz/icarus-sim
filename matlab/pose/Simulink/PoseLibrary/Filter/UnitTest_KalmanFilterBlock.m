clear all
close all
plotgraphs=1;
clc

% Load Data
Create_SimulinkBusObjects;
load('KalmanFilterUnitTest_dataset.mat');
R(2,2) = 0.64;
%z.signals.values(200:300,1) = z.signals.values(200:300,1)*.95;
%C.signals.values(200:300,1) = 1;
%z.signals.values(200:300,2) = -100;

% Run Model
out = sim('KalmanFilterUnitTestModel');

% Analyze Results
x = truth.signals.values;
xhat = out.xhat.Data;
error_xhat = x-xhat;
mean_kf_error = abs(mean(error_xhat));
std_kf_error = std(error_xhat);
disp(['Kalman Error Mean: ' num2str(mean_kf_error) ' STD: ' num2str(abs(std_kf_error))]);

if(plotgraphs)
    figure(1)
    CM = jet(z.signals.dimensions);
    hold on
    title('System and Measurement')
    plot(x,'k')
    for i = 1:z.signals.dimensions
        plot(z.signals.values(:,i),'color',CM(i,:));
    end
    hold off

    figure(2)
    hold on
    title('KF')
    plot(x,'b');
    plot(xhat,'r')
    legend('x','xhat')
    
    figure(3)
    hold on
    title('P')
    plot(out.P.Data)

    figure(4)
    hold on
    title('Error: XHat')
    plot(error_xhat)
    
    figure(5)
    hist(abs(error_xhat),20,'b');
    legend('Filter-Error');
end