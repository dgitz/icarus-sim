clear all
close all
plotgraphs=1;
clc

% Load Data
Create_SimulinkBusObjects;
load('KalmanFilterUnitTest_dataset.mat');

% Run Model
out = sim('KalmanFilterUnitTestModel');

% Analyze Results
x = truth.signals.values;
xhat = out.xhat.Data;
error_xhat = x-xhat;
mean_kf_error = abs(mean(error_xhat));
std_kf_error = std(error_xhat);