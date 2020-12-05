%% read input from file
clear;
clc;
Array=csvread('quasi_random_normal_distibution_test.csv');
x_data = Array(:, 1);
y_data = Array(:, 2);
fit = fitlm(x_data,y_data)
rsquared = fit.Rsquared.Ordinary;
qqplot(x_data,y_data)
annotation('textbox', [0.2, 0.75, 0.1, 0.1],'String', "R-squared=" + rsquared);