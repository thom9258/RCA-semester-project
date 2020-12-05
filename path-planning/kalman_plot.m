%% read input from file
clear;
clc;
Array=csvread('kalman_filtering_turn_straight.csv');
%Array=csvread('kalman_filtering_straight_turn_straight.csv');
%Array=csvread('dead_reckoning_test.csv');
%Array=csvread('kalman_filtering_test.csv');
drx = Array(:, 1);
dry = Array(:, 2);
dro = Array(:, 3);
kax = Array(:, 4);
kay = Array(:, 5);
kao = Array(:, 6);
kadet = Array(:, 7);

plot(drx,dry)
title('Robot Movement')
hold on
plot(kax,kay)
legend('Dead-Reckoning', 'Kalman Filtering')
xlabel('X-Position of robot') 
ylabel('Y-Position of robot')