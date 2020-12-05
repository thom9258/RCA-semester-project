%%
clear;
clc;
Array=csvread('deterministic_paths.csv');
x = [1:26];
p1 = Array(1, :);
p1 = p1 +1;
p2 = Array(2, :);
p2 = p2 +2;
p3 = Array(3, :);
p3 = p3 +3;
p4 = Array(4, :);
p4 = p4 +4;
p5 = Array(5, :);
p5 = p5 +5;
p6 = Array(6, :);
p6 = p6 +6;
p7 = Array(7, :);
p7 = p7 +7;
p8 = Array(8, :);
p8 = p8 +8;
p9 = Array(9, :);
p9 = p9 +9;
p10 = Array(10, :);
p10 = p10 +10;

k=1;
plot(x,p1)
hold on
k=2;
plot(x,p2)
hold on
k=3;
plot(x,p3)
hold on
k=4;
plot(x,p4)
hold on
k=5;
plot(x,p5)
hold on
k=6;
plot(x,p6)
hold on
k=7;
plot(x,p7)
hold on
k=8;
plot(x,p8)
hold on
k=9;
plot(x,p9)
hold on
k=10;
plot(x,p10)

set(gca, 'YTick', [])

title('Deterministic paths')
legend('test 1', 'test 2','test 3', 'test 4','test 5', 'test 6','test 7', 'test 8','test 9', 'test 10')

ylabel('tests')
xlabel('movement steps') 
%%
clear;
clc;
Array=csvread('best_epsilon_value_static_marble.csv');
epsilon = Array(:, 1);
it = Array(:, 2);

plot(epsilon,it)
title('Determine best epsilon')
%legend('Dead-Reckoning', 'Kalman Filtering')
ylabel('Iterations') 
xlabel('epsilon')
%%
clear;
clc;
Array=csvread('epsilon_alpha_iterations_random_marbles.csv');
epsilon = Array(:, 1);
alpha = Array(:, 2);
it = Array(:, 3);

figure
[X,Y] = meshgrid(epsilon,alpha);
f = scatteredInterpolant(epsilon,alpha,it);
Z = f(X,Y);

%raw data
plot3(epsilon,alpha,it)

%interpolation data
%mesh(X,Y,Z)

title('Determine best epsilon and alpha')
ylabel('Alpha') 
xlabel('Epsilon')
zlabel('Iterations')
%%
clear;
clc;
Array1=csvread('decaying_epsilon_random_marbles.csv');
%Array2 = csvread('non_decaying_epsilon_random_marbles.csv');
epsilon1=Array1(:, 1);
it1=Array1(:, 2);

epsilon2 = Array2(:, 1);
it2 = Array2(:, 2);

plot(epsilon1,it1)
%hold on
%plot(epsilon1,it2)
title('Epsilon decay rate')
%legend('Decreasing epsilon', 'Constant epsilon')
xlabel('epsilon decay rate')
ylabel('Iterations') 