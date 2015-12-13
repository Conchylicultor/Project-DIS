clearvars;
close all;
clc;

results = load('fitnessEvolve.csv');

figure;
hold on;
plot(results(:,1));
plot(results(:,2));

title('Evolution of the fitness with PSO');

legend('Best', 'Average', 'Location', 'northwest');

ylabel('Fitness')
xlabel('Iterations');
