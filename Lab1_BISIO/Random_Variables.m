%% Autonomous Vehicles - Lab1 
%  Isabella-Sole Bisio

%% Part 1: Random variables

close all
clear 
clc

%% 1.1: A few univariate distributions
% Choose the required value of N

  N = 100;
% N = 4000;

%% Generate N realizations
% Gaussian distro
x_g = randn(N,1);

% Uniform distro
x_u = 2*sqrt(3)*(rand(N,1)-0.5);

% Gaussian mixture distro
m=0.95; 
x_gm=randn(N,1)*sqrt(1-m*m)+m; 
k=find(rand(N,1)>0.5); 
x_gm(k) = x_gm(k)-2*m;
 

%% Plot the normalized histogram of these N realizationsand and plot the probability density function on the same figure
% Gaussian distro
hist_pdf(N, x_g, 1);

% Uniform distro
hist_pdf(N, x_u, 2);

% Gaussian mixture distro
hist_pdf(N, x_gm, 3);


%% Estimate the mean and the standard deviation (mean, std).
% Gaussian distro
mean_g = mean(x_g);
fprintf('The mean of the Gaussian distribution is %d \n', mean_g);
std_g = std(x_g);
fprintf('The mean of the Gaussian distribution is %d \n', std_g);

% Uniform distro
mean_u = mean(x_u);
fprintf('The mean of the Uniform distribution is %d \n', mean_u);
std_u = std(x_u);
fprintf('The mean of the Uniform distribution is %d \n', std_u);


% Gaussian mixure distro
mean_gm = mean(x_gm);
fprintf('The mean of the Gaussian mixure distribution is %d \n', mean_gm);
std_gm = std(x_gm);
fprintf('The mean of the Gaussian mixure distribution is %d \n', std_gm);


%% Plot the N realizations as an independent random signal
figure
hold on
plot(x_g,'-b');
plot(x_u, '-r');
plot(x_gm, '-g');
hold off
title('N realizations as independent random signal');
xlabel('x');
ylabel('y');


%% 1.2 Joint distribution

%% Generate N = 200 realizations of this random variable
% My bivariate normal random variable is x = [x1 x2]' 
N2 = 200;
sig_x1 = 2;
sig_x2 = 5;
rho = 0.9;
m = 0; % mean of my r.v.
C = [sig_x1*sig_x1 rho*sig_x1*sig_x2 ; rho*sig_x1*sig_x2 sig_x2*sig_x2]; % variance my r.v.
x = chol(C,'lower')*randn(length(C),N2) + m*ones(1,N2);

%% Estimate the mean and the variance (mean, cov)

mean_r = mean(x'); % mean of my realisations
mean_realisation = [mean_r(1,1); mean_r(1,2)];
fprintf('The mean of my realisations is \n');
disp(mean_realisation)
var_r = cov(x'); % variance of my realisations
var_realisation = [var_r(1,1) var_r(2,1); var_r(1,2) var_r(2,2)];
fprintf('The variance of my realisations is \n');
disp(var_realisation)

%% Plot these realizations together with the 91 % confidence ellipse
P0 = 0.91;
t = linspace(0,2*pi,100);
X = sqrt(-2*log(1-P0))*chol(C,'lower')*[cos(t); sin(t)] + m*ones(1,length(t));
figure
hold on 
plot(x(1,:),x(2,:),'*r');
plot(X(1,:),X(2,:))
hold off
title('Realizations and the 91% confidence ellipse')



