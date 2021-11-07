%% Autonomous Vehicles - Lab1 
%  Isabella-Sole Bisio

%% Part 2: Kalman Filter

close all
clear 
clc

%% 2.1 Input voltage 

Delta = 0.1; % Period [sec] 
A = 0.1; % Peak-to-peak amplitude [volt]
Ts = 0.001; % Sample time [sec]
D = 1; % Duration [sec]

u= inputvoltage(D,A,Delta,Ts);

%% 2.2 System modeling and simulation

G = 50; % [rad/(sec*volt)]
T = 0.02; % [sec] Period
L = 512; % Precision of the encoder [angles per lap]
x1 = [0 ; 0]; % Initial state vector


[y,x] = simulate(u,G,T,Ts,L,x1);


%% 2.3 Kalman Filter


% Propose an initialization of the prior information and propose a value for its variance

x1_0 = [0 ; 0]; % giustified by what is written in the PDF 
P1_0 = [((2*pi)^2)/12 0 ; 0 0]; % value of its variance
q = 0.005; % trust of the system


% State estimation using Kalman Filter

xe = kal(y, u, G, T, Ts, L, x1_0, P1_0, q, 1);


% State estimation using Stationary Kalman Filter
% Since matrices Ad, Cd, Q, R are time independent I can use this simplified version of the filter

xe_s = kal_s(y, u, G, T, Ts, L, x1_0, q, 0);


%% 2.4 Simulations
%% Case 1: the model of the system is perfect

% Set the simulation parameters

G = 50; % [rad/(sec*volt)]
T = 0.02; % [sec] Period
L = 512; % Precision of the encoder [angles per lap]
x1 = [0 ; 0]; % Initial state vector


% Simulation of the model

[y_model,x_model] = simulate(u,G,T,Ts,L,x1);


% Try to use different values of q

 q = 0.005;
% q = 0.1;
% q = 0.0005;


P_1 = [((2*pi)^2)/12, 0 ;0, 0];
x1_est = [x1(1)+0.05; x1(2)]; % Set the initial value suggested by the PDF
%x1est = [x1(1)-0.05; x1(2)];

% Simulation of the Kalman filter
Xe1 = kal(y,u,G,T,Ts,L,x1_est,P_1,q,1);

% Simulation of the Stationary Kalman filter
Xes1 = kal_s(y,u,G,T,Ts,L,x1_est,q,0);


%% Case 2: the model of the system is rough

% Add a variable with this model since Tactual and Tfilter do not correspond
Tfilt = 0.025; % [sec] Period


% Simulate the filters
Xe2 = kal(y,u,G,Tfilt,Ts,L,x1_est,P_1,q,1);
Xes2 = kal_s(y,u,G,Tfilt,Ts,L,x1_est,q,0);


%% PLOT THE COMPARISONS

% Angle comparison
figure;
subplot(2,1,1);
title('Angle comparison between real system, KF and static KF - CASE 1');
xlabel('time');
ylabel('amplitude');
hold on;
plot(x_model(:,1), '-r'); %Red, Actual model 
plot(Xe1(:,1), '-g'); %Green, KF
plot(Xes1(:,1), '-b'); %Blue, Static KF
legend({'x(:,1)','Xe(:,1)','Xes(:,1)'});
hold off;

% Angle comparison
subplot(2,1,2);
title('Angle comparison between real system, KF and static KF - CASE 2');
xlabel('time');
ylabel('amplitude');
hold on;
plot(x(:,1), '-r'); %Red, Actual model 
plot(Xe2(:,1), '-g'); %Green, KF
plot(Xes2(:,1), '-b'); %Blue, Static KF
legend({'x(:,1)','Xe(:,1)','Xes(:,1)'});
hold off;


% Velocity comparison
figure;
subplot(2,1,1);
title('Ang. velocity comparison between real system, KF and static KF - CASE 1');
xlabel('time');
ylabel('amplitude');
hold on;
plot(x_model(:,2), '-r'); %Red, Actual model 
plot(Xe1(:,2), '-g'); %Green, KF
plot(Xes1(:,2), '-b'); %Blue, Static KF
legend({'x(:,2)','Xe(:,2)','Xes(:,2)'});
hold off;

% Velocity comparison
subplot(2,1,2);
title('Ang. velocity comparison between real system, KF and static KF - CASE 2');
xlabel('time');
ylabel('amplitude');
hold on;
plot(x(:,2), '-r'); %Red, Actual model 
plot(Xe2(:,2), '-g'); %Green, KF
plot(Xes2(:,2), '-b'); %Blue, Static KF
legend({'x(:,2)','Xe(:,2)','Xes(:,2)'});
hold off;

