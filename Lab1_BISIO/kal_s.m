function [xe_s] = kal_s(y, u, G, T, Ts, L, x1_0, q, flag)

%% Initialisation 
% Initialisation of my system matrices in continuous time

A = [0 1 ; 0 (-1/T)];
B = [0 ; G/T];
C = [1 0];
D = 0;


% Pass from continous to descrete time 

[Ad,Bd,Cd,Dd] = c2dm(A,B,C,D,Ts,'zoh');


% Define the variance of the quantisation error

R = ((2*pi / L)^2)/12; % Quantization error: (LSB^2)/12


% Define the length of the vector u in order to loop the KF

j = length(u);


% Use a simplified stationary KF

[K, P, Z, E] = dlqe(Ad, Bd, Cd, q, R);


%% Kalman filter loop
 
for n = 1:j
    
    if n >= 2     
        
        %RECURSION
        % prediction of the nth observation
        Ynm1 = Cd * Xnm1 + Dd * u(n);
        
        %OBSERVATION
        yn = y(n);   

        % estimation of the nth state 
        Xn = Xnm1 + K*(yn-Ynm1);

        % prediction of the (n+1)th state
        Xnp1 = Ad*Xn +Bd*u(n);

        % update the state
        Xnm1 = Xnp1;
        
        % Fill the output vector
        xe_s(n,1) = Xnm1(1);
        xe_s(n,2) = Xnm1(2);
    else
        
        % First loop, first state prediction
        Xnm1 = x1_0;
        
        % Fill the output vector
        xe_s(n,1) = x1_0(1);
        xe_s(n,2) = x1_0(2);
   
    end
end 
    

%% Plot for test

% if flag == 0
%     figure
%     plot(xe_s(:,1));
%     title('Anlge trend in Stationary KF');
%     xlabel('time');
%     ylabel('amplitude');
%     figure
%     plot(xe_s(:,2))
%     title('Angular velocity trend in Stationary KF');
%     xlabel('time');
%     ylabel('amplitude');
% end
  


end

