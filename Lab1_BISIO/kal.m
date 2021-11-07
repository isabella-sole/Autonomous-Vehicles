function [xe] = kal(y, u, G, T, Ts, L, x1_0, P1_0, q, flag)

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



%% Kalman filter loop
 
for n = 1:j
    
    if n >= 2     
        
        %RECURSION
        % prediction of the nth observation
        Ynm1 = Cd * Xnm1 + Dd * u(n);
        Cxy = Pnm1 * Cd';
        Cyy = Cd * Pnm1 * Cd' + R;
        
        %OBSERVATION
        yn = y(n);   

        % estimation of the nth state 
        Xn = Xnm1 + Cxy * inv(Cyy) * (yn-Ynm1);
        Pn = Pnm1 - Cxy * inv(Cyy) * Cxy';

        % prediction of the (n+1)th state
        Xnp1 = Ad * Xn + Bd * u(n);
        Pnp1 = Ad * Pn * Ad' + Bd * q * Bd';

        % update the state
        Xnm1 = Xnp1;
        Pnm1 = Pnp1;
        
        % Fill the output vector
        xe(n,1) = Xnm1(1);
        xe(n,2) = Xnm1(2);
    else
        
        % First loop, first state prediction
        Xnm1 = x1_0;
        Pnm1 = P1_0;
        
        % Fill the output vector
        xe(n,1) = x1_0(1);
        xe(n,2) = x1_0(2);
   
    end
end 
    

%% Plot for test

% if flag == 1
%     figure
%     plot(xe(:,1));
%     title('Angle trend in KF');
%     xlabel('time');
%     ylabel('amplitude');
%     figure
%     plot(xe(:,2));
%     title('Angular velocity trend in KF');
%     xlabel('time');
%     ylabel('amplitude');
% end
  


end

