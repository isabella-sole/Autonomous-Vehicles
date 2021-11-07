function [y,x] = simulate(u,G,T,Ts,L,x1)

% Initialisation of my model matrices

A = [0 1 ; 0 (-1/T)];
B = [0 ; G/T];
C = [1 0];
D = 0;


% Pass from continous to descrete time 

[Ad,Bd,Cd,Dd] = c2dm(A,B,C,D,Ts,'zoh');


% Innitialise the state and the matrix x

x_n = x1;
x = zeros(length(u),2);

% Iteration

for i = 1: length(u)
    
    x_nn = Ad * x_n + Bd * u(i); % Compute the state given the input
    theta = Cd * x_n + Dd * u(i); % Compute the output given the input
    y_n(i) = round((theta*L/2)/pi)*2*pi/L; % Observation of the angle with the encorder
    
    x(i,1) = x_n(1); % Fill first coloumn of x
    x(i,2) = x_n(2); % Fill second coloumn of x
    x_n = x_nn; % Update the state
    
    
     
end

y = y_n'; % y should be a coloumn vector


end

