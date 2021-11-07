function u = inputvoltage(D,A,Delta,Ts)

% Duration of my input wave sampled with Ts
t = 0:Ts:D;

% Build the required square wave
freq = 1/Delta; % [Hertz]
u = A/2*square(2*pi*freq*t);
u = u'; % the outuput should be a coloumn vector as written in the text requirements

figure
plot(t,u);


end

