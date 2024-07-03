%%% ----------------------------------------------------------------------------
% Neural EMS Controller for Electric Vehicle with hybrid storage - 1.0.
% Author: Alex do Nascimento Ribeiro (alexnascimentor@gmail.com)
% Intitution: Universidade de Brasília  (UnB)
% Department: Departamento de Engenharia Mecânica
% Local and data:  Brasília, Distrito Federal, Brasil  - 2024

% Main paper:
% Ribeiro, A. N., MUNOZ, D. M., "Neural Network Controller for Hybrid Energy
% Management System Applied to Electric Vehicles ", under avaliation in Journal of Energy Storage 

% 
% 
% 
%%% ----------------------------------------------------------------------------

close all
clear all


%%% Time
dt = 0.001;     % Sample Time [s]
t  = 0:dt:70;   % Time Vector [s]

nt=length(t);   % Number of Samples


%%% Vehicle Constants
g   = 9.8;                   % Gravity acceleration [m/s^2]
M   = 1000;                  % Mass [kg]
Cd  = 0.37;                  % Drag coefficient
Ad  = 2.2;                   % Effective area [m^2]
rho = 1.226;                 % Air density [kg/m^3]
Kat = 0.5*rho*Ad*Cd;         % Total Air drag
Crr = 0.012;                 % Rolling Resistance Coefficient
n   = 0.85;                  % Electromechanical conversion rate


%%% Time variable vectors
V  = 0*t;                   % Velocity [km/h]
theta = 0*V;                % Angle [rad]
a=0*V;                      % Acceleration [km/(h*s)]

a((10/dt):(32.5/dt))=4;     % Speed up  [km/(h*s)]
a((42.5/dt):(57.5/dt))=-6;  % Slow down [km/(h*s)]

Fnet = 0*V;                 % Net force [N]
Fair = 0*V;                 % Air drag force [N]
Frl = 0*V;                  % Rolling resistance [N]
Wtg = 0*V;                  % Weight tangent component [N]
Fm = 0*V;                   % Motive force [N]

Pm = 0*V;                   % Motive power [W]
Pload = 0*V;                % Electrical load power [J]

Em = 0*V;                   % Motive energy [W]
Eload = 0*V;                % Electrical load energy [J]


%%% Velocity calculation
for i=2:1:nt
    V(i)=V(i-1)+dt*(a(i)+a(i-1))/2; % in [km/h]
end

V(V<0)=0;                           % remove negative speed 

V=V/3.6;                            % now in [m/s]


%%% Power calculation
for i=2:1:nt
    a(i) = (V(i)-V(i-1))/dt;        % Acceleration in [m/s^2]
    
    Fair(i) = Kat*(V(i)^2);
    Frl(i)  = Crr*M*g*cos(theta(i))*sign( V(i) );
    Wtg(i)  = M*g*sin(theta(i));
end

Fnet = M*a;    
Fm = Fnet + Fair + Frl + Wtg;
Pm = Fm.*V;

Pload = Pm;
Pload(Pload>0)=Pload(Pload>0)/n;    % Electromechanical conversion
Pload(Pload<0)=Pload(Pload<0)*n;


%%% Energy calculation
for i=2:1:nt
    Em(i) = Em(i-1)+(Pm(i)+Pm(i-1))*dt/2;
    Eload(i) = Eload(i-1)+(Pload(i)+Pload(i-1))*dt/2;
    
end


%%%
figure(1)
plot(t,V*3.6)
title('Vehicle velocity in a micromovement')
xlabel('time [ s ]')
ylabel('Velocity [ km/h ]')
grid on

figure(2)
yyaxis left
plot(t,3.6*V)
ylim([-100 100])
ylabel('Velocity [ km/h ]')

yyaxis right
plot(t,3.6*a)
ylim([-8 8])
title('Vehicle velocity and acceleration')
xlabel('time [ s ]')
ylabel('Acceleration [ (km/h)/s ]')
legend('Speed','Acceleration')
grid on


figure(3)
yyaxis left
plot(t,V*3.6,'b--')
ylim([-100 100])
ylabel('Velocity [ km/h ]')

yyaxis right
plot(t,Fnet,'k',t,Fm,'r-',t,Fair,'g-',t,Frl,'c-',t,Wtg,'m-')
ylim([-1700 1700])
ylabel('Force [ N ]')
xlabel('time [ s ]')
title('Motive force and speed')
legend('Speed','Net force','Motive force','Air drag','Rolling resistance','Tangent Weight')
grid on


figure(4)
yyaxis left
plot(t,V*3.6)
ylim([-100 100])
ylabel('Velocity [ km/h ]')

yyaxis right
plot(t,Pm)
ylim([-40000 40000])
ylabel('Power [ W ]')
xlabel('time [ s ]')
title('Motive power and speed')
legend('Speed','Motive force')
grid on


figure(5)
plot(t,Pm,t,Pload)
ylabel('Power [ W ]')
xlabel('time [ s ]')
title('Motive power and electrical load')
legend('Motive power','Electrical load')
grid on


save("MicroMovCycle.mat","t","V","Pload")








