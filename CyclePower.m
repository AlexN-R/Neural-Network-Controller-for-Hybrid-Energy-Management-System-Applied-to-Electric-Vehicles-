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

%%%%
Cycle = readtable("epa_gov_sites_default_files_2015-10_uddscol.txt");

OriginalTime = Cycle.Var1';                 % in [s]
OriginalVelocity = Cycle.Var2';             % in [mph]
%N_ori = length(OriginalTime);

%%% Final time and velocity
dt = 0.001;                                 % Sample time [s]
t  = OriginalTime(1):dt:OriginalTime(end);  % Time Vector [s]
V  = 0*t;                                   % Velocity [mph]
nt = length(t);                             % Number of samples


%%% Resample
j = 2;

T1 = OriginalTime(1);
T2 = OriginalTime(2);

V2 = OriginalVelocity(1);
V1 = OriginalVelocity(2);

V(1) = OriginalVelocity(1);

for i=1:1:nt                                % Resample loop
    
    if t(i)>T2
        
        j=j+1;
        
        T1 = T2;
        V1 = V2;
        
        T2 = OriginalTime(j);
        V2 = OriginalVelocity(j);
        
    end
    
    V(i) = V1 + (V2-V1)*(t(i)-T1)/(T2-T1); % Linear interpolation
    
end

V=V*0.44704;                               % Convert to [m/s]
V(V<0)=0;                                  % remove negative speed 


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
theta = 0*V;                % Angle [rad]
a=0*V;                      % Acceleration [km/(h*s)]

Fnet = 0*V;                 % Net force [N]
Fair = 0*V;                 % Air drag force [N]
Frl = 0*V;                  % Rolling resistance [N]
Wtg = 0*V;                  % Weight tangent component [N]
Fm = 0*V;                   % Motive force [N]

Pm = 0*V;                   % Motive power [W]
Pload = 0*V;                % Electrical load power [J]

Em = 0*V;                   % Motive energy [W]
Eload = 0*V;                % Electrical load energy [J]


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
ylim([-1.1*max(3.6*V) 1.1*max(3.6*V)])
ylabel('Velocity [ km/h ]')

yyaxis right
plot(t,3.6*a)
ylim([-1.1*max(abs(3.6*a)) 1.1*max(abs(3.6*a))])
title('Vehicle velocity and acceleration')
xlabel('time [ s ]')
ylabel('Acceleration [ (km/h)/s ]')
legend('Speed','Acceleration')
grid on


figure(3)
yyaxis left
plot(t,V*3.6,'b--')
ylim([-1.1*max(3.6*V) 1.1*max(3.6*V)])
ylabel('Velocity [ km/h ]')

yyaxis right
plot(t,Fnet,'k',t,Fm,'r-',t,Fair,'g-',t,Frl,'c-',t,Wtg,'m-')
ylim([-1.1*max(Fm) 1.1*max(Fm)])
ylabel('Force [ N ]')
xlabel('time [ s ]')
title('Motive force and speed')
legend('Speed','Net force','Motive force','Air drag','Rolling resistance','Tangent Weight')
grid on


figure(4)
yyaxis left
plot(t,V*3.6)
ylim([-1.1*max(3.6*V) 1.1*max(3.6*V)])
ylabel('Velocity [ km/h ]')

yyaxis right
plot(t,Pm)
ylim([-1.1*max(abs(Pm)) 1.1*max(abs(Pm))])
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


save("EPAUDDSCycle.mat","t","V","Pload")








