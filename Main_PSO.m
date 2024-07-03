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

clear all

%%% Load the training cycle
ciclo = load("MicroMovCycle.mat");

t = ciclo.t;                    % Time [s]
Vel = ciclo.V;                  % Velocity [m/s]
Pload = ciclo.Pload;            % Electrical load [W]


%%% Neural Network size %%%
NetNeurons = [4,10,40,40,2];                                % Neurons at each layer
NetLayerWeights = [NetNeurons(2:end) 0].*(NetNeurons+1);    % Weights plus bias at each layer
NetLayers = length(NetNeurons);                             % Number of layers

%%% Capacitor and EMS action 
capON=1;                                                    % Capacitor action (ON=1)
emsON=1;                                                    % EMS action (ON=1)

%%% Opmization %%%

    % Opmization dimension
    Ne = 1;                                                 % Epoch
    Ni = 1;                                               % Iterations 
    Np = 20;                                                % Particles
    Nd = sum([NetNeurons(2:end) 0].*(NetNeurons+1));        % Dimensions (total Weights plus bias count)
    
   % PSO Constants 
    u0 = 1.2;                       % Initual inertial value                 
    uf = 0.8;                       % Final inertial value

    c1 = 2.05;                      % Cognitive coefficient
    c2 = 2.05;                      % Social coefficient
    
    XLim = 4;                       % Space limit
    DeltaXMax = 4;                  % Maximum displacement
   
    Xmax=2;                         % Maximum initial coordinate value  
    Xmin=-2; %TIRAR?                % Minimum initial coordinate value  

    
% Training variables
    %Optimization variables
    CurrentFitness = 10^99;                                 % Fitness do vetor trial
    BestFitness = 10^99;                                    % Epoch best fitness
    
    % Matrizes de otimização
    %P  = 5*2*(rand(Np,Nd)-0.5);                            % Matriz de Partículas
      
    %Optimization vectors
    BestFitIter = zeros(1,Ni);                              % Best iteration fitness
    %FitVector = (10^99)*ones(Np,1);                        % Fitness de cada partícula
    FitVectorEpoch = (10^99)*zeros(Ne,1);                   % Best fitness within epochs
   
    
%%% Optimization Loop %%%
for e=1:1:Ne                                            % Epoch loop
    
    % PSO matrices
    X1 = Xmax*2*( rand(Np,Nd)-0.5*ones(Np,Nd) );        % Current particles positions (randomly sorted)
    X0 = X1;                                            % Previous particles positions

    Xb = X1;                                            % Best particles positions
    Xg = Xb(1,:);                                       % Best group position
    
    DeltaX1 = DeltaXMax*2*( rand(Np,Nd)-0.5*ones(Np,Nd) );   % Current  particles displacement
    DeltaX0 = DeltaX1;                                  % Previous particles displacement
   
    F  = (10^99)*ones(Np,1);                            % Particles current fitness
    Fb = F;                                             % Particles best known fitness
    Fg = (10^99);                                       % Group best known fitness
    
    %X_iter = zeros(Ni,Nd); %REMOVER
    
    u = zeros(1,Ni);                                    % Inertial factor
    
    r1= zeros(Np,Nd);                                   % Cognitive random vector 
    r2= zeros(Np,Nd);                                   % Social random vector
    
    
    %%%%
    for iter=1:1:Ni                                     % Iteration loop
       
       u(iter)= u0 + iter*(uf-u0)/iter;                 % New inertial value 
       
       DeltaX0 = DeltaX1;                               % Vectors actualization
       X0 = X1;
       
       r1 = rand(Np,Nd);                                % Sort new r1 and r2
       r2 = rand(Np,Nd);
       
       %Calculate a new displacement       
       DeltaX1 = u(iter)*DeltaX0 + c1*r1.*(Xb-X0) + c2*r2.*( (ones(Np,1).*Xg) - X0 );  
              
       
       for part=1:1:Np                                                 % Particles loop
           DeltaXMod = norm(DeltaX1(part,:));                          % Displacement absolute value

           if DeltaXMod>DeltaXMax                                      % Correct the displacement if it is greater than its maximum
               DeltaX1(part,:)=DeltaXMax*(DeltaX1(part,:)/DeltaXMod);                 
           end


           %Calculate a new position
           X1(part,:) = X0(part,:) + DeltaX1(part,:);
           
           
           for coord=1:1:Nd                                            % Coordinates loop
               
                if X1(part,coord) > XLim                               % Verify a coordinate going beyond the limit
                   X1(part,coord) = XLim;
                end
                
                 if X1(part,coord) < -XLim
                    X1(part,coord) = -XLim;
                end

           end
           
          
           Ucap0=486*0.625;                                             % Capacitor initial voltage
           
           % CORRIGIR
           % Call the electric simulation
           % [CurrentFitness,ErroQuad,Pcap,Pbat,Udc,Udc_erro,Ubat,Ucap,Uibat,Uicap,Icap,Ibat,Icap_dc,Ibat_dc,Iload,Idc,Ibat_rms] = SimuladorEletrico(NetNeurons,X1(part,:),na,nb,Ucap0,t,Vel,Pload,capON,emsON);
           [CurrentFitness,Pcap,Pbat,Udc,Udc_error,Ubat,Ucap,Uibat,Uicap,Icap,Ibat,Icap_dc,Ibat_dc,Iload,Idc,Ibat_rms] = ElectricSimulator(NetNeurons,X1(part,:),Ucap0,t,Vel,Pload,capON,emsON);
           F(part) = CurrentFitness;           
         
           
           %Fitness verification    
           if F(part) < Fg                         % Verify is the current fitness is better than the best known by the particle
                Fg = F(part);                      % Save this fitness
                Xg = X1(part,:);                   % Save this position
                                
           end 
            
           if F(part) < Fb(part)                   % Verify is the current fitness is better than the best known by the group
                Fb(part) = F(part);                % Set as best fitness
                Xb(part,:) = X1(part,:);           % Set as best position
                
           end  
           
       end
       
        % Print the iteration information
        fprintf(' Iteration nº %2.1f // Best fitness = %f\n ',iter,Fg);
        fprintf('\n');
        BestFitIter(iter) = Fg;
        
    end
    FitVectorEpoch(e)=BestFitness;

end

% Execute the simulation with best particle 
Ucap0 = 486*0.675;
[CurrentFitness,Pcap,Pbat,Udc,Udc_error,Ubat,Ucap,Uibat,Uicap,Icap,Ibat,Icap_dc,Ibat_dc,Iload,Idc,Ibat_rms] = ElectricSimulator(NetNeurons,Xg,Ucap0,t,Vel,Pload,capON,emsON);

%%% Shows best particle graphs %%%
close all
CreateGraphs(CurrentFitness         ,Pcap ,Pbat ,Udc ,Udc_error ,Ubat ,Ucap ,Uibat ,Uicap ,Icap ,Ibat ,Icap_dc ,Ibat_dc ,Iload ,Idc , t, Vel, Pload);
    


