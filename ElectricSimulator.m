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

function [Fitness,Pcap,Pbat,Udc,Udc_error,Ubat,Ucap,Uibat,Uicap,Icap,Ibat,Icap_dc,Ibat_dc,Iload,Idc,Ibat_rms] = ElectricSimulator(NetNeurons,Weights,Ucap0,t,V,Pload,capON,emsON)

%[Fitness,ErroQuad,Pcap,Pbat,Udc,Udc_error,Ubat,Ucap,Uibat,Uicap,Icap,Ibat,Icap_dc,Ibat_dc,Iload,Idc,Ibat_rms] = ElectricSimulator(NetNeurons,Weights,na,nb,Ucap0,t,V,Pload,capON,emsON)

%%% Time variables
dt = t(2)- t(1);
nt = length(t);

%%% Performance results
Fitness  = 0;
Ibat_rms = 0;               % Battery RMS current


%%% Electric constants 
Cdc  = 0.005;               % DC capacitance [F]
Ccap = 3.55;                % Super capacitor capacitance [F]

Rcap = 0.1;                 % Super capacitor internal resistance [ohm]
Rbat = 0.3;                 % Battery internal resistance [ohm]

ConvTau = 0.1;              % DC/DC converter time constant [s]

Ucap_max = 486;             % Supercapacitor maximum voltage [V]
Ucap_min = 486*0.25;        % Supercapacitor minimum voltage [V]


%%% PID controller constants
KP = (1*10^4);
KI = (1*10^4);
KD = (0.3*10^3);


%%% Neural Input normalization constants
Pload_max = max(Pload);                     % Maximum electric load [W]

Vmax = 28;                                  % Maximum velocity [m/s]
Vmed = 14;                                  % Medium Velocity [m/s]

Udc_erro_max = 60;                          % DC link voltage normalization constant [V]


%%% Time varying vectors
    % Current [A]
    Icap_dc = 0*t;                  % Capacitor current (On DC bus side)
    Ibat_dc = 0*t;                  % Battery current   (On DC bus side)
    Iload   = 0*t;                  % Load current 
    Idc     = 0*t;                  % DC capacitor current 

    Icap   = 0*t;                   % Supercapacitor current (storage side)
    Ibat   = 0*t;                   % Battery current        (storage side)

    % Voltage [V]
    Udc    = 0*t;                   % DC bus voltage
    Ubat   = 0*t;                   % Battery terminal voltage
    Ucap   = 0*t;                   % Supercapacitor terminal voltage

    Uibat   = 0*t;                  % Battery internal voltage
    Uicap   = 0*t;                  % Supercapacitor internal voltage
    
    
    % PID controller error voltage
    Udc_error   = 0*t;               % Voltage control error
    Udc_error_I = 0*t;               % Voltage control error integral
    Udc_error_D = 0*t;               % Voltage control error derivative
    Udc_PID    = 0*t;                % PID controller action

    % Power [W]
    Pdc  = 0*t;                      % DC bus capacitor power

    Pbat = 0*t;                      % Battery power
    Pcap = 0*t;                      % Supercapacitor power

    Pbat_ref = 0*t;                  % Battery DC/DC converter power reference
    Pcap_ref = 0*t;                  % Supercapacitor DC/DC converter power reference

    
    % Neural network vectors
    NetInput = zeros(NetNeurons(1),nt);
    NetOutput = zeros(NetNeurons(end),nt);
    
    Pload_norm    = 0*t;            %Normalized input values
    V_norm        = 0*t;
    Udc_erro_norm = 0*t;
    Ucap_norm     = 0*t;


%%% time loop %%%
    % Initial conditions
    Udc0 = 600;                 % DC bus initial voltage
    Ubat0 = 300;                % Battery internal voltage
    
    Udc(1) = Udc0;              
    
    Ubat(1) = Ubat0;
    Ucap(1) = 486*0.75;

    Uibat(1:end)= 300;  
    Uicap(1) = Ucap0;
    
    for i=2:1:nt
        % Control action 
        
            % Pré processing (Normalization)            
            Pload_norm(i)    = Pload(i)/Pload_max;
            V_norm(i)        = (V(i)-Vmed)/(Vmax/2);
            Udc_erro_norm(i) =  Udc_error(i-1)/Udc_erro_max;
            Ucap_norm(i)     = ( Ucap(i-1) - 0.5*(Ucap_max+Ucap_min) )/( 0.5*(Ucap_max-Ucap_min) );
            
            
            % Network input
            NetInput(:,i)=[ Pload_norm(i-1) V_norm(i-1) Udc_erro_norm(i-1) Ucap_norm(i-1)]; 
                        
            
            % Neuron output
            NetOutput(:,i) = NeuronNetwork(NetInput(:,i),NetNeurons,Weights);            
            
            
            % Post processing (Denormalization)
            Pbat_ref(i) = emsON*(0.5*10^5)*2*(NetOutput(1,i)-0.5);
            Pcap_ref(i) = emsON*(0.5*10^5)*2*(NetOutput(2,i)-0.5)*capON;
           
            % PID controller
            Udc_error(i) = ( Udc0 - Udc(i-1) );                                     % DC link voltage error
            
            Udc_error_I(i) = Udc_error_I(i-1) + Udc_error(i)*dt;
            Udc_error_D(i) = (Udc_error(i)-Udc_error(i-1))/dt;
            
            Udc_PID(i) = KP*Udc_error(i) + KI*Udc_error_I(i) + KD*Udc_error_D(i);
            
            
            % Controllers actions sumation (Neural+PID)
            Pbat_ref(i) = Pbat_ref(i) + (0.2)*( Udc_PID(i) );
            Pcap_ref(i) = Pcap_ref(i) + (0.8)*( Udc_PID(i) )*capON;
            
            
        % DC/DC Converters dynamics 
        [Pbat(i)] = RK4RC(Pbat(i-1), Pbat_ref(i-1), Pbat_ref(i),dt,ConvTau);
        [Pcap(i)] = RK4RC(Pcap(i-1), Pcap_ref(i-1), Pcap_ref(i),dt,ConvTau);
        
        if ( Uicap(i-1)>(Ucap_max-1) ) && ( Pcap(i) < 0 )                       % Block supercapacitor overvoltage
            Pcap(i)=0;
        end
        
        if ( Uicap(i-1)<(Ucap_min+1) ) && ( Pcap(i) > 0 )                       % Block supercapacitor undervoltage
            Pcap(i)=0;
        end
        
        
        % Electrical circuit solution
        
            % DC bus
            Idc(i) = ( Pbat(i)+Pcap(i)-Pload(i) )/Udc(i-1);
            
            Udc(i) = Udc(i-1) + dt*( Idc(i) + Idc(i-1) )/(2*Cdc); 
            
            Icap_dc(i) = Pcap(i)/Udc(i);
            Ibat_dc(i) = Pbat(i)/Udc(i);
            Iload(i)   = Pload(i)/Udc(i);
            
            
            % Battery
            Ibat(i) = ( Uibat(i-1) - ( (Uibat(i-1)^2) - 4*Rbat*Pbat(i) )^(1/2) )/(2*Rbat);
            Ubat(i) = Uibat(i) - Ibat(i)*Rbat; % Melhor mudar para um modelo que liga potência direto com corrente?
                        
                        
            % Supercapacitor        
            Icap(i) = ( Uicap(i-1) - ( (Uicap(i-1)^2) - 4*Rcap*Pcap(i) )^(1/2) )/(2*Rcap);            
            Ucap(i)  = Uicap(i-1) - Icap(i)*Rcap;            
            Uicap(i) = Uicap(i-1) - dt*( Icap(i) + Icap(i-1) )/(2*Ccap); 
            
            % Fitness            
            Fitness = Fitness + dt*((Ibat(i)/1)^2);
            
                       
    end
    
    Ibat_rms = ( Fitness/( t(end)- t(2) ) )^(1/2);
end

