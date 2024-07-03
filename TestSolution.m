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

%%% Select a Cycle

    cycle = load("MicroMovCycle.mat");
    %cycle = load("EPAUDDSCycle.mat"); %alternative

    t = cycle.t;
    Vel = cycle.V;
    Pload = cycle.Pload;

%%% Enter the neural network size %%%
    NetNeurons = [4,10,40,40,2];


% Select a particle
    Xb = load("BestParticleXg.mat");                                          
    
    Xg = Xb.Xg;                                       % particle


%%% Testing of multiple condition
    capON=1; emsON=1; %Capacitor ON and EMS ON
    
    %%% Test 1
    Ucap0 = 486*0.4375;       % Set voltage initial condition
    %capON=0; emsON=0;
    [Fitness ,Pcap ,Pbat ,Udc ,Udc_error ,Ubat ,Ucap ,Uibat ,Uicap ,Icap ,Ibat ,Icap_dc ,Ibat_dc ,Iload ,Idc ,Ibat_rms] = ElectricSimulator(NetNeurons,Xg ,Ucap0,t,Vel,Pload,capON,emsON);
    %%% Test 2
    Ucap0 = 486*0.625;
    %capON=1; emsON=0;
    [Fitness2 ,Pcap2,Pbat2,Udc2,Udc_error2,Ubat2,Ucap2,Uibat2,Uicap2,Icap2,Ibat2,Icap_dc2,Ibat_dc2,Iload2,Idc2,Ibat_rms2] = ElectricSimulator(NetNeurons,Xg ,Ucap0,t,Vel,Pload,capON,emsON);
    
    %%% Test 3
    Ucap0 = 486*0.8125;
    %capON=1; emsON=1;
    [Fitness3 ,Pcap3,Pbat3,Udc3,Udc_error3,Ubat3,Ucap3,Uibat3,Uicap3,Icap3,Ibat3,Icap_dc3,Ibat_dc3,Iload3,Idc3,Ibat_rms3] = ElectricSimulator(NetNeurons,Xg ,Ucap0,t,Vel,Pload,capON,emsON);
    
    
%%% Graphic creation %%%
    close all

    %%% Singular solution Graphics
    
    %CreateGraphs(Fitness ,Pcap ,Pbat ,Udc ,Udc_error ,Ubat ,Ucap ,Uibat ,Uicap ,Icap ,Ibat ,Icap_dc ,Ibat_dc ,Iload ,Idc ,t,Vel,Pload);
    CreateGraphs(Fitness2,Pcap2,Pbat2,Udc2,Udc_error2,Ubat2,Ucap2,Uibat2,Uicap2,Icap2,Ibat2,Icap_dc2,Ibat_dc2,Iload2,Idc2,t,Vel,Pload);
    %CreateGraphs(Fitness3,Pcap3,Pbat3,Udc3,Udc_error3,Ubat3,Ucap3,Uibat3,Uicap3,Icap3,Ibat3,Icap_dc3,Ibat_dc3,Iload3,Idc3,t,Vel,Pload);

    
    %%% Different solution comparison 
    
    figure(9)
    plot(t,Ibat,'k',t,Ibat2,'b',t,Ibat3,'r',[t(1) t(end)],[0 0],'k')
    title('Battery Current for different capacitor initial voltage')
    xlabel('time [ s ]')
    ylabel('current [ A ]')
    legend('25% of total range','50% of total range','75% of total range')
    grid on

    Ucap_max = 486;
    Ucap_min = 486*0.25;

    figure(10)
    plot(t,Uicap,'k',t,Uicap2,'b',t,Uicap3,'r',[t(1) t(end)],[0 0],'k',[t(1) t(end)],[Ucap_max Ucap_max],'b--',[t(1) t(end)],[Ucap_min Ucap_min],'b--')
    title('Capacitor voltage with different initial conditions ')
    xlabel('time [ s ]')
    ylabel('voltage [ V ]')
    legend('25% of total range','50% of total range','75% of total range')
    grid on

    figure(11)
    plot(t,Icap,'k',t,Icap2,'b',t,Icap3,'r')
    title('Capacitor current with different initial conditions ')
    xlabel('time [ s ]')
    ylabel('current [ A ]')
    legend('25% of total range','50% of total range','75% of total range')
    grid on

    figure(12)
    subplot(2,1,1);
    plot(t,Uicap,'k',t,Uicap2,'b',t,Uicap3,'r',[t(1) t(end)],[0 0],'k',[t(1) t(end)],[Ucap_max Ucap_max],'b--',[t(1) t(end)],[Ucap_min Ucap_min],'b--')
    title('Capacitor voltage with different initial conditions ')
    xlabel('time [ s ]')
    ylabel('voltage [ V ]')
    legend('25% of total range','50% of total range','75% of total range')
    grid on

    subplot(2,1,2);
    plot(t,Icap,'k',t,Icap2,'b',t,Icap3,'r')
    title('Capacitor current with different initial conditions ')
    xlabel('time [ s ]')
    ylabel('current [ A ]')
    grid on

    
    

