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

function [Done] = CreateGraphs(Fitness,Pcap,Pbat,Udc,Udc_error,Ubat,Ucap,Uibat,Uicap,Icap,Ibat,Icap_dc,Ibat_dc,Iload,Idc,t,Vel,Pload)
    Done = 1;
    
    close all
    figure(1)
    plot(t,Vel*3.6)
    title('Vehicle velocity')
    xlabel('time [ s ]')
    ylabel('velocity [ km/h ]')
    grid on

    figure(2)
    plot(t,Udc)
    title('DC link voltage')
    xlabel('time [ s ]')
    ylabel('voltage [ V ]')
    grid on

    figure(3)
    plot(t,Pload/1000,'k','LineWidth',1.5)
    hold on
    plot(t,Pbat/1000,'b',t,Pcap/1000,'r',[t(1) t(end)],[0 0],'k')
    legend('Pload','Pbat','Pcap')
    title('DC link power components')
    xlabel('time [ s ]')
    ylabel('power [ kW ]')
    grid on

    figure(4)
    plot(t,Iload,'k','LineWidth',1.5)
    hold on
    plot(t,Icap_dc,'r',t,Ibat_dc,'b',t,Idc,'m',[t(1) t(end)],[0 0],'k')
    title('DC link current components')
    xlabel('time [ s ]')
    ylabel('current [ A ]')
    legend('Iload','Icap','Ibat','Idc')
    grid on

    figure(5)
    plot(t,Icap,'r',t,Ibat,'b',[t(1) t(end)],[0 0],'k')
    title('Storage devices currents')
    xlabel('time [ s ]')
    ylabel('current [ A ]')
    legend('Icap','Ibat')
    grid on

    Ucap_max = 486;
    Ucap_min = 486*0.25;

    figure(6)
    plot(t,Ucap,t,Uicap,'r',[t(1) t(end)],[Ucap_max Ucap_max],'b--',[t(1) t(end)],[Ucap_min Ucap_min],'b--')
    title('Capacitor voltage')
    xlabel('time [ s ]')
    ylabel('voltage [ V ]')
    legend('terminal','internal')
    grid on


    figure(7)
    plot(t,Ubat,t,Uibat,'r')
    title('Battery voltage')
    xlabel('time [ s ]')
    ylabel('voltage [ V ]')
    legend('terminal','internal')
    grid on

    figure(8)
    yyaxis left
    plot(t,Ucap,'r',t,Uicap,'b--')
    ylim([Ucap_min Ucap_max])
    ylabel('Velocity [ km/h ]')
    
    yyaxis right
    plot(t,Vel*3.6,'k')
    ylim([-1.1*max(3.6*Vel) 1.1*max(3.6*Vel)])
    ylabel('Velocity [ km/h ]')
    title('Capacitor voltage and velocity')
    xlabel('time [ s ]')
    ylabel('Voltage [ V ]')
    legend('U internal','U terminal','Velocity')
    grid on
    
    
end

