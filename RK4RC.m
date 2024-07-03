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

function [Y2] = RK4RC(Y1,In1,In2,dt,RC)
   
    k1=0;
    k2=0;
    k3=0;
    k4=0;
    
    k1 = (In1-Y1)/(RC);
    k2 = ( ((In2+In1)/2) - (Y1 + dt*k1/2) )/(RC);
    k3 = ( ((In2+In1)/2) - (Y1 + dt*k2/2) )/(RC);
    k4 = (    (In2)      -  (Y1 + dt*k3)  )/(RC);
    
    
    Y2 = Y1 + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
end

