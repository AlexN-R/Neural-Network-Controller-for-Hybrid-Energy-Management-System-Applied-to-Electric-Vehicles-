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

function [Output] = NeuronNetwork(Input,NetNeurons,Weights)
    
    Layers = length(NetNeurons);
    NetLayerWeights = [NetNeurons(2:end) 0].*(NetNeurons+1);

    W   = [];
    
    In  = Input;                    
    Sum = In;
    Out = 1./(1+exp(-Sum));        
    
    lim1 = 1;
    lim2 = 0;
   
    for L=2:1:Layers
        L;
        lim2 = lim2 + NetLayerWeights(L-1);                                     
        
        W = reshape( Weights(lim1:lim2), NetNeurons(L), NetNeurons(L-1)+1 );    
        
        Sum = W*[Out;1];                                                        
        
        Out = 1./(1+exp(-Sum.*exp(1)));
        
        lim1 = lim2+1;                                                          
                
    end
    
    Output = Out;

end

