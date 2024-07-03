# Neural-Network-Controller-for-Hybrid-Energy-Management-System-Applied-to-Electric-Vehicles-
This is the repository of sources and codes used in that paper "Neural Network Controller for Hybrid Energy Management System Applied to Electric Vehicles " ( under avaliation).

Title: Neural EMS Controller for Electric Vehicle with hybrid storage - 1.0.

Author: Alex do Nascimento Ribeiro (alexnascimentor@gmail.com).

Intitution: Universidade de Brasília  (UnB).

Department: Departamento de Engenharia Mecânica.

Local and data:  Brasília, Distrito Federal, Brasil  - 2024.

Main paper: Ribeiro, A. N., MUNOZ, D. M., "Neural Network Controller for Hybrid Energy
Management System Applied to Electric Vehicles ", under avaliation in Journal of Energy Storage. 

Instructions:
- The "Main_PSO.m" file will execute a optimization rotine that creates a neural controller for the power split problem in electric vehicles with hybrid storage.

- The main code uses "MicroMovCycle.mat" as training data, this set is generated by "MicromovementPower.m" code. Alternatively, a validation set, "EPAUDDSCycle.mat", is used and can generated by "CyclePower.m" code.

- The best particle found in this designing process can be loaded throught the "BestParticleXg.mat" file. The code "TestSolution.m" can test and present graphs for a particular solution under different initial conditions. It is already setted to show that best particle results.
