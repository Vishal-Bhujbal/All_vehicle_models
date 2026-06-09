% Generating the function to run the simulation from the app directly
function out = runSimulation(modelname)
    if ~bdIsLoaded(modelname)
        load_system(modelname);
    end

    StopTime   = '20';
    Solver     = 'ode23s';

    % Simulation input setup
    simIn = Simulink.SimulationInput(modelname);
    simIn = simIn.setModelParameter('StopTime', StopTime);
    simIn = simIn.setModelParameter('Solver', Solver);    
    % Run the simulation
    out = sim(simIn);
end