classdef Configuration
    % CONFIGURATION: Genetic algorithm configuration.
    
    properties
        NumberofChargers=4;
        NumberofWorkers=5;
        MaximumIterations = 50; % Maximum of iterations to perform in the GA. Default value: 50.
        CrossoverRate = 0.8; % Crossover probability of the GA. Default value: 0.8 (80%).
        MutationRate = 0.2 % Mutation probability of the GA. Default value: 0.03 (3%).
        PopulationSize = 50; % Population size in the GA. Default value: 50.
        mutationProbability
        numberOfReplications 
        PopulationType = 'random'; % Population type of the GA. Possible values: 'random'/'custom'. Default value: 'random'.
    TournamentSize

    end
    
    methods
        function this = Configuration()
            % Creates a fsf.ga.Configuration instance.
            %
            % @return this: The fsf.ga.Configuration instance.
        end 
    end    
end