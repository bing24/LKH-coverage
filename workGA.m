clear all;
close all;
tic

% Load inputs
map = Map('map_image.bmp','resolution',20,'hieght',200);

% map.show('border')
represent(map)

% encoding(map)
% GA config
gaConfig = Configuration();
gaConfig.MaximumIterations = 50;
gaConfig.PopulationSize = 50;
gaConfig.PopulationType = 'random';
gaConfig.CrossoverRate = 0.8;
gaConfig.MutationRate = 0.03;
gaConfig.TournamentSize=10;
gaConfig.mutationProbability=0.01;
gaConfig.numberOfReplications = 10;

optimizor=galayer(5,4);
contour(map.matrix,1,'black','linewidth',5)
hold on
plot(map.mission_location(:,1),map.mission_location(:,2),'.')
figure

% Generatue initial populations
initials=[2,3;3,7;8,9;10,7;12,3];
Colors=[1 1 0;1 0 1;0 1 1;1 0 0;0 0 1];
number_of_spicies = 5;
for agent_number = 1:number_of_spicies
    population(agent_number)= InitializePopulation(map, gaConfig);
    SetInitial(population(agent_number),initials(agent_number,:));
    SetColor(population(agent_number),Colors(agent_number,:));
    Evaluating(population(agent_number),map,gaConfig);
end
% Generate charging robots
number_of_charging_robots=4;
for chargers_number=1:number_of_charging_robots
	chargers(chargers_number)=charging_robots(Colors(chargers_number,:),chargers_number);
end
% First evaluation
EvaluatingWorking(optimizor,population,map,gaConfig);
for agent_number = 1:number_of_spicies
	population(agent_number).fitness=optimizor.fitness;
	population(agent_number).bestIndividualIndex=optimizor.bestIndividualIndex;
end
% Selection and crossover
for agent_number = 1:number_of_spicies
	Selecting(population(agent_number),gaConfig,0.5);
end
% Mutate
randIndexes = ceil(rand(1,gaConfig.numberOfReplications).*gaConfig.PopulationSize);
for agent_number = 1:number_of_spicies
	Mutating(population(agent_number),gaConfig,randIndexes)
end

generation=1000;
% optimizing coverage
while sum(optimizor.cover==1)==0 && optimizor.count<1000
	for agent_number = 1:number_of_spicies
	    % population(agent_number)= InitializePopulation(map, gaConfig);
	    Evaluating(population(agent_number),map,gaConfig);
	end
		EvaluatingCover(optimizor,population,map,gaConfig);
	for agent_number = 1:number_of_spicies
		population(agent_number).fitness=optimizor.fitness;
		population(agent_number).bestIndividualIndex=optimizor.bestIndividualIndex;
	end
	for agent_number = 1:number_of_spicies
		Selecting(population(agent_number),gaConfig,0.5);
	end
		% Mutate
		randIndexes = ceil(rand(1,gaConfig.numberOfReplications).*gaConfig.PopulationSize);
	for agent_number = 1:number_of_spicies
		Mutating(population(agent_number),gaConfig,randIndexes)
	end
	
	plotWorking(optimizor,population,gaConfig)
	

	drawnow
end
%optimizing working distance
while  sum(abs(diff(optimizor.record_dis_working(end-200:end))))~=0 && optimizor.count<2000
% while  optimizor.minimumFitness>57 && optimizor.count<2000
	for agent_number = 1:number_of_spicies
	    % population(agent_number)= InitializePopulation(map, gaConfig);
	    Evaluating(population(agent_number),map,gaConfig);
	end
		EvaluatingWorking(optimizor,population,map,gaConfig);
	for agent_number = 1:number_of_spicies
		population(agent_number).fitness=optimizor.fitness;
		population(agent_number).bestIndividualIndex=optimizor.bestIndividualIndex;
	end
	for agent_number = 1:number_of_spicies
		Selecting(population(agent_number),gaConfig,0.5);
	end
		% Mutate
		randIndexes = ceil(rand(1,gaConfig.numberOfReplications).*gaConfig.PopulationSize);
	for agent_number = 1:number_of_spicies
		Mutating(population(agent_number),gaConfig,randIndexes)
	end
	
	plotWorking(optimizor,population,gaConfig)
	
	drawnow
end
%cooperative part
% figure
% contour(map.matrix,1,'black','linewidth',5)
% hold on
% plot(map.mission_location(:,1),map.mission_location(:,2),'.')
% for i=1:generation
    
% 	for agent_number = 1:number_of_spicies
% 	    % population(agent_number)= InitializePopulation(map, gaConfig);
% 	    Evaluating(population(agent_number),map,gaConfig);
% 	end
% 		EvaluatingAll(optimizor,population,map,gaConfig,chargers,randIndexes);
% 	for agent_number = 1:number_of_spicies
% 		population(agent_number).fitness=optimizor.fitness;
% 		population(agent_number).bestIndividualIndex=optimizor.bestIndividualIndex;
% 	end
% 	for agent_number = 1:number_of_spicies
% 		Selecting(population(agent_number),gaConfig,0.5);
% 	end
% 		% Mutate
% 		randIndexes = ceil(rand(1,gaConfig.numberOfReplications).*gaConfig.PopulationSize);
% 	for agent_number = 1:number_of_spicies
% 		Mutating(population(agent_number),gaConfig,randIndexes)
% 	end
% 	plotall(optimizor,population,chargers,gaConfig)
% 	drawnow
% end

% Evaluating(population(agent_number),map,gaConfig);
% plotall(optimizor,population,chargers,gaConfig)
toc
% obj=population;

obj=optimizor;