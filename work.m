% barzintest2.m
close all
clear all
clear class


% FigHandle=figure(1);
% set(FigHandle, 'Position', [600, 0, 1600, 800]);
sim=Simulation();

sim.addOperatingRobots(5);
sim.addChargingRobots(4);
sim.setImageScale(.1);

Colors=[1 1 0;1 0 1;0 1 1;1 0 0;0 0 1];
number_of_spicies = 5;
for agent_number = 1:number_of_spicies
    
    SetColor(sim.list_of_operating_robots(agent_number),Colors(agent_number,:));
    
end

load 58;




for i = 1:5
	LKHchange(population(i),optimizor);
	population(i).LKHevaluate;
    sim.list_of_operating_robots(i).trajectory_x=population(i).LKHx;
    sim.list_of_operating_robots(i).trajectory_y=population(i).LKHy;
end




for i = 1:4
setpos(sim.list_of_charging_robots(i),i+1,i+2);
	setspeed(sim.list_of_charging_robots(i),2);
end



setTimeStep(sim,1);
setChargingTime(sim,1);
a=sim.list_of_operating_robots(1);

plan(sim,'LKH','Distance');


sim.plot()
contour(map.matrix,1,'black','linewidth',5)
set(gca,'XTick',[])
set(gca,'YTick',[])
% pause (3)
% sim.simulate()
for agent_number = 1:number_of_spicies
        % population(agent_number)= InitializePopulation(map, gaConfig);
        Evaluating(population(agent_number),map,gaConfig);
    end
plotWorking(optimizor,population,gaConfig)
figure(3)
contour(map.matrix,1,'black','linewidth',5)
figure(1)
contour(map.matrix,1,'black','linewidth',5)