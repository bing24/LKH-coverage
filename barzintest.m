close all 
clear all
simulation_time=1000;


or=OperatingRobot();
or.setTrajectoryBarzinEdition([0 2000],[0 0],'set');
or.scale_coef=.2;
or.plot();
axis equal
hold on
for i=1:or.simulation_time
	% clc
	or.plot(i);
	% i
	pause(.01);
end
disp('###################### Simulation Finished ######################');