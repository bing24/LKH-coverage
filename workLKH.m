
close all
clear all

% Load inputs
map = Map('map_image.bmp','resolution',20,'hieght',200);

% map.show('border')
represent(map)
% FigHandle=figure(1);

sim=Simulation();

sim.addOperatingRobots(1);
sim.addChargingRobots(5);
sim.setImageScale(.1);

Colors=[1 1 0;1 0 1;0 1 1;1 0 0;0 0 1];
for i =1:5
SetColor(sim.list_of_charging_robots(i),Colors(i,:));
end


initials=[2,3;3,7;8,9;10,7;12,3];
count=1;
for i = initials(:,1)'
    for j = find(map.mission_location(:,1)==i)'
        if map.mission_location(j,2) == initials(count,2)
            remo(count)=j;
        end 
    end
    count=count+1;
end 
map.mission_location(remo,:)=[];

sim.list_of_operating_robots.trajectory_x=map.mission_location(:,1);
sim.list_of_operating_robots.trajectory_y=map.mission_location(:,2);
chargers_positions=[2,3;3,7;8,9;10,7;12,3]+.0001;
for i = 1:5
setpos(sim.list_of_charging_robots(i),chargers_positions(i,1),chargers_positions(i,2));
	setspeed(sim.list_of_charging_robots(i),2);
end



setTimeStep(sim,1);
setChargingTime(sim,0);
a=sim.list_of_operating_robots(1);

sim.list_of_operating_robots.charging(1,:)=sim.list_of_operating_robots.trajectory_x;
sim.list_of_operating_robots.charging(2,:)=sim.list_of_operating_robots.trajectory_y;
sim.list_of_operating_robots.charging(3,:)=45;
plan(sim,'LKH','Distance');


sim.plot()
hold on
contour(map.matrix,1,'black','linewidth',5)
set(gca,'XTick',[])
set(gca,'YTick',[])
% pause (3)
% sim.simulate()
