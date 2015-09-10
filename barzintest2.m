% barzintest2.m
close all
clear all


% FigHandle=figure(1);
% set(FigHandle, 'Position', [600, 0, 1600, 800]);
sim=Simulation();
sim.addOperatingRobots(6)
sim.addChargingRobots(3);
temp_var=10;
sim.list_of_operating_robots(1).setTrajectoryBarzinEdition(-temp_var*3,0);
sim.list_of_operating_robots(2).setTrajectoryBarzinEdition(temp_var*3,0);
sim.list_of_operating_robots(3).setTrajectoryBarzinEdition(-temp_var*3,-temp_var);
sim.list_of_operating_robots(4).setTrajectoryBarzinEdition(temp_var*3,-temp_var);
lawn_mower=[-2*temp_var 2*temp_var 2*temp_var -2*temp_var -2*temp_var 2*temp_var 2*temp_var -2*temp_var -2*temp_var 2*temp_var 2*temp_var -2*temp_var;
			temp_var temp_var 2*temp_var 2*temp_var 3*temp_var 3*temp_var 4*temp_var 4*temp_var 5*temp_var 5*temp_var 6*temp_var 6*temp_var];
sim.list_of_operating_robots(5).trajectory_x=temp_var*2+lawn_mower(1,:);
sim.list_of_operating_robots(6).trajectory_x=-temp_var*2+lawn_mower(1,:);
sim.list_of_operating_robots(5).trajectory_y=temp_var+lawn_mower(2,:);
sim.list_of_operating_robots(6).trajectory_y=temp_var+lawn_mower(2,:);
sim.list_of_operating_robots(5).segmentTrajectory();
sim.list_of_operating_robots(6).segmentTrajectory()
sim.plot();
a=sim.list_of_operating_robots(1)





















% close all
%% Set initial positions and speed of charging robots using ChargingRobot.m 
for i=1:length(sim.list_of_charging_robots)
	setpos(sim.list_of_charging_robots(i),i*100,10);
	setspeed(sim.list_of_charging_robots(i),999);
end

%% Set time step, meeting point time and charging time using Simulation.m

setTimeStep(sim,1);
setChargingTime(sim,5);
% setTotalSteps(sim);
% setMeetingTime(sim);

%% Calculated the tracjectories of charging robots using Simulation.m

plan(sim,'LKH','Distance');

sim.plot()
% close all
% %% Set initial positions and speed of charging robots using ChargingRobot.m
% for i=1:length(sim.list_of_charging_robots)
% setpos(sim.list_of_charging_robots(i),5,i*3);
% setspeed(sim.list_of_charging_robots(i),1);
% end


%% Set time step, meeting point time and charging time using Simulation.m
% setChargingTime(sim,5);
% setTotalSteps(sim);
% setMeetingTime(sim);
% plan(sim,'LKH','Distance');









% sim.setSimulationTime(1000);
% sim.plot()
% for i=1:1000
% 	clc
% 	i
% 	sim.plot(i);
% 	drawnow;
% 	pause(.01);
% end




% sim




% %% Set time step, meeting point time and charging time using Simulation.m
% setTimeStep(sim,1);
% setChargingTime(sim,5);
% setTotalSteps(sim);
% setMeetingTime(sim);
% plan(sim,'LKH','Distance');
% plotMap(sim);
% while t<simulation_time
% if ishandle(handle_1) delete(handle_1); end
% handle_1=plot(alpha.trajectory_x(t),alpha.trajectory_y(t),'k.','MarkerSize',30);
% if ishandle(handle_2) delete(handle_2); end
% handle_2=plot(beta.trajectory_x(t),beta.trajectory_y(t),'k.','MarkerSize',30);
% if ishandle(handle_3) delete(handle_3); end
% handle_3=plot(gamma.trajectory_x(t),gamma.trajectory_y(t),'k.','MarkerSize',30);
% if ishandle(handle_4) delete(handle_4); end
% handle_4=plot(charging_robot_trajectory_x(t),charging_robot_trajectory_y(t),'r.','MarkerSize',35);
% t=t+1;
% drawnow;
% if recordBool
% frame = getframe;
% writeVideo(SimulationVideo,frame);
% end
% pause(.1)
% end
%
%
% %% Calculated the tracjectories of charging robots using Simulation.m
%
% plan(sim,'LKH','Distance');
% plotMap(sim);
% % Make the video
% choice='No';
% dlg_string = sprintf('Do you want to record a video of results?\n\nThe name of the file will be the current date and time and it will be saved in the same directory.');
% %choice=questdlg(dlg_string,'Video Recording','Yes','No','Cancel','No');
% if(strcmp(choice,'Cancel')||isempty(choice))
% break
% end
% recordBool=strcmp(choice,'Yes');
% if recordBool
%
% c=fix(clock);
% cstr=strcat(mat2str(c),'.avi');
% SimulationVideo = VideoWriter(cstr);
% open(SimulationVideo);
% axis tight
% set(gca,'nextplot','replacechildren');
% set(gcf,'Renderer','zbuffer');
% end
%
% FigHandle=figure(1);
% % plotTrajectory(alpha)
% hold on
% % plotTrajectory(beta)
% % plotTrajectory(gamma)
% % plotTrajectory(sim)
%
% t=1;
% simulation_time=100;
% handle_1=[];
% handle_2=[];
% handle_3=[];
% handle_4=[];
%
% set(FigHandle, 'Position', [100, 100, 1049, 895]);
%
% charging_robot_trajectory_x=[];
% charging_robot_trajectory_y=[];
% % feasibility check
% number_of_meeting_points=size(sim.meeting_locations,1);
% for i=2:number_of_meeting_points
% distance=norm(sim.meeting_locations(i,:)-sim.meeting_locations(i-1,:));
% time_difference=(sim.meeting_times(i)-sim.meeting_times(i-1));
% required_speed=distance/time_difference;
% current_theta=atan2((sim.meeting_locations(i,2)-sim.meeting_locations(i-1,2)),(sim.meeting_locations(i,1)-sim.meeting_locations(i-1,1)));
% current_path=[1:time_difference]*required_speed;
% path_x=current_path*cos(current_theta)+sim.meeting_locations(i-1,1);
% path_y=current_path*sin(current_theta)+sim.meeting_locations(i-1,2);
% charging_robot_trajectory_x=[ charging_robot_trajectory_x, path_x];
% charging_robot_trajectory_y=[ charging_robot_trajectory_y, path_y];
% fprintf('For path #%i the distance is %.3f and minimum required speed is %.3f.\n',i-1,distance,required_speed)
% if(required_speed>sim.max_speed)||(time_difference<0)
% disp('ERROR: Speed criteria is not met.')
% return
% end
% end
%
% charging_robot_trajectory_x=[charging_robot_trajectory_x,ones(1,simulation_time-length(charging_robot_trajectory_y))*charging_robot_trajectory_x(end)];
% charging_robot_trajectory_y=[charging_robot_trajectory_y,ones(1,simulation_time-length(charging_robot_trajectory_y))*charging_robot_trajectory_y(end)];
%
% while t<simulation_time
% if ishandle(handle_1) delete(handle_1); end
% handle_1=plot(alpha.trajectory_x(t),alpha.trajectory_y(t),'k.','MarkerSize',30);
% if ishandle(handle_2) delete(handle_2); end
% handle_2=plot(beta.trajectory_x(t),beta.trajectory_y(t),'k.','MarkerSize',30);
% if ishandle(handle_3) delete(handle_3); end
% handle_3=plot(gamma.trajectory_x(t),gamma.trajectory_y(t),'k.','MarkerSize',30);
% if ishandle(handle_4) delete(handle_4); end
% handle_4=plot(charging_robot_trajectory_x(t),charging_robot_trajectory_y(t),'r.','MarkerSize',35);
% t=t+1;
% drawnow;
% if recordBool
% frame = getframe;
% writeVideo(SimulationVideo,frame);
% end
% pause(.1)
% end
%
% if recordBool
% close(SimulationVideo);
