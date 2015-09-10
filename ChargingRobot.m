classdef ChargingRobot < handle
    
    properties
        id
        simulation_time=20
        initial_x
        initial_y
        path_x
        path_y
        trajectory_x
        trajectory_y
        max_speed=2
        meeting_locations
        meeting_times
        figure_handle=[]
        scale_coef=.01
        charging_period_time=0
        color;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj=ChargingRobot()
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setID(obj, ID_number)
            obj.id=ID_number;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setImageScale(obj,scale_coef)
            obj.scale_coef=scale_coef;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setpos(obj,xini,yini)
            obj.initial_x=xini;
            obj.initial_y=yini;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setSimulationTime(obj, simulation_time)
            obj.simulation_time=simulation_time;
            obj.trajectory_x=NaN*ones(1,simulation_time);
            obj.trajectory_y=NaN*ones(1,simulation_time);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setspeed(obj,speed)
            obj.max_speed=speed;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function segmentTrajectory(obj)
            number_of_meeting_points=length(obj.path_x);
            temp_trajectory_x=[];
            temp_trajectory_y=[];
            for i=2:number_of_meeting_points
                position_matrix=[obj.path_x(i-1:i)', obj.path_y(i-1:i)'];
                distance_between_points = diff(position_matrix,1);
                dist_from_vertex_to_vertex = hypot(distance_between_points(:,1), distance_between_points(:,2));
                cumulative_dist_along_path = [0; cumsum(dist_from_vertex_to_vertex,1)];
                if obj.meeting_times(i-1)==0
                    speed=dist_from_vertex_to_vertex/obj.meeting_times(2);
                    trajectory_indexes_to_fill=[1:obj.meeting_times(2)];
                else
                    speed=dist_from_vertex_to_vertex/(obj.meeting_times(i)-obj.meeting_times(i-1)-obj.charging_period_time);
                    trajectory_indexes_to_fill=[(obj.meeting_times(i-1)+obj.charging_period_time+1):obj.meeting_times(i)];
                end
                dist_steps = speed:speed:dist_from_vertex_to_vertex;
                new_points = interp1(cumulative_dist_along_path, position_matrix, dist_steps);
                obj.trajectory_x(trajectory_indexes_to_fill)=new_points(:,1)';
                obj.trajectory_y(trajectory_indexes_to_fill)=new_points(:,2)';
                trajectory_indexes_to_fill=[trajectory_indexes_to_fill(end)+[1:obj.charging_period_time]];
                obj.trajectory_x(trajectory_indexes_to_fill)=new_points(end,1)*ones(1,obj.charging_period_time);
                obj.trajectory_y(trajectory_indexes_to_fill)=new_points(end,2)*ones(1,obj.charging_period_time);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function SetColor(obj,color)
            obj.color=color;
        end
        function plot(obj,time)
            
                plot(obj.path_x,obj.path_y,'Color',obj.color,'linewidth',3);   
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
end