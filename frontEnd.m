% frontEnd.m
clc; clear all; close all;

map = Map('map_image.bmp','resolution',100,'hieght',2000)

map.show('border')

% Count the grids
aa=map.matrix;
chromo_number=size(aa,1)*size(aa,2);

% Establish the location matrix and find the location of mission area
location_matrix=zeros(size(aa,1)*size(aa,2),2);
% location_matrix(1:size(aa,1))=1:size(aa,1);
count=1;
n=1;
for i=1:size(aa,2)
for j=1:size(aa,1)
	location_matrix(j+size(aa,1)*(i-1),:)=[i,j];
	if aa(j,i)==0
		mission_num(count)=n;

		count=count+1;
	end
	n=n+1;
end 
end

for i=1:length(mission_num)
mission_location(i,:)=location_matrix(mission_num(i),:);
end
% Set the location matrix
% Generate random order permutation of grids
chromo=randperm(chromo_number);
% Sort the chromosome
for i=1:chromo_number

chromo_location(i,:)=location_matrix(chromo(i),:);
end

% Plot the current chromosome
figure
plot(chromo_location(:,1),chromo_location(:,2),'.')
hold on 
contour(map.matrix)

% Set the speed limit
% Simplify the battery drain rate
speed_max=0.5;
speed_min=0.1;

dis_per_battery_max=speed_max*2;
dis_per_battery_min=speed_min*5;
% Calculate Euclidean between each nodes
for i= 1: chromo_number-1
nodes_dis(i)=norm(chromo_location(i,:)-chromo_location(i+1,:));
% Insert charging period in each travel
charging_number_node(i)=randi([floor(nodes_dis(i)/dis_per_battery_max),floor(nodes_dis(i)/dis_per_battery_min)]);
if i==1
	chromo_charging=chromo(1);
else
chromo_charging=cat(1,chromo_charging,[chromo(i);zeros(charging_number_node(i),1)]);
end
end
chromo_charging=[chromo_charging;chromo(end)];
% Cost function (distance)

cost_dis=sum(nodes_dis);

% Cost function (time)

% Fitness evaluation








