classdef Evaluator < handle
properties
  fitness
  end

  methods

  function fit= Evaluating (obj)
% Calculate Euclidean between each nodes
for i= 1: obj.chromo_number-1
nodes_dis(i)=norm([obj.chromo_locationx(i),obj.chromo_locationy(i)]-[obj.chromo_locationx(i+1),obj.chromo_locationy(i+1)]);
% Insert charging period in each travel
% charging_number_node(i)=randi([floor(nodes_dis(i)/dis_per_battery_max),floor(nodes_dis(i)/dis_per_battery_min)]);
% if i==1
%     chromo_charging=obj.chromo(1);
% else
% chromo_charging=cat(1,chromo_charging,[obj.chromo(i);zeros(charging_number_node(i),1)]);
% end
end
% chromo_charging=[chromo_charging;obj.chromo(end)];
% Cost function (distance)

cost_dis=sum(nodes_dis);

fitness=cost_dis;
  end
 end % end methods
end % classdef

