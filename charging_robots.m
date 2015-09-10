classdef charging_robots < handle

properties
        temppx
        temppy
		ID
        color
        cover
        charging_index
        locationx
        locationy
        nodes_distance
        total_distance
        judger
        figure_handle
        best_chromo
    end
    
    methods
        
        function chargers = charging_robots(color, ID)
        	chargers.color=color;
        	chargers.ID=ID;
        end

        function chargingEvaluate(chargers,obj,gaConfig)
            
            % Charging robots locations
            for ii=1:gaConfig.PopulationSize   
                for j=1:obj(1).gene_length                    
                    for i=1:obj(1).agent_number
                       chargers(ceil(rand*gaConfig.NumberofChargers)).locationx     
                        
                    end
                end
            end
            obj.nodes_dis_charging=zeros(size(obj.charging_locationx,1)-1,gaConfig.PopulationSize);
            obj.cost_dis=[];
            % Calculate Euclidean between each nodes
            for j= 1: gaConfig.PopulationSize
                for i= 1: obj.chromo_number
                    if isempty(find(map.location_matrix(:,1)==obj.currentx(i,j) & map.location_matrix(:,2)==obj.currenty(i,j)))
                        obj.chromo_index(i,j)=0;
                    else
                        obj.chromo_index(i,j)=find(map.location_matrix(:,1)==obj.currentx(i,j) & map.location_matrix(:,2)==obj.currenty(i,j));
                    end
                    if obj.chromo(i,j)==0
                        obj.charging_index(i,j)=obj.chromo_index(i,j);
                    end
                end
                % chromo_charging=[chromo_charging;obj.chromo(end)];
                % Cost function (distance)
                
                obj.cost_dis(j)=length(nonzeros(obj.chromo(:,j)));
                
                % Calculate the travel distance of charging robots
                
                for i= 1: size(obj.charging_locationx,1)-1
                    if obj.charging_locationx(i+1,j)~= 0
                        obj.nodes_dis_charging(i,j)=norm([obj.charging_locationx(i,j),obj.charging_locationy(i,j)]-[obj.charging_locationx(i+1,j),obj.charging_locationy(i+1,j)]);
                    end

                end
            end
        end
        
        function Ploting(obj,optimizor)
            if ishandle(obj.figure_handle)
                delete(obj.figure_handle)
            end
            chargingx=nonzeros(obj.locationx(:,optimizor.bestIndividualIndex));
            chargingy=nonzeros(obj.locationy(:,optimizor.bestIndividualIndex));
            obj.figure_handle=plot(chargingx,chargingy,'-.k');
            title(optimizor.minimumFitness)
            
        end %function
        function preserving_chargers(obj,randIndexes)
            bestChromosome =obj.chromo(:,obj.bestIndividualIndex);
            obj.locationx(:,randIndexes) = repmat(bestChromosome,1,gaConfig.numberOfReplications);
            obj.locationy(:,randIndexes) = repmat(bestChromosome,1,gaConfig.numberOfReplications);
            
            
        end
    end %methods
end %classdef