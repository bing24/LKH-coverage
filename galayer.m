classdef galayer < handle
    
    properties
        agent_number
        charger_number
        gene_length=20;
        fitness
        distance_working
        distance_charging
        charging_enough
        judger
        cover
        minimumFitness
        bestIndividualIndex
        iteration;
        figure_handle1;
        figure_handle2;
        count=0;
        record_dis_working;
        record_cover;
    end
    
    methods

        function optimizor=galayer(n,m)
            optimizor.agent_number=n;
            optimizor.charger_number=m;
        end
        
        function  EvaluatingAll(optimizor,sub, map, gaConfig,chargers,randIndexes)
            optimizor.count=optimizor.count+1;
            optimizor.iteration(optimizor.count)=optimizor.count;

            for nn= 1:gaConfig.NumberofChargers
                chargers(nn).locationx=zeros(sub(1).chromo_number*gaConfig.NumberofChargers,gaConfig.PopulationSize);
                chargers(nn).locationy=zeros(sub(1).chromo_number*gaConfig.NumberofChargers,gaConfig.PopulationSize);
                chargers(nn).nodes_distance=zeros(sub(1).chromo_number*gaConfig.NumberofChargers,gaConfig.PopulationSize);
                
            end
            for i=1:gaConfig.PopulationSize
                % Calculate the total distance for all working robots
                temp=[];
                for j=1:optimizor.agent_number
                    temp(j)=sub(j).cost_dis(i);
                    optimizor.distance_working(i)=sum(temp);
                end
                % Calculate the total distance for all charging robots
                
                
                
                temp=[];
                for j = 1: gaConfig.NumberofWorkers
                    lengthofchro(j)=length(nonzeros(sub(j).charging_locationx(:,i)));
                    temp=max(lengthofchro);
                end
                for nn= 1:gaConfig.NumberofChargers
                    chargers(nn).temppx=[];
                    chargers(nn).temppy=[];
                    
                end
                for j = 1: temp
                    for ii= 1: gaConfig.NumberofWorkers
                        if j <= lengthofchro(ii)
                            pick=ceil(rand*gaConfig.NumberofChargers);
                            chargers(pick).temppx=cat(1,chargers(pick).temppx,sub(ii).charging_locationx(j,i));
                            chargers(pick).temppy=cat(1,chargers(pick).temppy,sub(ii).charging_locationy(j,i));
                        end
                    end
                end
                if sum(i==randIndexes)==1 && isempty(chargers(1).best_chromo)==0
                    
                    for jj = 1:gaConfig.NumberofChargers
                      chargers(jj).locationx(:,i) = chargers(jj).best_chromo(:,1);
                      chargers(jj).locationy(:,i) = chargers(jj).best_chromo(:,2);
                    end
                    
                else
                    
                    for jj = 1:gaConfig.NumberofChargers
                      chargers(jj).locationx(1:length(chargers(jj).temppx),i) = chargers(jj).temppx;
                      chargers(jj).locationy(1:length(chargers(jj).temppy),i) = chargers(jj).temppy;
                    end
                end
                    
                temp_dis=[];
                for jj= 1: gaConfig.NumberofChargers

                    MeetingNumbers = length(nonzeros(chargers(jj).locationx(:,i)));
                    for gg = 1:MeetingNumbers-1
                        chargers(jj).nodes_distance(gg,i)=norm([nonzeros(chargers(jj).locationx(gg,i)),nonzeros(chargers(jj).locationy(gg,i))]-[nonzeros(chargers(jj).locationx(gg+1,i)),nonzeros(chargers(jj).locationy(gg+1,i))]);
                    end
                    chargers(jj).total_distance(i)=sum(chargers(jj).nodes_distance(:,i));
                    temp_dis(jj)=chargers(jj).total_distance(i);
                end
                
                
                optimizor.distance_charging(i)=sum(temp_dis);
                
                
                
                % Calculate the coverage of all working robots
                count=0;
                
                for ii=1:size(map.mission_num,1)
                    cc=[];
                    for jj=1:optimizor.agent_number
                        cc=[cc; map.mission_num(ii)~=sub(jj).chromo_index(:,i)];
                    end
                    if sum(cc)~=optimizor.gene_length*optimizor.agent_number
                        count=count+1;
                    end
                end
                optimizor.cover(i)=count/size(map.mission_num,1);

                % adding penalty to low coverage
                if optimizor.cover(i)<0.9
                    optimizor.cover(i)=optimizor.cover(i)*0.001;
                end

                 % calculate charging time and operating time
                for j=1:optimizor.agent_number
                    
                    
                count=1;
                ii=1;
                gg=1;
                temp=[];
                tempp=[];
                while count < sub(j).gene_length
                    
                    if sub(j).chromo(count,i) == 0
                        cc=1;
                        
                        while sub(j).chromo(count,i)==0 & count < sub(j).gene_length
                            temp(ii)=cc;
                            cc=cc+1;
                            count=count+1;
                            
                        end
                        ii=ii+1;
                    else
                        dd=1;
                        while sub(j).chromo(count,i)~=0 & count < sub(j).gene_length
                            tempp(gg)=dd;
                            dd=dd+1;
                            count=count+1;
                        end
                        gg=gg+1;
                    end
                end
                
                if length(temp)>length (tempp)
                    temp=temp(1:end-1);
                elseif length(temp)<length(tempp)
                    tempp=tempp(1:end-1);
                end
                
                if isempty(tempp)
                    sub(j).charging_enough(1,i)=0.001; % adding penalty to zero charging trajectory
                else
                    sub(j).charging_enough(1,i)=sum(temp>tempp)/length(temp);
                end

                charging_enough_temp(j)=sub(j).charging_enough(1,i);
                end
                % Consider the maximum speed constraint
                for j = 1:gaConfig.NumberofChargers
                if sum(chargers(j).nodes_distance(:,i)>10)==0
                    chargers(j).judger(i)=1;
                else
                    chargers(j).judger(i)=100;
                end
                judger_temp(j)=chargers(j).judger(i);                
                end

                optimizor.charging_enough(i)=sum(charging_enough_temp)/optimizor.agent_number;
                optimizor.judger(i)=sum(judger_temp)/gaConfig.NumberofChargers;
                % optimizor.fitness(i)=(optimizor.distance_working(i)+optimizor.distance_charging(i))/optimizor.cover(i)/optimizor.charging_enough(i);
                
                optimizor.fitness(i)=(optimizor.distance_working(i)+optimizor.distance_charging(i))/optimizor.cover(i)/optimizor.charging_enough(i)*optimizor.judger(i);
            end
            [optimizor.minimumFitness, optimizor.bestIndividualIndex] = min(optimizor.fitness);
            fprintf('Minimum Fitness: %d\n',optimizor.minimumFitness);
            fprintf('Minimum Fitness index: %d\n',optimizor.bestIndividualIndex);
            for i=1:gaConfig.NumberofChargers
                chargers(i).best_chromo=[chargers(i).locationx(:,optimizor.bestIndividualIndex) chargers(i).locationy(:,optimizor.bestIndividualIndex)];
            end
            
        end % function
        function EvaluatingCover(optimizor,sub,map,gaConfig)
            optimizor.count=optimizor.count+1;
            optimizor.iteration(optimizor.count)=optimizor.count;
            for i=1:gaConfig.PopulationSize
                % Calculate the total distance for all working robots
                temp=[];
                for j=1:optimizor.agent_number
                    temp(j)=sub(j).cost_dis(i);
                    optimizor.distance_working(i)=sum(temp);
                end
                           
                % Calculate the coverage of all working robots
                count=0;
                
                for ii=1:size(map.mission_num,1)
                    cc=[];
                    for jj=1:optimizor.agent_number
                        cc=[cc; map.mission_num(ii)~=sub(jj).chromo_index(:,i)];
                    end
                    if sum(cc)~=optimizor.gene_length*optimizor.agent_number
                        count=count+1;
                    end
                end
                optimizor.cover(i)=count/size(map.mission_num,1);
                
                


                % optimizor.fitness(i)=(optimizor.distance_working(i)+optimizor.distance_charging(i))/optimizor.cover(i)/optimizor.charging_enough(i);
                
                % optimizor.fitness(i)=optimizor.distance_working(i)/optimizor.cover(i);
                optimizor.fitness(i)=1/optimizor.cover(i);
                if optimizor.cover(i) < 1
                    optimizor.fitness(i)=optimizor.fitness(i)*10;
                end
            end
            [optimizor.minimumFitness, optimizor.bestIndividualIndex] = min(optimizor.fitness);
            fprintf('Minimum Fitness: %d\n',optimizor.minimumFitness);
            fprintf('Minimum Fitness index: %d\n',optimizor.bestIndividualIndex);
            optimizor.record_cover(optimizor.count)=optimizor.cover(optimizor.bestIndividualIndex);
            optimizor.record_dis_working(optimizor.count)=optimizor.distance_working(optimizor.bestIndividualIndex); 
        end
         function EvaluatingWorking(optimizor,sub,map,gaConfig)
            optimizor.count=optimizor.count+1;
            optimizor.iteration(optimizor.count)=optimizor.count;
            for i=1:gaConfig.PopulationSize
                % Calculate the total distance for all working robots
                temp=[];
                for j=1:optimizor.agent_number
                    temp(j)=sub(j).cost_dis(i);
                    optimizor.distance_working(i)=sum(temp);
                end
                           
                % Calculate the coverage of all working robots
                count=0;
                
                for ii=1:size(map.mission_num,1)
                    cc=[];
                    for jj=1:optimizor.agent_number
                        cc=[cc; map.mission_num(ii)~=sub(jj).chromo_index(:,i)];
                    end
                    if sum(cc)~=optimizor.gene_length*optimizor.agent_number
                        count=count+1;
                    end
                end
                optimizor.cover(i)=count/size(map.mission_num,1);
                
                


                optimizor.fitness(i)=optimizor.distance_working(i)/optimizor.cover(i);
                
                
                
                if optimizor.cover(i) < 1
                    optimizor.fitness(i)=optimizor.fitness(i)*10;
                end
            end
            [optimizor.minimumFitness, optimizor.bestIndividualIndex] = min(optimizor.fitness);
            fprintf('Minimum Fitness: %d\n',optimizor.minimumFitness);
            fprintf('Minimum Fitness index: %d\n',optimizor.bestIndividualIndex);
            optimizor.record_cover(optimizor.count)=optimizor.cover(optimizor.bestIndividualIndex);
            optimizor.record_dis_working(optimizor.count)=optimizor.distance_working(optimizor.bestIndividualIndex); 
        end
        
        function chargingEvaluate(a,chargers,obj,gaConfig)
            
            % Charging robots locations
            
            for ii=1:gaConfig.PopulationSize
                for j=1:gaConfig.NumberofWorkers
                    for i=1:length(obj(j).charging_locationx)
                        chargers(ceil(rand*gaConfig.NumberofChargers)).locationx(j,ii) = obj(i).charging_locationx(j,ii);
                        chargers(ceil(rand*gaConfig.NumberofChargers)).locationy(j,ii) = obj(i).charging_locationx(j,ii);
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
        function gaMutate(obj)
            
        end
        function plotall(obj,workers,chargers,gaConfig)
            figure (3)
            for agent_number = 1:gaConfig.NumberofWorkers
                hold on
                Ploting(workers(agent_number))
                
            end
            for charger_number = 1:gaConfig.NumberofChargers
                Ploting(chargers(charger_number),obj)
            end
            
        end
        
        function plotWorking(obj,workers,gaConfig)
            figure(3)
            for agent_number = 1:gaConfig.NumberofWorkers
                hold on
                Ploting(workers(agent_number))
            end
            title(obj.count) 
            figure(2)
            
            if ishandle(obj.figure_handle1)
                delete(obj.figure_handle1)
            end
            subplot(2,1,1)
            obj.figure_handle1=semilogx(obj.iteration,obj.record_cover);
            xlabel('iteration')
            ylabel('Coverage')
            if ishandle(obj.figure_handle2)
                delete(obj.figure_handle2)
            end
            subplot(2,1,2)
            obj.figure_handle2=semilogx(obj.iteration,obj.record_dis_working);
            xlabel('iteration')
            ylabel('Total traveling distance of working robots')
            
            
        end

    end % method
    
end % classdef