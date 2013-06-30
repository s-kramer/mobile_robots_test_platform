%Copyright 2013 Sebastian Kramer <sebastian.kramer@wp.pl> 
%This file is part of "Matlab mobile robots test platform r1.0" 
%
%"Matlab mobile robots test platform" is free software: you can redistribute it and/or modify
%it under the terms of the GNU General Public License as published by
%the Free Software Foundation, either version 3 of the License, or
%(at your option) any later version.
%
%"Matlab mobile robots test platform r1.0" is distributed in the hope that it will be useful,
%but WITHOUT ANY WARRANTY; without even the implied warranty of
%MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%GNU General Public License for more details.
%
%You should have received a copy of the GNU General Public License
%along with "Matlab mobile robots test platform r1.0".  If not, see <http://www.gnu.org/licenses/>.

classdef Controller < handle
    
    properties
        %toDo: private parametes
        pointGoal = struct('x',0,'y',0);
		
        %states of robots:
		statLeaders={};
        statAgents={};
        statLandmarks={};

        %handle to parent (World)
        parent;

        %all groups available in the world names
        groups ={};        
        
        %all INVERSE forces between the groups, elements of the cell array are force structs defined below
        forces = {};
        %single inverse force between 2 groups parameters struct prototype
        %force = struct('from','','to','','c',[],'sig',[]);
        
        %all SPRING forces between the groups, elements of the cell array are force structs defined below
        springForces = {}
        %single spring force between 2 groups parameters struct prototype
        %force = struct('from','','to','','k',[],'l',[]);
        
        %%%% GRADIENG CLIMBING %%%% 
        %measured field potential vector
        potentials=[];
        
        %acutal least square approximation of the gradient of the field
        xLS = [];
    end
    
    methods
        %Creates Controller obiect. Takes the world obiect as argument
        %arg1 - World obiect
        function obj=Controller(world)
            %define force laws between robots
            obj.groups = {'agent','landmark','leader'};
           
            %INVERSE FORCES
            %NOTE: no empty spaces or comment lines allowed between structs!!!!
            % -c1 - repulsion force magnitude
            %  c2 - attaction force magnitude
            %sig1 - big causes the robot to repulse rapidly on short
            %distances
            %sig2 - small increases the range of attraction
            %relation "from agent to landmark" means, that those parameters will be used to control agents control
            %force when compared with landmark, i.e. if landmark is steady
            %this will controll agents to it. 
            obj.forces={        struct('to','leader','from','leader','c',[-20 1],'sig',[2 1]),...
                                struct('to','leader','from','agent','c',[-20 1],'sig',[2 1]),...
                                struct('to','leader','from','landmark','c',[-20 1],'sig',[2 1]),...
                                struct('to','agent','from','agent','c',[-20 1],'sig',[2 1]),...
                                struct('to','agent','from','leader','c',[-20 1],'sig',[2 1]),...
                                struct('to','agent','from','landmark','c',[-20 1],'sig',[2 1]),...
                                struct('to','landmark','from','landmark','c',[-20 1],'sig',[2 1]),...
                                struct('to','landmark','from','agent','c',[-80 1],'sig',[2 0.1]),...
                                struct('to','landmark','from','leader','c',[-20 1],'sig',[2 1])};
                            
            %SPRING FORCES
            %NOTE: no empty spaces or comment lines allowed between structs!!!!
            obj.springForces={  struct('to',    'agent1',   'from','agent2','k',5,'l',15),...
                                struct('to',    'agent1',   'from','agent3','k',5,'l',15),...
                                struct('to',    'leader1',  'from','agent1','k',15,'l',10),...
                                struct('to',    'leader1',  'from','agent3','k',15,'l',20)};
%                                 struct('to',    'agent2',   'from','agent3','k',5,'l',40),...
%                                 struct('to','leader1','from','leader2','k',5,'l',5)...
%                                 struct('to','leader2','from','leader3','k',5,'l',37)...
%                                 struct('to','landmark1','from','landmark2','k',5,'l',10)...
%                                 struct('to','landmark1','from','landmark3','k',5,'l',10)...
%                                 struct('to','landmark1','from','landmark4','k',5,'l',10)...
%                                 struct('to','landmark2','from','landmark3','k',5,'l',10)...
%                                 struct('to','landmark3','from','landmark4','k',5,'l',10)...
%                                 struct('to','landmark2','from','landmark4','k',5,'l',10)...
            obj.parent = world;
        end
        
        %Update the informations for virtual body calculations - required
        %by gradient climbing method. 
        function update(obj)
            obj.getMeasurements();
            obj.calculateVirtualBody();
            obj.sendVirtualBody();
        end
        
        %Set formation goal position (not used really)
        function setGoal(obj, x,y)
           obj.pointGoal=[x,y];
        end
        
        %Add another group to the controller (only if the group is not
        %already present)
        function addGroup(obj, group)
            for i=1:numel(group)
                %verify, if the group has not already been added 
                index = find(cellfun(@(x) strcmp(group{i},x),obj.groups),1);
                if(isempty(index))
                    obj.groups{end+1} = group;
                end
            end
        end
        
        %Add another INVERSE force to the controller (only if the relation
        %between robots is not already defined)
        %toDo: if the relation is already present - modify it.
        function addForce(obj,force)
            for i=1:numel(force)
                %verify, if the relation between groups has not already been added 
                index = find(cellfun(@(x) strcmp(force{i}.from,x.from) && strcmp(force{i}.to,x.to),obj.forces),1);
                if(isempty(index))
                    obj.forces{end+1} = force;
                end
            end
        end
        
        %Add another SPRING force to the controller (only if the relation
        %between robots is not already defined)
        %toDo: if the relation is already present - modify it.
        function addSpringForce(obj,force)
            for i=1:numel(force)
                %verify, if the relation between groups has not already been added 
                index = find(cellfun(@(x) strcmp(force{i}.from,x.from) && strcmp(force{i}.to,x.to),obj.springForces),1);
                if(isempty(index))
                    obj.springForces{end+1} = force;
                end
            end
        end
    end
    
    methods (Access=private)
        %Make the leaders and agents perform field potential measurements
        %and send them to the controller to calc the gradient steepest descent
        function pot = getMeasurements(obj)
            pot = cellfun(@(x)x.getFieldPotential(),horzcat(obj.parent.aLeaders,obj.parent.aAgents))';
            obj.potentials = pot;
        end
        
        %Calculate the new virtual body gradient and scalar field value
        %least square estimate
        function xLS = calculateVirtualBody(obj)
            C = cell2mat(cellfun(@(z) [z.getState.x z.getState.y 1],horzcat(obj.parent.aLeaders,obj.parent.aAgents),'UniformOutput',false)');
            xLS = (C'*C)^-1*C'*obj.potentials;
            %toDo: How to move structure? How to interpret s parameter? 
        end
        
        %Send new Virtual Body structure to the robots
        function sendVirtualBody(obj)
        
        end
    end 
    
end
