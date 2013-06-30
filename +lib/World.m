%Copyright 2013 Sebastian Kramer <sebastian.kramer@wp.pl> 
%This file is part of "Matlab mobile robots test platformi r1.0" 
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


classdef World < handle
    properties
               
        %tablice robot�w
		aAgents={};
        aLandmarks={};
		aLeaders={};
		
        %handle to controller
        controller;
                
        %previous positions of robots
        posSamples = struct('xAg',[],'yAg',[],'xLe',[],'yLe',[],'xLm',[],'yLm',[]);
    end
    
    properties (Access = private)
        %map size
        axisX;  
        axisY;
        
        %plot
		fig = plot(1:10,1:10,'bo',1:10,1:10,'r+',1:10,1:10,'k*');
    end
    methods (Access  = public)
        %Creates World obiect, which is essential for the rest of the %simulation
        %arg1 - size of world 
        function obj = World(varargin)            
            %default parameters
            if(numel(varargin)==0)
                obj.axisX=100;
                obj.axisY=100;
            elseif(numel(varargin)==1)
                obj.axisX=varargin{1};
                obj.axisY=varargin{1};
            end
            set(obj.fig,'markersize',4);
            axis([-20 obj.axisX+20 -20 obj.axisY+20]);
            obj.controller = lib.Controller(obj);
        end
        
        %Adds robots to world obiect. The robot must be already created.
        %Robots desired force law name must also be given
        %arg1 - Robot obiect
        function addRobots(obj,robot)
            for i=1:numel(robot)
                if isa(robot{i},'lib.robots.Agent')
                    obj.aAgents{end+1}=robot{i};
                elseif isa(robot{i},'lib.robots.Landmark')
                    obj.aLandmarks{end+1}=robot{i};
                else
                    obj.aLeaders{end+1}=robot{i};
                end            
                
                if(strcmp(robot{i}.getForceName(),'inverse'))
                    %add inverse power force parameters to the robot
                    for k=1:numel(robot{i}.groups)
                        for j=1:numel(obj.controller.forces)
                            if(strcmp(obj.controller.forces{j}.from,robot{i}.groups{k})==1)
                                robot{i}.addForce(obj.controller.forces{j})
                            end
                        end
                    end
                elseif(strcmp(robot{i}.getForceName(),'spring'))
                    %add spring force parameters to the robot
                    for k=1:numel(robot{i}.groups)
                        for j=1:numel(obj.controller.springForces)
                            if((strcmp(obj.controller.springForces{j}.from,robot{i}.groups{k})==1) || (strcmp(obj.controller.springForces{j}.to,robot{i}.groups{k})==1))
                                robot{i}.addSpringForce(obj.controller.springForces{j})
                            end
                        end
                    end
%                 else
                 %toDo: obliczanie w kontrolerze zamiast w klasie gradient
%                     for k=1:numel(robot{i}.groups)
%                         for j=1:numel(obj.controller.gradForces)
%                             if(strcmp(obj.controller.gradForces{j}.from,robot{i}.groups{k})==1)
%                                 robot{i}.addForce(obj.controller.gradForces{j})
%                             end
%                         end
%                     end
                end
            end
            obj.WindowUpdate();
        end
        
        %Make controller force a field potential measurement
        function controllerUpdate(obj)
            obj.controller.update();
        end
        
        %Update the world state. If gradient climbing law is used, compute
        %the virtual body and send it back to the robots (VB movement and sending not implemented).
        %Refresh the output window
        function update(obj)
% 				obj.controller.computeVirtualBody();
% 				SendResultToRobots();
% 				obj.controller.sendToLeaders(obj.aLeaders);
				obj.WindowUpdate();
% 				ComputeNext();
        end
        
        %Returns handle to the global controller
        function ctrl = getController(obj)
            ctrl = obj.controller;
        end            
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    methods(Access = private)
        
        %Refresh the output window 
        %toDo: change for loops to cellfun 
        function WindowUpdate(obj)
            %Clear the previous samples
            %Forsing false will allow to display the history of robots movement
            if(true)
                obj.posSamples.xAg = [];
                obj.posSamples.yAg = [];
                obj.posSamples.xLe = [];
                obj.posSamples.yLe = [];
                obj.posSamples.xLm = [];
                obj.posSamples.yLm = [];
            end
            %get states
            for it=1:length(obj.aAgents),
                obj.posSamples.xAg(end+1) = obj.aAgents{it}.getState.x;
                obj.posSamples.yAg(end+1) = obj.aAgents{it}.getState.y;
            end
            for it=1:length(obj.aLandmarks),
				obj.posSamples.xLm(end+1) = obj.aLandmarks{it}.getState.x;
                obj.posSamples.yLm(end+1) = obj.aLandmarks{it}.getState.y;
            end
            for it=1:length(obj.aLeaders),
				obj.posSamples.xLe(end+1) = obj.aLeaders{it}.getState.x;
                obj.posSamples.yLe(end+1) = obj.aLeaders{it}.getState.y;
            end
            
            %update the fig data
            tempx={};
            tempy={};
            tempx{1,1} = obj.posSamples.xAg;
            tempy{1,1} = obj.posSamples.yAg;
            tempx{2,1} = obj.posSamples.xLm;
            tempy{2,1} = obj.posSamples.yLm;
            tempx{3,1} = obj.posSamples.xLe;
            tempy{3,1} = obj.posSamples.yLe;
            set(obj.fig,{'Xdata'},tempx,{'Ydata'},tempy);
        end
    end   
end
