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

classdef InversePower < lib.forces.ForceLaws
    properties
        groupForces = {};    %cell array - holds all groups force parameters in form of forceParams structs cell array
%         forceParams;    %struct('to','','from','','c',[],'sig',[]) %definition below
    end
    methods
        %Create the inversePower object
        %arg1 - optional step the robots moves
        function obj = InversePower(varargin)
            obj.forceName = 'inverse'; 
            if(nargin==0)
                obj.step = 2;
            else
                obj.step = varargin{1};
            end
        end
        
        %Calculate the new value and direction of the resultant force
        %arg1 - currRobotsArrang - an array of handles to all other robots 
        %arg2 - state - the current state of robot
        function F = update(obj, currRobotsArrang, state)
            %F struct contains x and y components, and the force direction
            %and value
            F = struct('x',0,'y',0,'direction',0,'value',0);
            robots=horzcat(currRobotsArrang.aLeaders,currRobotsArrang.aLandmarks,currRobotsArrang.aAgents);
            %iterate through all sensed robots
            for i=1:numel(robots)
                %find the distance between 2 robots
                dx = -(state.x - robots{i}.state.x);  %-temp Xi.x + Xj.x (X-axis projection)
                dy = -(state.y - robots{i}.state.y);  %-temp Xi.y + Xj.y
                r = norm([dx, dy]);                      %temp ||Xi-Xj||
                %finding correct c & sig params
                %check what groups does the other robot belong to...
                
                for j=1:numel(robots{i}.groups)                    
                        %...and if there's a relation between robots (does the other robot impose force on the robot which 
                        %performs the calculations).
                        paramsIndex = find(cellfun(@(x)strcmp(x.from,robots{i}.groups{j}),obj.groupForces),1);
                        %if there's a relation, add it to the force
                        if(~isempty(paramsIndex))
                            %add all elements of the sum
                            for k=1:numel(obj.groupForces{paramsIndex}.c)
                                h = obj.groupForces{paramsIndex};   %just to shorten
                                F.x = F.x + (h.c(k)/(r^(h.sig(k))))*dx/r;
                                F.y = F.y + (h.c(k)/(r^(h.sig(k))))*dy/r;
%                                 if(strcmp(robots{i}.groups{j},'landmark'))
%                                     disp('landmark');
%                                 end
                            end
                        end
                end
            end
%             fprintf('F: %f %f dx %f dy %f r %f\n',F.x,F.y,dx,dy,r);
            F.direction = 180/pi*atan2(F.y,F.x);
            F.value = norm([F.x,F.y]);
            obj.direction = F.direction;
            obj.value = F.value;
            
        end
        
        %Calculate new inverse force parameters 
        %arg1 - struct('from','','to','','c',[],'sig',[])
        function addForce(obj, forceParams)
            obj.groupForces{end+1} =struct('to',forceParams.from,...
                'from',forceParams.to,...
                'c',forceParams.c,...
                'sig',forceParams.sig,...
                'd',(abs(forceParams.c(1))/abs(forceParams.c(2)))^(1/(forceParams.sig(1)-forceParams.sig(2)))); %fraction of d possible to use as step of regulation
        end
    end
end
