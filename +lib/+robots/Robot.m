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

classdef Robot < handle
    %ROBOT Robot superclass
    %   Robot superclass

    properties (Access = public)
        groups={};  %robots groups
    end
    
    properties (Access = protected)	
        %robot info
        position=struct('x',0,'y',0);   %robot position x,y only
        state = struct('x',0,'y',0,'fi',0);    %state vector: posx, posy, fi angle
        desState = struct('x',0,'y',0,'fi',0);  %currently desired state
        potential = 0;  %currently measured scalar field potential (the potential from landmarks, used in gradient climbing law)
        
        %linear speed components
        speed = struct('x',0,'y',0);
        
        forceLaw;   %force law type class
        currForce;  %actual force exerted from other robots struct ('x',0,'y',0,'direction',0,'value',0)
        viewRange = 0;  %robot range of view (radius) [pixel] NOT IMPLEMENTED
        
        %world-connected parameters
        parent; % the parent of the robot (World) handle
        controller; %global controler handle
        currRobotsArrang; %=struct('aLandmarks',{struct('groups',{},'state',[])},...
                           %     'aAgents',{struct('groups',{},'state',[])},...
                           %     'aLeaders',{struct('groups',{},'state',[])});    
                           %struct of all (sensed) robots states to get actual arrangement of  the robots
        
        %PID regulators - angle and distance norm regulation
        angRegulator = lib.robots.PID(230,0.003,0.001,20);
        posRegulator = lib.robots.PID(230,0.003,0.001,0.1);
        
        %3-phase regulator related only
        regPhases = struct('p1',false,'p2',false,'p3',false);
        reg3phase = false;
        
        %robot parameters
        Tp = 1e-3;  %internal sampling time Tp = 1ms 
    end
    
    methods (Access = public)
        
        %Set the type of regulation to 3 phase or 1 phase (desc @
        %steerToDestState method)
        function set3phase(obj,set)
            obj.reg3phase = set;
        end
        
        %Get functions
        function pos=getPosition(obj)
            pos=obj.position;
        end
        
        function act = getState(obj)
            act = obj.state;
        end
        
        function des = getDesState(obj)
            des = obj.desState;
        end
        
        function rng = getRange(obj)
            rng = obj.viewRange;
        end
        
        function name = getForceName(obj)
            name = obj.forceLaw.getForceName();
        end
        
        %set functions
        function changeDesState(obj,state)
            obj.desState = state;
        end
        
        %Calculate the eukledian distance to another robot
        function dist = getDistToRobot(obj, state)
            dist = struct('d',0,'dfi',0);
            dist.d = norm([obj.getState.x - state.x, obj.getState.y - state.y]);
            dist.dfi = obj.state.fi - state.fi;
        end
        
        %Move the robot towards desired position
        function steerToDestState(obj)
            loc2TargetOrient = 180/pi*atan2(obj.desState.y - obj.state.y,obj.desState.x - obj.state.x); %the relative orientation to the desired target
            eang = loc2TargetOrient - obj.state.fi; %orientation error
            if(eang<-180)
                eang=eang+360;
            elseif(eang>180)
                eang=eang-360;
            end
            epos = norm([obj.desState.x-obj.state.x,obj.desState.y-obj.state.y]);   %position error
            if(obj.forceLaw.value~=0)   %required for capability
                if(~obj.reg3phase)
                    %METHOD 1 (continous regulation, 2 available linear speeds)
                    %regulate the linear and angular speed simultaneously
                    v = [10,100];   %arbitrally chosen values
                    if(norm([obj.desState.x-obj.state.x, obj.desState.y-obj.state.y]) > obj.posRegulator.getTolError())
                        obj.nextState(struct('v',v(1+(abs(eang)<obj.angRegulator.getTolError())),'w',obj.angRegulator.genU(eang))); %v choosen depending on the orientation error
                        %v choosed in "python style"
                    end
                else
                    %METHOD 2: 3 phase
                    %phase 1 - get the orientation towards destination point
                    if((obj.regPhases.p1==false) && (abs(eang) >=obj.angRegulator.getTolError()) && (obj.desState.y ~= obj.state.y) && (obj.desState.x ~= obj.state.x))
                        obj.nextState(struct('v',0,'w',obj.angRegulator.genU(eang)));
    %                     disp('Phase 1');
                    %phase 2 - orientation correct, drive to dest point
                    elseif((obj.regPhases.p2==false) && (norm([obj.desState.y-obj.state.y, obj.desState.x-obj.state.x]) > obj.posRegulator.getTolError()))
                        obj.regPhases.p1=true;
                        obj.nextState(struct('v',obj.posRegulator.genU(epos),'w',obj.angRegulator.genU(eang)));
    %                     disp('Phase 2');
                    %phase 3 - dest point achived, set dest orientation
                    elseif(obj.regPhases.p3==false)
                        obj.regPhases.p2=true;
                        eang = obj.desState.fi - obj.state.fi;
                        obj.nextState(struct('v',0,'w',obj.angRegulator.genU(eang)));
                    end
                end
            end
        end
        
        %set robots range of view
        %arg1 - range of view
        function setRange(obj,range)
            obj.viewRange = range;
        end
        
        %Make the measurement of the field potential at robot's current
        %location
        function pot = getFieldPotential(obj)
            %measure the field potential (sum of 1/r_ij)
            pot = 0;
            
            %update the info about other robots
            obj.makeMeasurement();
            for i=1:numel(obj.currRobotsArrang.aLandmarks)
                pot = pot + 20/obj.getDistToRobot(obj.currRobotsArrang.aLandmarks{i}.state).d;
            end
            obj.potential = pot;
        end
        
        %calculate the next desired state
        function update(obj)
            obj.makeMeasurement(); %get other robots positions
            obj.currForce = obj.forceLaw.update(obj.currRobotsArrang, obj.state); %calc the force which causes the movement
            fi_d = obj.currForce.direction;
            
            obj.resetRegulators();  %reset PID's
            obj.changeDesState(struct('x',obj.getState.x+ obj.forceLaw.getStep()*cosd(fi_d),...
            'y',obj.getState.y + obj.forceLaw.getStep()*sind(fi_d),...
            'fi',fi_d));    %set new destination point and orientation            
        end
        
        %update the information about other robots arrangement
        function makeMeasurement(obj)
            %get all robots states from world
            obj.currRobotsArrang.aAgents = cellfun(@(x) {struct('state',x.getState,'groups',{x.groups})},obj.parent.aAgents);
            obj.currRobotsArrang.aLandmarks = cellfun(@(x) {struct('state',x.getState,'groups',{x.groups})},obj.parent.aLandmarks);
            obj.currRobotsArrang.aLeaders = cellfun(@(x) {struct('state',x.getState,'groups',{x.groups})},obj.parent.aLeaders);
            %remove the current robot from downloaded robots arrangement 
            if isa(obj,'lib.robots.Agent')==1
                    i = find(cellfun(@(z)...
                    (z.state.x == obj.state.x) && ...
                    (z.state.y == obj.state.y) && ...
                    (z.state.fi == obj.state.fi),obj.currRobotsArrang.aAgents),1);
                    obj.currRobotsArrang.aAgents(i) = [];
            elseif isa(obj,'lib.robots.Landmark')==1
                    i = find(cellfun(@(z)...
                    (z.state.x == obj.state.x) && ...
                    (z.state.y == obj.state.y) && ...
                    (z.state.fi == obj.state.fi),obj.currRobotsArrang.aLandmarks),1);
                    obj.currRobotsArrang.aLandmarks(i) = [];
            else
%                 for i=1:numel(obj.currRobotsArrang.aLeaders)
%                     state = obj.currRobotsArrang.aLeaders(i).getState;
%                     if (state.x == obj.state.x) && (state.y == obj.state.y) && (state.fi == obj.state.fi)
%                         obj.currRobotsArrang.aLeaders(i) = [];
%                         break
%                     end
%                 end                
                    i = find(cellfun(@(z)...
                    (z.state.x == obj.state.x) && ...
                    (z.state.y == obj.state.y) && ...
                    (z.state.fi == obj.state.fi),obj.currRobotsArrang.aLeaders),1);
                    obj.currRobotsArrang.aLeaders(i) = [];
            end        
        end
        
        %Add inverse force
        function addForce(obj,force)
            obj.forceLaw.addForce(force);
        end
        
        %Add spring force
        function addSpringForce(obj,force)
            obj.forceLaw.addSpringForce(force);
        end        
    end
    methods (Access = private)
        %Calculate robots new state from robots model dynamics 
        function next = nextState(obj, u)
            next.x = obj.Tp * u.v * cosd(obj.state.fi) + obj.state.x;
            next.y = obj.Tp * u.v * sind(obj.state.fi) + obj.state.y;
            next.fi = u.w*obj.Tp + obj.state.fi;
            obj.speed.x = next.x;
            obj.speed.y = next.y;
        
            %update robot info
            obj.setPosition(next.x, next.y);
            obj.state = next;
        end
        
        %Reset the regulators and regulation parameters
        function resetRegulators(obj)            
            obj.posRegulator.reset();
            obj.angRegulator.reset();
            obj.regPhases = struct('p1',false,'p2',false,'p3',false);            
        end
        
        %Set the new position calculated as a result of resultant force and
        %regulator calculations (actual movement done here)
        %arg - 1,2 - new X and Y coordinates
        function setPosition(obj, x,y)
            obj.position.x = x;
            obj.position.y = y;
        end


    end
end


