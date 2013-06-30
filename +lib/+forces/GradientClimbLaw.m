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

classdef GradientClimbLaw < lib.forces.ForceLaws
    properties (Access = private)
        %parameters of functions of x, that calculate the agent-leader and agent-agent force magnitude
        %a_n are the parameters of polynomial, prescribed as: y = a0*x + a1*x^2 + a3/x;
        fh_par = struct('a0',0,'a1',0,'a3',0); 
        fI_par = struct('a0',0,'a1',0,'a3',0);
        
        %toDo: move to controller
        hDist = struct('h0',10,'h1',80,'Fmax',100,'k',0.2);   %default parameters
        %Fmax is the peak value of the function
        %k is a distance of max force (F=f(x)), prescribed as Fmax = f(h0+k*(h1-h0)) 
        dDist = struct('d0',15,'d1',80,'Fmax',100,'k',0.2);
    end
    
    methods
        %Create the law  and optionally, specify the distances to leader
        %and other robots, max force power and k param 
        function obj = GradientClimbLaw(varargin)
           obj.forceName = 'gradient'; 
            if(nargin==2)
                obj.hDist = varargin{1};
                obj.dDist = varargin{2};
            end
            obj.setFI;
            obj.setFh;
        end
        
        %Calculate the resultant force driving the robot
        function F = update(obj,currRobotsArrang, state)
            F = struct('x',0,'y',0,'direction',0,'value',0);
            %Optional: landmarks dependence on the resultant force
            robots = horzcat(currRobotsArrang.aAgents);
            for i=1:1:numel(robots)
                %calc the position difference
                dx = (state.x - robots{i}.state.x);  
                dy = (state.y - robots{i}.state.y);  
                xij = norm([dx, dy]);        
                %append force
                F.x = F.x - obj.calcFI(xij)*dx/xij;
                F.y = F.y - obj.calcFI(xij)*dy/xij;
            end
            robots = currRobotsArrang.aLeaders;
            for i=1:1:numel(robots)
                dx = (state.x - robots{i}.state.x);  
                dy = (state.y - robots{i}.state.y);  
                xij = norm([dx, dy]);                
                F.x = F.x - obj.calcFh(xij)*dx/xij;
                F.y = F.y - obj.calcFh(xij)*dy/xij;
            end            
            F.direction = atan2(F.y,F.x)*180/pi;
            F.value = norm([F.x,F.y]);
            obj.value = F.value;
            obj.direction = F.direction;
            %toDo: doda� t�umienie kx' ?
        end
        
        %Sets the gradient force law agents-leaders realtion parameters
        function setFh(obj,varargin)
            if(numel(varargin)==1)
                %calc and update new force params and 
                obj.hDist.h0 = varargin{1}.h0;
                obj.hDist.h1 = varargin{1}.h1;
                obj.hDist.ymax = varargin{1}.ymax;
                obj.hDist.k = varargin{1}.k;
            end
            obj.fh_par = obj.calcNewForce(obj.hDist);
        end
        %Sets the gradient force law agents-agents realtion parameters
        function setFI(obj,varargin)
            if(numel(varargin)==1)
                %set new parameters
                obj.dDist.d0 = varargin{1}.d0;
                obj.dDist.d1 = varargin{1}.d1;
                obj.dDist.ymax = varargin{1}.ymax;
                obj.dDist.k = varargin{1}.k;
            end
            %calc and update new force params and 
            obj.fI_par = obj.calcNewForce(obj.dDist);
        end
        
        %Returns the gradient force law leader-agent relation value with
        %given position x
        function f = calcFh(obj,x)
            if(x<obj.hDist.h1)
                f = obj.fh_par.a0 + obj.fh_par.a1*x + obj.fh_par.a3/x;
            else
                f=0;
            end
        end
        
        %Returns the gradient force law agent-agent relation value
        function f = calcFI(obj,x)
            if(x<obj.dDist.d1)
                f = obj.fI_par.a0 + obj.fI_par.a1*x + obj.fI_par.a3/x;      
            else
                f=0;
            end
        end
    end
    
    methods (Access = private)        
        %Calculate the parameters of new gradient force law relation
        function f = calcNewForce(obj,Dist)
            %calculate the force polynomial
            
            %for "shorten"
            fields = fieldnames(Dist);
            Dist = struct(  'p0',Dist.(fields{1}),...
                            'p1',Dist.(fields{2}),...
                            'Fmax',Dist.Fmax,...
                            'k',Dist.k);
            syms a0 a1 a3 p0 p1
            c = (1-Dist.k)*Dist.p0 + Dist.k*Dist.p1;
            
            %find solutions
            [a0,a1,a3] = solve( a0*p0 + a1*(p0^2) + a3,...
                                a0*p1 + a1*(p1^2) + a3,...
                                a0*c + a1*c^2 + a3-Dist.Fmax*c,a0,a1,a3);
            %evaluate coefs
            na0 = subs(a0,{'p0','p1'},{Dist.p0,Dist.p1});
            na1 = subs(a1,{'p0','p1'},{Dist.p0,Dist.p1});
            na3 = subs(a3,{'p0','p1'},{Dist.p0,Dist.p1});               
            f= struct('a0',na0,'a1',na1','a3',na3);
        end
    end
    
end

