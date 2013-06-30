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

classdef Leader < lib.robots.Robot
    properties
    end
    
    methods
        %constructor
        %arg1 - handle to World
        %arg2 - initial struct={x,y,fi}
        %[arg3] - forcelaw obiect
        %[arg4] - integer modificator (for gradient and spring laws)
        %[arg5] - range of view
        function obj = Leader(parent,state,varargin)
            if(parent~='empty') %enables single robots tests
                obj.parent= parent;
                obj.controller = obj.parent.getController();
            end
            
            obj.state = state;
            obj.position.x = state.x;
            obj.position.y = state.y;
            obj.set3phase(true);
            if(numel(varargin)==0)
                obj.viewRange = 25; 
                obj.forceLaw = lib.forces.InversePower;
                obj.groups{end+1} = 'leader';
            elseif(numel(varargin)==1)
                obj.viewRange = 25;
                obj.forceLaw = varargin{1};
                obj.groups{end+1} = 'leader';
            elseif(numel(varargin)==2)
                obj.viewRange = 25;
                obj.forceLaw = varargin{1};
                obj.groups{end+1} = strcat('leader',int2str(varargin{2}));
            elseif(numel(varargin)==3)
                obj.viewRange = varargin{3};
                obj.forceLaw = varargin{1};
                obj.groups{end+1} = strcat('leader',int2str(varargin{2}));
            end 
        end
    end
end
