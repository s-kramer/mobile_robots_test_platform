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

classdef Circle < lib.forces.ForceLaws
    
    properties
    end
    
    methods
        function obj = Circle()
           obj.forceName = 'Circle'; 
        end
        %Makes the robot move circular and right
        %varargin required for compatibility
        function F = update(obj,varargin)
            F = struct('direction',0,'value',0);
            obj.direction=obj.direction+10;
            obj.direction = mod(obj.direction,360);            
            obj.value = 1;
            
            F.value = obj.value;
            F.direction =obj.direction;
        end
    end
    
end

