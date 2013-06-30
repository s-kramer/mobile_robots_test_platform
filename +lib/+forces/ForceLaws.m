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

classdef ForceLaws <handle 
    properties
        direction =0;  %direction of resultant force <0,360)
        value =1;  %the magnitute of resultant force, non zero default force required for single robot testing
        step = 2;   %the step of movement
        forceName;
    end
    methods(Access=public)
        %Sets the step of movement of the robot
        %arg1 - step integer
        function setStep(obj,step)
           obj.step = step; 
        end
        
        %Returns the step of movement
        function step = getStep(obj)
            %toDo: zwracanie wielko�ci zale�nej od d?
            step = obj.step;
        end
        
        %Get force name
        function name = getForceName(obj)
           name = obj.forceName; 
        end
    end
    
    methods (Abstract)
        update;
    end
    
end
