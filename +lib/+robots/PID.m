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

classdef PID
    %REGULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    
    properties (Access = private)
        params = struct('kp',0,'ki',0,'kd',0,'Tf',0);
        objectParams = struct('k',0,'T',0,'T0',0);
        
        
        e=0;      %current deviation
        ePrev=0;  %previous deviation
        eSum=0;   %sum of previous deviation
        udPrev=0; %previous steering od D part
        uiSum=0;  %sum of previous integral steering part
        
        toleratedError; %error that will limit the next regulation phase trigger
        
        Tp= 1e-3;   %samping time - for calculations only
    end
    
    methods
        %create PID instance given parameters of a first order inertial element
        %arg1 - pid amplification
        %arg2 - time constant
        %arg3 - delay time
        %arg4 - tollerated error - the error below which the regulation will not proceed
        function obj = PID(ko,T,T0,error)
           obj.objectParams.k = ko;
           obj.objectParams.T = T;
           obj.objectParams.T0 = T0;
           
           obj.params.kp = 1.2*T/(ko*T0);
           obj.params.ki = 1/(2*T0);
           obj.params.kd = T0/2;
           obj.params.Tf = T0/5;           
           
            obj.toleratedError = error;
        end
        
        %Set tolleration error value
        function setTolError(obj, error)
            obj.toleratedError = error;
        end
        
        %Get tolleration error value
        function error = getTolError(obj)
            error = obj.toleratedError;
        end            
        
        %Generate control signal
        function u = genU(obj,e)
            obj.ePrev = obj.e;
            obj.e = e;
            obj.eSum = obj.eSum + obj.e;
            
            up = obj.params.kp*e;
            ui = obj.params.ki*obj.eSum;
            ud = obj.params.kp*(2*obj.params.kd*(obj.e - obj.ePrev)-(obj.Tp-2*obj.params.Tf)*obj.udPrev)/(obj.Tp+2*obj.params.Tf);
            obj.udPrev =ud;
            u = up+ui+ud;
        end
        
        %Reset the regulator internal parameters
        function reset(obj)
            e=0;      
            ePrev=0;  
            eSum=0;   
            udPrev=0; 
            uiSum=0;  
        end
    end
    
end
