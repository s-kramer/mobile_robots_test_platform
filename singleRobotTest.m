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


%simulation only
h = plot(1:1:10,1:1:10);
set(h,'markersize',4);

xPrev = [];
yPrev = [];
fi = [];

%state vector
startState = struct('x',60,'y',20,'fi',50);

%robot definition
%landmark - one phase regulation - straight line
%leader and agent - 3 phase regulation - curve line
%robot = lib.robots.Agent('empty',startState); %'empty' instread of world handle!
robot = lib.robots.Landmark('empty',startState); %'empty' instread of world handle!
robot.changeDesState(struct('x',0,'y',0,'fi',250));

%plot
for i=1:3610
%     robot
    robot.steerToDestState();
    xPrev = [xPrev, robot.getState.x];
    yPrev = [yPrev, robot.getState.y];
    fi = [fi robot.getState.fi];

    %update plot data
    set(h,'Xdata',xPrev,'Ydata',yPrev)
    drawnow
end
