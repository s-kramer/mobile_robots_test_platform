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

%%%%%%%%%%%%% CONFIGURATION %%%%%%%%%%%%%
%simulation arguments
howManyLeaders = 1;
howManyAgents = 6;
howManyLandmarks = 2;
howManyIterations = 600;
howOftenForceCalculation = 1;   %force calculated (mod(howManyIterations,howOftenForceCalculation)==0) times
worldSize = 100;    %world x and y size

useForce = 'inverse';    %The proper arguments are: {'spring','inverse','gradient'}

%Plot colors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%blue circle   red plus    black star
%Agent         Landmark    Leader
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%% IMPLEMENTATION  %%%%%%%%%%%%%
%%%%%%%%informational part only
if(strcmp(useForce,'spring'))
    disp('Spring forces simulation.');
elseif(strcmp(useForce,'inverse'))
    disp('Inverse power law simulation.');
else
    disp('Gradient Climbing law simulation.');
end

%world definition
world = lib.World(worldSize);
disp('World created');

%robots definition
robots={};
if(strcmp(useForce,'spring'))
    for i=1:howManyLeaders 
    %%%%%%The force for a particular robot may be changed below
    %%%%%% SPRING LAW %%%%%%
        robots{end+1} = lib.robots.Leader(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)),lib.forces.Steady,i);    
    end
    for i=1:howManyAgents 
        robots{end+1} = lib.robots.Agent(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)),lib.forces.SpringLaw,i);    
    end
    for i=1:howManyLandmarks 
        robots{end+1} = lib.robots.Landmark(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)),lib.forces.Steady,i);    
    end
elseif(strcmp(useForce,'inverse'))
    %%%%%% INVERSE LAW %%%%%%
    for i=1:howManyLeaders 
        robots{end+1} = lib.robots.Leader(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)));
    end
    for i=1:howManyAgents 
        robots{end+1} = lib.robots.Agent(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)));    
    end
    for i=1:howManyLandmarks 
        robots{end+1} = lib.robots.Landmark(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)),lib.forces.Steady);    
    end
else
    %%%%%% GRADIENT LAW %%%%%%
    for i=1:howManyLeaders 
        robots{end+1} = lib.robots.Leader(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)),lib.forces.Circle,i);    
    end
    for i=1:howManyAgents 
        robots{end+1} = lib.robots.Agent(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)),lib.forces.GradientClimbLaw,i);    
    end
    for i=1:howManyLandmarks 
        robots{end+1} = lib.robots.Landmark(world,struct('x',randi(worldSize),'y',randi(worldSize),'fi',randi(360)),lib.forces.Steady,i);    
    end
end

%Appent robots to world
world.addRobots(robots);
disp(sprintf('Robots created: \n%i Leaders \t %i Agents \t %i Landmarks',howManyLeaders,howManyAgents,howManyLandmarks));

%plot
disp('Simulation in progress...');
for i=1:howManyIterations 
    if(mod(i,25)==0)
        disp(sprintf('%i\\%i',i,howManyIterations));
    end
    if(mod(i,howOftenForceCalculation)==0)
        for j=1:numel(robots)
            if(strcmp(useForce,'gradient'))
                world.controllerUpdate();            
            end
            robots{j}.update(); 
        end
    end
    for j=1:numel(robots)
       robots{j}.steerToDestState(); 
    end
    world.update();
    drawnow
end
disp('End of simulation.');
clear i j worldSize
