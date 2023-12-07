clear, close all
clc

%create the roadmap:
road = zeros(10, 10);

%add aisles to the map:
road(3:8,[3:4, 7:8]) = 1;

%Defining start position and end position
start = [2 7 pi/2];
goal = [9 3 3*pi/2];

lp = Lattice(road, 'grid', 1, 'root', [2 2 0]);

%Defining cost
lp.plan('cost', [10 10 10])
lp.query(start, goal);

% Plotting the lattice planner
lp.plot()
