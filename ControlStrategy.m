%% Initialiserer lattice planner
load varehus %Henter varehuset på 100 kvadratmeter 
mesh(road,'DisplayName','map');

% Initialiserer lattice planner
lp = Lattice(road,'grid',1 ,'root',[8 2 0]);
%Definerer kost relatert til svinging
lp.plan('cost', [1 100 100]) 

% Plotter lattice
figure;
lp.plot();

%% Ruteplanlegging
start = [2 7 pi/2];
goal = [9 3 3*pi/2];

p = lp.query(start, goal);
figure;
lp.plot();

p2 = lp.query(goal, start);
figure;
lp.plot();

%% Robot med Moving to a point
punkt = p;
figure
hold on
plot(punkt(:,1), punkt(:,2), 'r--');
x0 = punkt(1, 1:3);
for i = 1:size(punkt,1) - 1 
    xg = punkt(i + 1, 1:2);
    r = sim('sl_drivepoint');
    q = r.find('y');
    plot(q(:,1), q(:,2));
    x0 = q(end,:);
end
hold off
grid on
xlim([0 10]);
axis equal


punkt = p2;
figure
hold on
plot(punkt(:,1), punkt(:,2), 'r--');
x0 = punkt(1, 1:3);
for i = 1:size(punkt,1) - 1 
    xg = punkt(i + 1, 1:2);
    r = sim('sl_drivepoint');
    q = r.find('y');
    plot(q(:,1), q(:,2));
    x0 = q(end,:);
end
hold off
grid on
xlim([0 10]);
axis equal

