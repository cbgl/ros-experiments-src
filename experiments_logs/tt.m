load('global_pose')
load('ground_truth')
g = global_pose;
t = ground_truth ;

figure
hold on
axis equal
plot(g(:,1), g(:,2), 'x');
plot(t(:,1), t(:,2), 'o');
