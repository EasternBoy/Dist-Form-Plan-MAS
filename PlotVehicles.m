clear


load('traj.mat')

dimen   = size(traj);
max_idx = dimen(2);
Nv      = dimen(3);

str = {'r', 'g', 'b', 'c',' m','y', [0.6350 0.0780 0.1840], [0 0.4470 0.7410], [0.4940 0.1840 0.5560]};

for snap = 1:1:max_idx
    clf(figure(snap)); hold on; box on
    set(gca, 'position', [0.08 0.08 0.89 0.89], 'FontSize', 18);
    for i = 1:Nv
        h = circle(traj(1,snap,i), traj(2,snap,i), 0.3, str{i});
        axis([0 11 0 11])
        hold on
        plot(traj(1,1:snap,i), traj(2,1:snap,i), 'color', str{i}, LineWidth=1);
        hold on;
    end
    pause(0.2)
end