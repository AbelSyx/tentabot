filename = '/home/server/workspace/tentabot_ws/src/tentabot/dataset/trajectory_sampling/rosbot/20220402_230708/trajectory_data.csv';
M = csvread(filename);

[r,c] = size(M)

for ri = 1:r
    x = []
    y = []
    z = []
    for xi = 1:3:c
        x = [x M(ri,xi)];
    end
    
    for yi = 2:3:c
        y = [y M(ri,yi)];
    end
    
    for zi = 3:3:c
        z = [z M(ri,zi)];
    end

    plot3(x,y,z,'o')
    %hold on
end
%hold off
