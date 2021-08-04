addpath(genpath('/home/hopkinsonlab/Documents/MATLAB/Library/gridfitdir'));

pc_file = './points3D_test.txt';

fid = fopen(pc_file,'r');

pc_raw = [];
i = 1;
limit_xyz = 20; %max x,y,z dimensions of point cloud

while(~feof(fid))

    line = fgetl(fid);
    C = strsplit(line);
    x = str2double(C{2});
    y = str2double(C{3});
    z = str2double(C{4});
    if(abs(x) > limit_xyz || abs(y) > limit_xyz || abs(z) > limit_xyz)
        continue;
    end
    pc_raw(i,:) = [x, y, z];
    i = i+1;
end

%smooth point cloud
pc = pointCloud(pc_raw);
pcout = pcdenoise(pc);
pcout_raw = pcout.Location;

%fit surface to pointcloud
pc_min = min(pcout_raw);
pc_max = max(pcout_raw);
gx = pc_min(1):0.25:pc_max(1);
gy = pc_min(2):0.25:pc_max(2);

g=gridfit(pcout_raw(:,1),pcout_raw(:,2),pcout_raw(:,3),gx,gy);
grid_size = size(g);
grid_size_x = grid_size(2);
grid_size_y = grid_size(1);


%convert from surface grid to triangular mesh
x_mesh = repmat(gx,size(gy,2),1);
x_mesh = x_mesh(:);

y_mesh = repmat(gy',1,size(gx,2));
y_mesh = y_mesh(:);

z_mesh = g(:);

n_elm = 2* (grid_size_y -1) * (grid_size_x - 1);
T = zeros(n_elm,3); %triangular vertex matrix

elm = 1;
for j = 1:grid_size_x-1
    for i = 1:grid_size_y-1
        T(elm, :)   = [ (j-1)*grid_size_y + i     , (j-1)*grid_size_y + i + 1, j*grid_size_y + i     ];
        T(elm+1, :) = [ (j-1)*grid_size_y + (i + 1), j*grid_size_y + i        ,  j*grid_size_y + (i+1) ];
        elm = elm + 2;
    end
end

%visualize
TO = triangulation(T,x_mesh,y_mesh,z_mesh);
trimesh(TO);
hold on
scatter3(pcout_raw(:,1), pcout_raw(:,2), pcout_raw(:,3));