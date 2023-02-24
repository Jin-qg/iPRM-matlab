% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% Experiment Preparation to Create Node Files in This Project
clc;
clear;

%% Input settings
map_names = {'map1','map2','map_lib','map_pkl'}; % change this to your map name
nodeNums = [10,10; 60,15; 60,20; 60,20]; % change this as the node number and its increment
startLocations = [30,40; 30,40; 75 110; 90,170]; % set the start and end location of your map
endLocations = [460,450; 480,470; 680 525; 550,530];
safeDistance = 5; % distance within which space is regarded as occupied.

display = false;
nodes_group_num = 3;
nodes_repeat_num = 10;

for map_i = 1:size(map_names,2)
    map_name = map_names{map_i};
    fprintf('%s is now creating nodes.\n',map_name);
    
    %% load map and source location
    map = im2bw(imread(strcat('./src/',map_name,'.bmp')));
    startLocation = startLocations(map_i,:);
    endLocation = endLocations(map_i,:);
    
    data = [];
    %% iter nodes num
    for node_group_i = 1:nodes_group_num
        nodeNum = nodeNums(map_i,1) + nodeNums(map_i,2)*(node_group_i-1);
        
       %% iter creating repeatedly same amount of nodes
        for node_repeat_i = 1:nodes_repeat_num
            if display
                imshow(map);
                rectangle('position',[1 1 fliplr(size(map))-1],'edgecolor','k');
                showSourceLocation(startLocation,endLocation);
            end
            
            t = clock;
            prmNodes = [startLocation;endLocation];
            while length(prmNodes) < nodeNum+2
                x = double(int32(rand(1,2) .* size(map)));
                x1 = int32(rand(1,2) .* size(map));
                if feasiblePoint(x,map,safeDistance)
                    prmNodes = [prmNodes;x]; 
                end
            end
            time = etime(clock,t);
            cell = {' ' , Inf};
            cell{1} = sprintf('%d_%d',nodeNum,node_repeat_i);
            cell{2} = time;
            data = [data; cell];
            

            file_name = sprintf('./src/fixed_nodes/%s/%d_%d.txt',map_name,nodeNum,node_repeat_i);
            fid = fopen(file_name,'w');
            for i = 3:length(prmNodes)
                fprintf(fid,'%d %d\n',prmNodes(i,1),prmNodes(i,2));
            end
            fclose(fid);
            fprintf('%s/%d_%d.txt is created in time %s. [c/p]\n',map_name,nodeNum,node_repeat_i,num2str(time));

            if display
                for i = 3:length(prmNodes)
                    x = prmNodes(i,1:2);
                    rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','b');
                    text(x(2),x(1)-10,num2str(i),'Color','r','FontSize',10);
                end
                waitforbuttonpress;
            end
 
        end %end of each repeat
    end %end of each nodes_num
    %% generate exp data
    
    output_path = strcat('./result/create_nodes.xlsx');
    xlranges = {'A1:B50','D1:E50','G1:H50','J1:K50'};
    xlrange = xlranges{map_i};
    xlswrite(output_path, data, xlrange);
    
end %end of each map










