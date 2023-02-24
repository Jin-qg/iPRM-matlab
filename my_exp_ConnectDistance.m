% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% Experiments of Connection Distance in PRM
close;
clc;
clear;

%% input settings
map_names = {'map1','map2','map_lib','map_pkl'};
nodeNums = [10,10; 60,15; 60,20; 60,20];
startLocations = [30,40; 30,40; 75 110; 90,170];
endLocations = [460,450; 480,470; 680 525; 550,530];
connectDis = [0.25, 0.5, 0.75, 1];
safeDistance = 5;

nodes_group_num = 3;
nodes_repeat_num = 10;
display = true;
save = false;

for map_i = 1:size(map_names,2)
    map_name = sprintf('%s',map_names{map_i});
    
    %% load map and source location
    startLocation = startLocations(map_i,:);
    endLocation = endLocations(map_i,:);
    
    %% iter nodes num
    for node_group_i = 1:nodes_group_num
        nodeNum = nodeNums(map_i,1) + nodeNums(map_i,2)*(node_group_i-1);
        data = [];
        
       %% iter creating repeatedly same amount of nodes
        for node_repeat_i = 1:nodes_repeat_num
            nodeRepeat = node_repeat_i;
            fprintf('=====exp %s-%d-%d=====\n',map_name,nodeNum,node_repeat_i);
            
            for cd_i = 1:size(connectDis,2)
                fprintf('%s-%d-%d-%s: ',map_name,nodeNum,nodeRepeat,num2str(connectDis(cd_i)));
                cell = myEXPFuncConnectDis(map_name,nodeNum,node_repeat_i,startLocation,endLocation,connectDis(cd_i),safeDistance,display,save);
                data = [data;cell];
            end
            
            if display, waitforbuttonpress; end
           
        end %of each repeat
       %% write xls
        con_dis_num = size(connectDis,2);
        output_path = strcat('./result/connect_distance/',map_name,'_cd.xlsx');
        line_start = 3 + (node_group_i-1)*nodes_repeat_num*con_dis_num;
        line_end = line_start + nodes_repeat_num*con_dis_num - 1;
        xlrange = sprintf('B%d:L%d',line_start,line_end);
        xlswrite(output_path, data, xlrange);
        
        fprintf('%s : exp connect distance finished.\n', map_name);
        
    end %of each nodes_num   
end %of each map









