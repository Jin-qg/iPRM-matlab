% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% A demo of PRM on Multilayer Map
close;
clc;
clear;

%% input settings
nodes_num = 3; % choose node num by id{1, 2 ,3} -> {node-num-1, node-num-2, node-num-3}
fixed_nodes_map_num = 10; % choose node distribution under node num
display = true; % {true,false} whether to show planning results. 
safeDistance = 5; % distance within which space is regarded as occupied.

%% load map and source location
map_names = {'map_lib_l', 'map_lib_u'};
map_name_l = sprintf('%s',map_names{1});
map_name_u = sprintf('%s',map_names{2});

nodeNums = [60,20];
startLocation = [75 110];
endLocation = [680 525];
connectDis = [0.25, 0.5, 0.75, 1];

%% iter nodes num
for node_num_i = 1:nodes_num
    nodeNum = nodeNums(1,1) + nodeNums(1,2)*(node_num_i-1);
    data = [];

    for cd_i = 1:size(connectDis,2)
        con_dis_num = size(connectDis,2);
            
        % down map Area 1
        cell = myEXPFuncBasicPRMforMultiLayer(map_name_l,30,1,startLocation,endLocation,connectDis(cd_i),safeDistance,display);
        data = [data;cell];
        
        % upper map Area 3
        cell = myEXPFuncBasicPRMforMultiLayer(map_name_u,10,3,startLocation,endLocation,connectDis(cd_i),safeDistance,display);
        data = [data;cell];
    
        % down map Area 2
        cell = myEXPFuncBasicPRMforMultiLayer(map_name_l,30,2,startLocation,endLocation,connectDis(cd_i),safeDistance,display);
        data = [data;cell];
    end

    if display, waitforbuttonpress; end
end %of each nodes_num   









