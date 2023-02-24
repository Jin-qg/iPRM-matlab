% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% Experiment Data Output of iPRM
function output_data=myEXPFuncCollisionSmooth(expMapName,nodeNum,nodeRepeat,startLocation,endLocation,expConnectDis,safeDis,display,save)

%% Input settings
map_name = expMapName;
connectDistanceWeight = expConnectDis;
output_data = {nodeNum,nodeRepeat,expConnectDis, 0, Inf,Inf,Inf,Inf,Inf, 0, 0,0,0, 0};

%% Load map
fprintf('Loading map. ');
if strcmp(map_name,'map1')
    map = im2bw(imread('./src/map1.bmp'));
elseif strcmp(map_name,'map2')
    map = im2bw(imread('./src/map2.bmp'));
elseif strcmp(map_name,'map_lib')
    map = im2bw(imread('./src/map_lib.bmp'));
elseif strcmp(map_name,'map_pkl')
    map = im2bw(imread('./src/map_pkl.bmp'));
end


%% Read nodes
fprintf('Reading nodes. ');
file_name = sprintf('./src/fixed_nodes/%s/%d_%d.txt',map_name,nodeNum,nodeRepeat);
fid = fopen(file_name,'r');
prmNodesRead = textscan(fid,'%f %f\n',nodeNum);
prmNodes = [startLocation;endLocation;prmNodesRead{1} prmNodesRead{2}];
fclose(fid);

if display
    imshow(map);
    rectangle('position',[1 1 fliplr(size(map))-1],'edgecolor','k')
    showSourceLocation(startLocation, endLocation);
    for i = 3:size(prmNodes,1)
        x = prmNodes(i,1:2);
        rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','b');
        text(x(2),x(1)-10,num2str(i),'Color','r','FontSize',10);
    end
    fprintf('Nodes are shown.\n Total node num : %d. [click/press]\n',nodeNum+2);
    waitforbuttonpress; 
end

%% Create edges (adjacency list)
fprintf('Creating edges. ');
edgeNum = 0;
t = clock;
prmEdges = cell(nodeNum+2, 1);
prmConnectDistance = connectDistanceWeight * sqrt(sum(size(map).^2));
for i=1:nodeNum+2
    for j=i+1:nodeNum+2
        if distancePoints(prmNodes(i,:),prmNodes(j,:)) > prmConnectDistance
            continue;
        else
            prmEdges{i} = [prmEdges{i};j];
            prmEdges{j} = [prmEdges{j};i];
            edgeNum = edgeNum + 1;
            if display  
                if checkPath(prmNodes(i,:),prmNodes(j,:),map,safeDis)
                    line([prmNodes(i,2);prmNodes(j,2)],[prmNodes(i,1);prmNodes(j,1)]);
                else
                    line([prmNodes(i,2);prmNodes(j,2)],[prmNodes(i,1);prmNodes(j,1)],'Color','r');
                end
            end
        end
    end
end
time = etime(clock,t);
output_data{4} = edgeNum;
output_data{5} = time;

if display 
    fprintf('Edges are shown.\n');
    fprintf('Total initial edge num : %d. [c/p]\n',edgeNum);
    waitforbuttonpress; 
end


%% Path searching (using A*)
fprintf('Searching path. ');
t = clock;
startIndex = 1;
endIndex = 2;
Q = [startIndex 0 heuristic(prmNodes(startIndex,:),endLocation) 0+heuristic(prmNodes(startIndex,:),endLocation) -1];
closed = [];
pathFound = false;
while size(Q,1) > 0
    [A, I] = min(Q,[],1);
     n = Q(I(4),:);
     Q = [Q(1:I(4)-1,:);Q(I(4)+1:end,:)];
     
     if n(1) == endIndex
         pathFound = true;
         break;
     end
     
     for mv = 1 : length(prmEdges{n(1),1}) 
         newNode = prmEdges{n(1),1}(mv);
         
         if isempty(closed) || isempty(find(closed(:,1)==newNode, 1))
             historicCost = n(2) + historic(prmNodes(n(1),:),prmNodes(newNode,:));
             heuristicCost = heuristic(prmNodes(newNode,:),endLocation);
             totalCost = historicCost + heuristicCost;
             add = true; 
             
             if length(find(Q(:,1)==newNode)) >= 1
                 I = find(Q(:,1)==newNode);
                 if Q(I,4) < totalCost
                     add = false;
                 else
                     Q = [Q(1:I-1,:);Q(I+1:end,:);];
                     add = true;
                 end
             end
             
             if add
                 Q = [Q;newNode historicCost heuristicCost totalCost size(closed,1)+1];
             end
         end           
     end
     
     closed = [closed;n];
end
time = etime(clock,t);
output_data{6} = time;


%% Output result
if ~pathFound
    fprintf('No path found.\n');
else
    %% Initial path
    t = clock;
    prmPath = [2,prmNodes(n(1),:)];
    prev = n(5);
    while prev>0
        node_temp = [closed(prev,1),prmNodes(closed(prev,1),:)];
        prmPath = [node_temp;prmPath];
        prev = closed(prev,5);
    end
    time = etime(clock,t);
    output_data{7} = time;

    if display
        for i=1:size(prmPath,1)-1
            node1 = prmPath(i,2:3);
            node2 = prmPath(i+1,2:3);
            if checkPath(node1,node2,map,safeDis)
                line([node1(1,2);node2(1,2)],[node1(1,1);node2(1,1)],'Color','g','LineWidth',1);
            else
                line([node1(1,2);node2(1,2)],[node1(1,1);node2(1,1);prmNodes(j,1)],'Color','m','LineWidth',1);
            end
        end
        waitforbuttonpress; 
    end

    %% Check inital path 2
    fprintf('Checking path. ');
    update_count = 0;
    t = clock;
    i = 1;
    while i < size(prmPath,1)
        nodeIndex1 = prmPath(i,1);     nodePosition1 = prmPath(i,2:3);
        nodeIndex2 = prmPath(i+1,1);   nodePosition2 = prmPath(i+1,2:3);
        if checkPath(nodePosition1,nodePosition2,map,safeDis)
            i = i + 1;
            if display, line([nodePosition1(1,2);nodePosition2(1,2)],[nodePosition1(1,1);nodePosition2(1,1)],'Color','g','LineWidth',2); end
        else    
            newEdge = prmEdges{nodeIndex1};
            newEdge(newEdge==nodeIndex2) = [];
            prmEdges{nodeIndex1}=newEdge;
            newEdge = prmEdges{nodeIndex2};
            newEdge(newEdge==nodeIndex1) = [];
            prmEdges{nodeIndex2}=newEdge; 

            newPath = pathSearching(nodeIndex1,nodeIndex2,prmNodes,prmEdges,display);
            if isempty(newPath)
                if nodeIndex2 == 2
                    time = etime(clock,t);
                    output_data{6} = output_data{6} + time;
                    output_data{9} = update_count;
                    output_data{13} = 2;
                    fprintf('Update path failed.\n');
                    return;
                else
                    prmPath(i+1,:) = [];
                end
            else 
                prmPath = [prmPath(1:i-1,:);newPath;prmPath(i+2:end,:)];
                if display, line(newPath(:,3),newPath(:,2),'Color','y','LineWidth',1); end
            end
            
            update_count = update_count + 1;
        end
    end
    time = etime(clock,t);
    output_data{6} = output_data{6} + time;
    output_data{10} = update_count;
    
    if display
        fprintf('Check path finished. [c/p]\n');
        waitforbuttonpress; 
    end

    %% Collision-free path
    prmPath_cf_length = 0;
    for i=1:size(prmPath,1)-1
        prmPath_cf_length = prmPath_cf_length + distancePoints(prmPath(i,2:3),prmPath(i+1,2:3)); 
        if display, line(prmPath(:,3),prmPath(:,2),'Color','b','LineWidth',2); end
    end
    output_data{11} = prmPath_cf_length;
    
    if display
        fprintf('Collision-free path : length : %s, time = %s. [c/p]\n',prmPath_cf_length,num2str(time));
        waitforbuttonpress; 
    end
    
    %% Path smoothing opt.
    fprintf('Smoothing. ');
    
    t = clock;
    [prmPath1, prmPath1_length] = myEXPFuncSmooth1(prmPath,map,safeDis,display);
    time = etime(clock,t);
    output_data{8} = time;
    
    t = clock;
    [prmPath2, prmPath2_length] = myEXPFuncSmooth2(prmPath1,map,safeDis,display);
    time = etime(clock,t);
    output_data{9} = time;
    
    output_data{12} = prmPath1_length;
    output_data{13} = prmPath2_length;
    output_data{14} = 1;
    fprintf('Path finished.\n');
    
    if display
        fprintf('Final smooth path : length : %s, time = %s. \n',prmPath2_length,num2str(time));
        fprintf('Smooth ratio: %s. \n',strcat(num2str(prmPath2_length/prmPath_cf_length*100),'%'));
        waitforbuttonpress; 
    end
    
    if save
        file_name1 = sprintf('./result/path/advanced/%s/%d_%s_%d_1.txt',map_name,nodeNum,num2str(expConnectDis),nodeRepeat);
        fid = fopen(file_name1,'w');
        for i = 1:size(prmPath,1)
            fprintf(fid,'%d %d %d\n',prmPath(i,1),prmPath(i,2),prmPath(i,3));
        end
        fclose(fid);
        
        file_name2 = sprintf('./result/path/advanced/%s/%d_%s_%d_2.txt',map_name,nodeNum,num2str(expConnectDis),nodeRepeat);
        fid = fopen(file_name2,'w');
        for i = 1:size(prmPath1,1)
            fprintf(fid,'%d %d %d\n',prmPath1(i,1),prmPath1(i,2),prmPath1(i,3));
        end
        fclose(fid);
        
        file_name3 = sprintf('./result/path/advanced/%s/%d_%s_%d_3.txt',map_name,nodeNum,num2str(expConnectDis),nodeRepeat);
        fid = fopen(file_name3,'w');
        for i = 1:size(prmPath2,1)
            fprintf(fid,'%d %d %d\n',prmPath2(i,1),prmPath2(i,2),prmPath1(i,3));
        end
        fclose(fid);
    end
    
end



