% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% A demo of iPRM
clc;
clear;

%% Input settings
choose_map = 4; % choose map by id{1,2,3,4}.
display = true; % {true,false} whether to show planning results. 
display1 = true; % nodes
display11 = false; % edges
display2 = true; % update
display3 = true; % smooth
connectDistanceWeight = 0.75; % control distance within which the nodes are performed for connection. [0.1, 1.0]
safeDistance = 5; % distance within which space is regarded as occupied.
timeTotal = 0;

%% Load map
if (choose_map == 1)
    map=im2bw(imread('./src/map1.bmp'));
    nodeNum = 10;   nodeNumIncrease = 10;
    startLocation=[30 40];  endLocation=[460 450];
elseif (choose_map == 2)
    map=im2bw(imread('./src/map2.bmp'));
    nodeNum =60;    nodeNumIncrease = 15;
    startLocation=[30 40];  endLocation=[480 470];
elseif (choose_map == 3)
    map=im2bw(imread('./src/map_lib.bmp'));
    nodeNum = 60;   nodeNumIncrease = 20;
    startLocation=[75 110];
    endLocation=[680 525];
elseif (choose_map == 4)
    map=im2bw(imread('./src/map_pkl.bmp'));
    nodeNum = 60;   nodeNumIncrease = 20;
    startLocation=[90 170]; % source position in Y, X format
    endLocation=[550 530]; % goal position in Y, X format
end
    
if ~feasiblePoint(startLocation,map,safeDistance), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(endLocation,map,safeDistance), error('goal lies on an obstacle or outside map'); end
if display
    imshow(map);
    rectangle('position',[1 1 fliplr(size(map))-1],'edgecolor','k')
    line([startLocation(2); endLocation(2)],[startLocation(1);endLocation(1)],'color','y');
    line([startLocation(2)-5; startLocation(2)+5],[startLocation(1);startLocation(1)],'color','g','linewidth',5);
    line([startLocation(2); startLocation(2)],[startLocation(1)-5;startLocation(1)+5],'color','g','linewidth',5);
    line([endLocation(2)-5; endLocation(2)+5],[endLocation(1);endLocation(1)],'color','r','linewidth',5);
    line([endLocation(2); endLocation(2)],[endLocation(1)-5;endLocation(1)+5],'color','r','linewidth',5);
    disp('Map is shown. [click/press any key]');
    waitforbuttonpress; 
end


%% Create nodes
disp('Start creating nodes.');
t = clock;
prmNodes=[startLocation;endLocation];
while size(prmNodes,1) < nodeNum+2
    x = double(int32(rand(1,2) .* size(map)));
    x1 = int32(rand(1,2) .* size(map));
    if feasiblePoint(x,map,safeDistance)
        prmNodes = [prmNodes;x]; 
    end
end
time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('time : %s\n',num2str(time));
if display1
    for i = 3:size(prmNodes,1)
        x = prmNodes(i,1:2);
        rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','b');
        text(x(2),x(1)-10,num2str(i),'Color','r','FontSize',10);
    end
    fprintf('Nodes are shown.\n Total node num : %d. [click/press]\n',nodeNum+2);
    waitforbuttonpress; 
end


%% Create edges (adjacency list)
disp('Start creating edges.');
edgeNum = 0;
t = clock;
prmEdges = cell(nodeNum+2, 1); % edges to be stored as an adjacency list
prmConnectDistance = connectDistanceWeight * sqrt(sum(size(map).^2));
for i=1:nodeNum+2
    for j=i+1:nodeNum+2
        if distancePoints(prmNodes(i,:),prmNodes(j,:)) > prmConnectDistance
            continue;
        else
            prmEdges{i} = [prmEdges{i};j];
            prmEdges{j} = [prmEdges{j};i];
            edgeNum = edgeNum + 1;
            if display11
                if checkPath(prmNodes(i,:),prmNodes(j,:),map, safeDistance)
                    line([prmNodes(i,2);prmNodes(j,2)],[prmNodes(i,1);prmNodes(j,1)]);
                else
                    line([prmNodes(i,2);prmNodes(j,2)],[prmNodes(i,1);prmNodes(j,1)],'Color','r','LineWidth',0.5);
                end
            end
        end
    end
    fprintf('Creating edges from node %d finished.\n',i);
end
time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('time : %s\n',num2str(time));
if display11
    fprintf('Edges are shown.\n Total edge num : %d. [c/p]\n',edgeNum);
    waitforbuttonpress; 
end


%% Path searching (using A*)
disp('Start searching path.');
t = clock;
startIndex = 1;
endIndex = 2;
Q = [startIndex 0 heuristic(prmNodes(startIndex,:),endLocation) 0+heuristic(prmNodes(startIndex,:),endLocation) -1]; % A* algorihtm open list
closed = []; % A* algorihtm closed list
pathFound = false;
while size(Q,1) > 0
    % find smallest 
    [A, I] = min(Q,[],1);
     n = Q(I(4),:); % smallest cost element to process
     Q = [Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; % delete element under processing
     
     % goal test
     if n(1) == endIndex
         pathFound = true;
         break;
     end
     
     % iterate through all edges from the node
     for mv = 1 : length(prmEdges{n(1),1}) 
         newNode = prmEdges{n(1),1}(mv);
         
         % not already in closed
         if isempty(closed) || isempty(find(closed(:,1)==newNode, 1))
             historicCost = n(2) + historic(prmNodes(n(1),:),prmNodes(newNode,:));
             heuristicCost = heuristic(prmNodes(newNode,:),endLocation);
             totalCost = historicCost + heuristicCost;
             add = true; 
             
             % not already in queue with better cost
             if length(find(Q(:,1)==newNode)) >= 1
                 I = find(Q(:,1)==newNode);
                 if Q(I,4) < totalCost
                     add = false;
                 else
                     Q = [Q(1:I-1,:);Q(I+1:end,:);];
                     add = true;
                 end
             end
                
             % add new nodes in queue
             if add
                 Q = [Q;newNode historicCost heuristicCost totalCost size(closed,1)+1];
             end
         end           
     end
     
     % update closed lists
     closed = [closed;n];
end
time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('time : %s\n',num2str(time));
if ~pathFound
    error('No path found')
end


%% Initial path
prmPath = [2,prmNodes(n(1),:)];
prev = n(5);
while prev>0
    node_temp = [closed(prev,1),prmNodes(closed(prev,1),:)];
    prmPath = [node_temp;prmPath];
    prev = closed(prev,5);
end
fprintf('Intial Path : length = %s, time = %s. [c/p]\n',num2str(n(4)),num2str(timeTotal)); 

%% Check inital path
if display2
    for i=1:size(prmPath,1)-1
        node1 = prmPath(i,2:3);
        node2 = prmPath(i+1,2:3);
        if checkPath(node1,node2,map,safeDistance)
            line([node1(1,2);node2(1,2)],[node1(1,1);node2(1,1)],'Color','g','LineWidth',2);
        else
            line([node1(1,2);node2(1,2)],[node1(1,1);node2(1,1);prmNodes(j,1)],'Color','m','LineWidth',2);
        end
    end
    waitforbuttonpress; 
end

%% Check inital path 2
update_count = 0;
t = clock;
i = 1;
while i < size(prmPath,1)
    nodeIndex1 = prmPath(i,1);     nodePosition1 = prmPath(i,2:3);
    nodeIndex2 = prmPath(i+1,1);   nodePosition2 = prmPath(i+1,2:3);
    if checkPath(nodePosition1,nodePosition2,map,safeDistance)
        i = i + 1;
        if display2
            line([nodePosition1(1,2);nodePosition2(1,2)],[nodePosition1(1,1);nodePosition2(1,1)],'Color','g','LineWidth',2);
        end
        
    else    
        % delete infeasible node in prmEdges
        newEdge = prmEdges{nodeIndex1};
        newEdge(newEdge==nodeIndex2) = [];
        prmEdges{nodeIndex1}=newEdge;
        newEdge = prmEdges{nodeIndex2};
        newEdge(newEdge==nodeIndex1) = [];
        prmEdges{nodeIndex2}=newEdge; 
        % search a new path and add it to initial path
        newPath = pathSearching(nodeIndex1,nodeIndex2,prmNodes,prmEdges,display);
        if isempty(newPath)
            if prmPath(i+1,1) == 2
                time = etime(clock,t);
                fprintf('end time = %s. \n',num2str(time));
                error('Update path failed.');
            else
                prmPath(i+1,:) = [];
                fprintf('Removed subpath %d-%d.\n',nodeIndex1,nodeIndex2);
            end
        else 
            prmPath = [prmPath(1:i-1,:);newPath;prmPath(i+2:end,:)];
            if display2, line(newPath(:,3),newPath(:,2),'Color','y','LineWidth',1); end
        end

        update_count = update_count + 1;
    end
    
end
time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('Check path finished. [c/p]\n');
waitforbuttonpress; 

%% Collision-free path
prmPath1_length = 0;
for i=1:size(prmPath,1)-1
    prmPath1_length = prmPath1_length + distancePoints(prmPath(i,2:3),prmPath(i+1,2:3)); 
    if display3, line(prmPath(:,3),prmPath(:,2),'Color','y','LineWidth',2); end
end
fprintf('Collision-free path : length : %s,time = %s. [c/p]\n',prmPath1_length,num2str(time));
waitforbuttonpress; 

%% Path smoothing opt.
t = clock;

%% smooth 1
prmPath2 = prmPath;
i = 1;
j = i+1;
rm_id = [];
while j <= size(prmPath2,1)
    index = prmPath2(i,1);
    [r,c] = find(prmPath2(i:end,1) == index);
    if size(r,1) > 1
        new_i = max(r)+i-1;
        old_i = min(r)+i-1;
        rm_id = [rm_id old_i:(new_i-1)];
        i = new_i;
        j = i + 1;
    else
        if checkPath(prmPath2(i,2:3), prmPath2(j,2:3), map, safeDistance)
            rm_id = [rm_id j];
            j = j + 1;
        else
            rm_id(end) = [];
            i = j - 1;
        end
    end
end
if rm_id(end) == size(prmPath2,1)
    rm_id(end) = [];
end
    
prmPath2(rm_id,:) = [];
prmPath2_length = 0;
for i=1:size(prmPath2,1)-1
    prmPath2_length = prmPath2_length + distancePoints(prmPath2(i,2:3),prmPath2(i+1,2:3));
    if display3, line(prmPath2(:,3),prmPath2(:,2),'Color','c','LineWidth',5); end
end
waitforbuttonpress; 

% smooth 2
prmPath3 = prmPath2;
i = 1;
j = size(prmPath3,1);
rm_id = [];
while i < size(prmPath3,1)
    while j > i
        if checkPath(prmPath3(i,2:3), prmPath3(j,2:3), map, safeDistance)
            rm_id = [rm_id i+1:j-1];
            i = j;
            j = size(prmPath3,1);
            break;
        else
            j = j - 1;
        end
    end
end
prmPath3(rm_id,:) = [];

prmPath3_length = 0;
for i=1:size(prmPath3,1)-1
    prmPath3_length = prmPath3_length + distancePoints(prmPath3(i,2:3),prmPath3(i+1,2:3));
    if display3, line(prmPath3(:,3),prmPath3(:,2),'Color','g','LineWidth',5); end
end

time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('Smoothing path finished. [c/p]\n');

fprintf('Final smooth path : length : %s, time = %s. \n',prmPath2_length,num2str(time));
fprintf('Smooth ratio: %s. \n',strcat(num2str(prmPath2_length/prmPath1_length*100),'%'));

fprintf('All process finished. Total time = %s.\n',timeTotal); 
