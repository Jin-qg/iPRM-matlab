% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% A demo of Basic PRM
clc;
clear;

%% Input settings
choose_map = 4; % choose map by id{1,2,3,4}.
display = true; % {true,false} whether to show planning results.
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
while length(prmNodes)<nodeNum+2
    x=double(int32(rand(1,2) .* size(map)));
    if feasiblePoint(x,map,safeDistance)
        prmNodes=[prmNodes;x]; 
        if display, rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','b'); end
    end
end
time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('time : %s\n',num2str(time));
if display
    fprintf('Nodes are shown.\n Total node num : %d. [click/press]\n',nodeNum+2);
    waitforbuttonpress; 
end


%% Create edges (adjacency list)
disp('Start creating edges.');
edgeNum = 0;
t = clock;
prmEdges=cell(nodeNum+2,1); % edges to be stored as an adjacency list
for i=1:nodeNum+2
    for j=i+1:nodeNum+2
        if checkPath(prmNodes(i,:),prmNodes(j,:),map,safeDistance)
            prmEdges{i}=[prmEdges{i};j];prmEdges{j}=[prmEdges{j};i];
            edgeNum = edgeNum + 1;
            if display, line([prmNodes(i,2);prmNodes(j,2)],[prmNodes(i,1);prmNodes(j,1)]); end
        end
    end
    fprintf('Creating edges from node %d finished.\n',i);
end
time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('time : %s\n',num2str(time));
if display 
    fprintf('Edges are shown.\n Total edge num : %d. [c/p]\n',edgeNum);
    waitforbuttonpress; 
end


%% Path searching (using A*)
disp('Start searching path.');
t = clock;
Q=[1 0 heuristic(prmNodes(1,:),endLocation) 0+heuristic(prmNodes(1,:),endLocation) -1]; % A* algorihtm open list
closed=[]; % A* algorihtm closed list
pathFound=false;
while size(Q,1)>0
    % find smallest 
    [A, I]=min(Q,[],1);
     n=Q(I(4),:); % smallest total_cost element to process
     Q=[Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; % delete element under processing
     
     % goal test
     if n(1) == 2
         pathFound=true;break;
     end
     
     % iterate through all edges from the node
     for mv=1:length(prmEdges{n(1),1}) 
         newNode=prmEdges{n(1),1}(mv);
         
         % not already in closed
         if isempty(closed) || isempty(find(closed(:,1)==newNode, 1))
             historicCost=n(2)+historic(prmNodes(n(1),:),prmNodes(newNode,:));
             heuristicCost=heuristic(prmNodes(newNode,:),endLocation);
             totalCost=historicCost+heuristicCost;
             add=true; 
             
             % not already in queue with better cost
             if length(find(Q(:,1)==newNode))>=1
                 I=find(Q(:,1)==newNode);
                 if Q(I,4)<totalCost
                     add=false;
                 else
                     Q=[Q(1:I-1,:);Q(I+1:end,:);];
                     add=true;
                 end
             end
             
             % add new nodes in queue
             if add
                 Q=[Q;newNode historicCost heuristicCost totalCost size(closed,1)+1]; 
             end
         end           
     end
     
     % update closed lists
     closed=[closed;n];
end
time = etime(clock,t);
timeTotal = timeTotal + time;
fprintf('time : %s\n',num2str(time));
if ~pathFound
    error('No path found')
end


%% Output result
fprintf('Processing time=%s. Path length=%s.\n', num2str(timeTotal), num2str(n(4))); 
prmPath=[prmNodes(n(1),:)]; %retrieve path from parent information
prev=n(5);
while prev>0
    prmPath=[prmNodes(closed(prev,1),:);prmPath];
    prev=closed(prev,5);
end

if display
    line(prmPath(:,2),prmPath(:,1),'color','r','LineWidth',2);
end