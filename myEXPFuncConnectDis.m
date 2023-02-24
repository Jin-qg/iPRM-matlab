% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% Experiment Data Output of PRM Connection Distance
function output_data=myEXPFuncConnectDis(expMapName,nodeNum,nodeRepeat,startLocation,endLocation,expConnectDis,safeDis,display,save)

%% Input settings
map_name = expMapName;
connectDistanceWeight = expConnectDis;
output_data = {nodeNum,nodeRepeat,expConnectDis, 0,0,0, Inf, Inf,Inf,Inf, 0};

%% Load map
fprintf('Loading map. ');
t = clock;
if strcmp(map_name,'map1')
    map = im2bw(imread('./src/map1.bmp'));
elseif strcmp(map_name,'map2')
    map = im2bw(imread('./src/map2.bmp'));
elseif strcmp(map_name,'map_lib')
    map = im2bw(imread('./src/map_lib.bmp'));
elseif strcmp(map_name,'map_pkl')
    map = im2bw(imread('./src/map_pkl.bmp'));
end
time = etime(clock,t);
output_data{7} = time;


%% Read nodes
fprintf('Reading nodes. ');
file_name1 = sprintf('./src/fixed_nodes/%s/%d_%d.txt',map_name,nodeNum,nodeRepeat);
fid = fopen(file_name1,'r');
prmNodesRead = textscan(fid,'%f %f\n',nodeNum);
prmNodes = [startLocation;endLocation;prmNodesRead{1} prmNodesRead{2}];
fclose(fid);

if display
    imshow(map);
    rectangle('position',[1 1 fliplr(size(map))-1],'edgecolor','k')
    showSourceLocation(startLocation, endLocation);
    for i = 3:length(prmNodes)
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
edgeNumCollided = 0;
edgeNumSkipped = 0;

timeTotal2 = 0;
t = clock;
prmEdges = cell(nodeNum+2, 1);
prmConnectDistance = connectDistanceWeight * sqrt(sum(size(map).^2));

for i=1:nodeNum+2
    for j=i+1:nodeNum+2
        if distancePoints(prmNodes(i,:),prmNodes(j,:)) > prmConnectDistance
            edgeNumSkipped = edgeNumSkipped + 1;
            continue;
        else
            if checkPath(prmNodes(i,:),prmNodes(j,:),map,safeDis)
                prmEdges{i} = [prmEdges{i};j];
                prmEdges{j} = [prmEdges{j};i];
                edgeNum = edgeNum + 1;
                if display, line([prmNodes(i,2);prmNodes(j,2)],[prmNodes(i,1);prmNodes(j,1)]); end
            else
                edgeNumCollided = edgeNumCollided + 1;
                if display, line([prmNodes(i,2);prmNodes(j,2)],[prmNodes(i,1);prmNodes(j,1)],'Color','r'); end
            end
        end
    end
end
time = etime(clock,t);
output_data{7} = time;
output_data{4} = edgeNum;
output_data{5} = edgeNumCollided;
output_data{6} = edgeNumSkipped;

if display 
    fprintf('Edges are shown.\n');
    fprintf('Total edge num : %d. Failed edge num : %d. [c/p]\n',edgeNum, edgeNumCollided);
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


%% Output result
if ~pathFound
    fprintf('No path found.\n');
else
    fprintf('Path found.\n');
    t = clock;
    prmPath = [2,prmNodes(n(1),:)];
    prev = n(5);
    while prev>0
        node_temp = [closed(prev,1),prmNodes(closed(prev,1),:)];
        prmPath = [node_temp;prmPath];
        prev = closed(prev,5);
    end
    time = etime(clock,t);
    output_data{11} = time;
    
    prmPathLength = 0;
    for i=1:length(prmPath)-1
        prmPathLength = prmPathLength + distancePoints(prmPath(i,2:3),prmPath(i+1,2:3));
    end
    if display
        line(prmPath(:,3),prmPath(:,2),'color','r','LineWidth',2);
        waitforbuttonpress; 
    end
    output_data{8} = prmPathLength;
    
    %% Smooth
    [prmPath1, prmPath1_length] = myEXPFuncSmooth1(prmPath,map,display);
    [prmPath2, prmPath2_length] = myEXPFuncSmooth2(prmPath1,map,display);
    output_data{9} = prmPath1_length;
    output_data{10} = prmPath2_length;
    output_data{11} = 1;
    
    if save
        file_name1 = sprintf('./result/path/traditional/%s/%d_%s_%d_1.txt',map_name,nodeNum,num2str(expConnectDis),nodeRepeat);
        fid = fopen(file_name1,'w');
        for i = 1:size(prmPath,1)
            fprintf(fid,'%d %d %d\n',prmPath(i,1),prmPath(i,2),prmPath(i,3));
        end
        fclose(fid);
        
        file_name2 = sprintf('./result/path/traditional/%s/%d_%s_%d_2.txt',map_name,nodeNum,num2str(expConnectDis),nodeRepeat);
        fid = fopen(file_name2,'w');
        for i = 1:size(prmPath1,1)
            fprintf(fid,'%d %d %d\n',prmPath1(i,1),prmPath1(i,2),prmPath1(i,3));
        end
        fclose(fid);
        
        file_name3 = sprintf('./result/path/traditional/%s/%d_%s_%d_3.txt',map_name,nodeNum,num2str(expConnectDis),nodeRepeat);
        fid = fopen(file_name3,'w');
        for i = 1:size(prmPath2,1)
            fprintf(fid,'%d %d %d\n',prmPath2(i,1),prmPath2(i,2),prmPath1(i,3));
        end
        fclose(fid);
    end

end


