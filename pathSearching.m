% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

function newPath = pathSearching(startIndex,endIndex,prmNodes,prmEdges,display)
    if display, fprintf('Start searching new path between node %d and %d.\n',startIndex,endIndex); end
    t_ps = clock;
    
    endLocation = prmNodes(endIndex,:);
    Q = [startIndex 0 heuristic(prmNodes(startIndex,:),endLocation) 0+heuristic(prmNodes(startIndex,:),endLocation) -1];
    closed = [];
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
    
    if ~pathFound
        newPath = [];
        return
    else
        newPath = [endIndex,prmNodes(n(1),:)];
        prev = n(5);
        while prev>0
            node_temp = [closed(prev,1),prmNodes(closed(prev,1),:)];
            newPath = [node_temp;newPath];
            prev = closed(prev,5);
        end

        time_ps = etime(clock,t_ps);
        if display, fprintf('Processing time=%s.\n', num2str(time_ps)); end
    end
    