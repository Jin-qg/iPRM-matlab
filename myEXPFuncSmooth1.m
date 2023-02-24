% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% Experiment of Backward-Check Optimization
function [prmPath1, prmPath1_length] = myEXPFuncSmooth1(prmPath,map,safeDis,display)
    prmPath1 = prmPath;
    i = 1;
    j = i+1;
    rm_id = [];
    while j <= size(prmPath1,1)
        index = prmPath1(i,1);
        [r,~] = find(prmPath1(i:end,1) == index);
        if size(r,1) > 1
            new_i = max(r)+i-1;
            old_i = min(r)+i-1;
            rm_id = [rm_id old_i:(new_i-1)];
            i = new_i;
            j = i + 1;
        else
            if checkPath(prmPath1(i,2:3), prmPath1(j,2:3), map, safeDis)
                if display
                    node1 = prmPath1(i,2:3);
                    node2 = prmPath1(j,2:3);
                end
                rm_id = [rm_id j];
                j = j + 1;
            else
                rm_id(end) = [];
                i = j - 1;
            end
        end
    end
    if rm_id(end) == size(prmPath1,1)
        rm_id(end) = [];
    end
    
    prmPath1(rm_id,:) = [];
    prmPath1_length = 0;
    for i=1:size(prmPath1,1)-1
        prmPath1_length = prmPath1_length + distancePoints(prmPath1(i,2:3),prmPath1(i+1,2:3));
        if display, line(prmPath1(:,3),prmPath1(:,2),'Color','c','LineWidth',5); end
    end
    if display, waitforbuttonpress; end