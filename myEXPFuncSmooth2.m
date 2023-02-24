% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

% Experiment of Forward-Check Optimization
function [prmPath2, prmPath2_length] = myEXPFuncSmooth2(prmPath1,map,safeDis,display)
    prmPath2 = prmPath1;
    i = 1;
    j = size(prmPath2,1);
    rm_id = [];
    while i < size(prmPath2,1)
        while j > i
            if checkPath(prmPath2(i,2:3), prmPath2(j,2:3), map, safeDis)
                rm_id = [rm_id i+1:j-1];
                i = j;
                j = size(prmPath2,1);
                break;
            else
                j = j - 1;
            end
        end
    end
    prmPath2(rm_id,:) = [];

    prmPath2_length = 0;
    for i=1:size(prmPath2,1)-1
        prmPath2_length = prmPath2_length + distancePoints(prmPath2(i,2:3),prmPath2(i+1,2:3));
        if display, line(prmPath2(:,3),prmPath2(:,2),'Color','g','LineWidth',5); end
    end
    if display, waitforbuttonpress; end