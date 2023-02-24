% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

function feasible=feasiblePoint(point,map,radius)
feasible=true;
for i=-radius:1:radius
    for j=-radius:1:radius
        point2=[point(1)+i point(2)+j];
        if ~(point2(1)>=1 && point2(1)<=size(map,1) && point2(2)>=1 && point2(2)<=size(map,2) && map(point2(1),point2(2))==1)
            feasible=false;
        end
    end
end
