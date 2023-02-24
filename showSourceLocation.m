% Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% This work is as the source code of article paper accessed on: https://www.mdpi.com/2504-446X/7/2/92.
% Please cite the work in all materials as: Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92. https://doi.org/10.3390/drones7020092.
% or other appropriate citation style.

function showSourceLocation(startLocation, endLocation)
        line([startLocation(2); endLocation(2)],[startLocation(1);endLocation(1)],'color','y');
        line([startLocation(2)-5; startLocation(2)+5],[startLocation(1);startLocation(1)],'color','g','linewidth',5);
        line([startLocation(2); startLocation(2)],[startLocation(1)-5;startLocation(1)+5],'color','g','linewidth',5);
        line([endLocation(2)-5; endLocation(2)+5],[endLocation(1);endLocation(1)],'color','r','linewidth',5);
        line([endLocation(2); endLocation(2)],[endLocation(1)-5;endLocation(1)+5],'color','r','linewidth',5);
