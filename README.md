iPRM Guides
---

This is the path planning source code of article paper entitled ***"An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles"*** (https://www.mdpi.com/2504-446X/7/2/92).

**Qingeng Jin, School of Remote Sensing and Information Engineering, Wuhan University, Creative Commons Attribution-ShareAlike 4.0 International License.**
The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
 Please cite the work in all materials as: ***Jin, Q.; Hu, Q.; Zhao, P.; Wang, S.; Ai, M. An Improved Probabilistic Roadmap Planning Method for Safe Indoor Flights of Unmanned Aerial Vehicles. Drones 2023, 7, 92.*** *https://doi.org/10.3390/drones7020092*. or other appropriate citation style.

# Contents
- [Contents](#contents)
- [1 File Format and Folder Structure](#1-file-format-and-folder-structure)
- [2 Experiment Params](#2-experiment-params)
  - [2.1 Path Planning Params](#21-path-planning-params)
  - [2.2 Experiment Params](#22-experiment-params)
  - [2.3 Other Params](#23-other-params)
- [3 How to Run the Experiments](#3-how-to-run-the-experiments)
  - [3.1 Map Data Preparation](#31-map-data-preparation)
  - [3.2 Demo Only](#32-demo-only)
    - [3.2.1 Demo: Basic PRM](#321-demo-basic-prm)
    - [3.2.2 Demo: iPRM](#322-demo-iprm)
    - [3.2.3 Demo: Planning in Multilayer Map](#323-demo-planning-in-multilayer-map)
  - [3.3 Experimental Analysis](#33-experimental-analysis)
    - [3.3.1 Automatically Create Node Files](#331-automatically-create-node-files)
    - [3.3.2 Analysis: Basic PRM](#332-analysis-basic-prm)
    - [3.3.3 Analysis: Connection Distance](#333-analysis-connection-distance)
    - [3.3.4 Analysis: iPRM](#334-analysis-iprm)
- [4 Output Results](#4-output-results)
  - [4.1 Basic PRM](#41-basic-prm)
  - [4.2 Connection Distance](#42-connection-distance)
  - [4.3 iPRM](#43-iprm)


# 1 File Format and Folder Structure
The file folder includes:
- Source data
  - Virtual binary image maps
    - `./src/map1.bmp`
    - `./src/map2.bmp`
  - Reduced-dimensional map
    - `./src/map_lib.bmp`: a library reading room.
    - `./src/map_pkl.bmp`: a underground parking lot.
  - Nodes for control
    - `./src/fixed_node/{map_name}/{node_num}_{repeat_num}.txt`
  - Multilayer map demo.
    - `./src/multilayers/map_lib_l.bmp`: lower layer map (Area 1 and Area 2, referred to the paper).
    - `./src/multilayers/map_lib.u.bmp`: upper layer mao (Area 3).
- Execution
  - Validation
    - `./checkPath.m`
    - `./distancePoints.m`
    - `./feasiblePoint.m`
    - `./feasiblePoint2.m`
    - `./heuristic.m`
    - `./historic.m`
    - `./showSourceLocation.m`
    - `./pathSearching.m`
  - Planning
    - `./myEXPFuncBasicPRM.m`
    - `./myEXPFuncConnectDis.m`
    - `./myEXPFuncCollisionSmooth.m`
    - `./myEXPFuncSmooth1.m`
    - `./myEXPFuncSmooth2.m`
    - `./myEXPFuncBasicPRMforMultiLayer.m`
- Demo
  - `./my_demo_BasicPRM.m`
  - `./my_demo_iPRM.m`
  - `./my_demo_MultilayerMap.m`
- Experiments
  - `./my_exp_CreateNodes.m`
  - `./my_exp_BasicPRM.m`
  - `./my_exp_ConnectDistance.m`
  - `./my_exp_CollisionSmooth.m`

# 2 Experiment Params
## 2.1 Path Planning Params
nodeNums: number of nodes to sample in the map.
connectDis: distance within which the nodes are checked for collision.
startLocation: source node where path search begins.
endLocation: goal node where path search ends.
safeDistance: distance within which space is regarded as occupied.

## 2.2 Experiment Params
map_name: choose a map from *{map1, map2, map_lib, map_pkl}*.
node_group_num (**node_group_i**): different node number.
nodes_repeat_num (**node_repeat_i**): different node distribution under same node number.
**cd_i**: different connection distance under same nodes.

## 2.3 Other Params
display: whether to show planning results.
save: whether to save the statistic and analysis results using *Excel*.

# 3 How to Run the Experiments
## 3.1 Map Data Preparation
This project contains 4 maps that are used in the paper. Users are available for using your own map by adding the map in `./src/{your-map-name}.bmp` and changing the `{map-name}` to your map in the project. In this way, you can choose one of the following solutions:
- Use random node generation, referring to [3.2 Demo Only](#32-demo-only).
- Use `./my_exp_CreateNodes.m` to automatically create nodes, referring to [3.3.1 Automatically Create Node Files](#331-automatically-create-node-files).
- Create nodes by your own and add node files in `./src/fixed_nodes/{your-map-name}/{node-num}_{repeat-num}.txt`.

## 3.2 Demo Only
### 3.2.1 Demo: Basic PRM
In `./my_demo_BasicPRM.m`:
```matlab
%% Input settings
choose_map = 4; % choose map by id{1,2,3,4}.
display = true; % {true,false} whether to show planning results. 
safeDistance = 5; % distance within which space is regarded as occupied.
```
Press 'F5' in the Editor Window to run the demo.
![bPRM result](https://user-images.githubusercontent.com/62129013/221075736-23ef617b-3cd2-40fc-99cf-8934575f8cad.png)

### 3.2.2 Demo: iPRM
In `./my_demo_iPRM.m`:
```matlab
%% Input settings
choose_map = 4; % choose map by id{1,2,3,4}.
display = true; % {true,false} whether to show planning results. 
display1 = true; % nodes
display11 = false; % edges
display2 = true; % update
display3 = true; % smooth
connectDistanceWeight = 0.75; % control distance within which the nodes are performed for connection. [0.1, 1.0]
safeDistance = 5; % distance within which space is regarded as occupied.
```
Press 'F5' in the Editor Window to run the demo.
![iPRM result](https://user-images.githubusercontent.com/62129013/221075727-19cac570-9dde-4c32-a7ba-37de9d94cac8.png)

### 3.2.3 Demo: Planning in Multilayer Map
in `./my_demo_MultilayerMap.m`:
```matlab
%% input settings
nodes_num = 3; % choose node num by id{1, 2 ,3} -> {node-num-1, node-num-2, node-num-3}
fixed_nodes_map_num = 10; % choose node distribution under node num
display = true; % {true,false} whether to show planning results. 
safeDistance = 5; % distance within which space is regarded as occupied.
```
Press 'F5' in the Editor Window to run the demo.
![multilayer](https://user-images.githubusercontent.com/62129013/221075762-8032b8a3-74dc-4664-8d12-fcf93b8fab23.png)

## 3.3 Experimental Analysis
### 3.3.1 Automatically Create Node Files
If users want to use your own maps and nodes, you can use `./my_exp_CreateNodes.m` to automatically create node files in the project.
In `./my_exp_CreateNodes.m`:
```matlab
map_names = {'map1','map2','map_lib','map_pkl'}; % change this to your map name
nodeNums = [10,10; 60,15; 60,20; 60,20]; % change this as the node number and its increment
startLocations = [30,40; 30,40; 75 110; 90,170]; % set the start and end location of your map
endLocations = [460,450; 480,470; 680 525; 550,530];
safeDistance = 5; % distance within which space is regarded as occupied.
```
Press 'F5' in the Editor Window to automatically create node files saved in `./src/fixed_nodes/{your-map-name}/{node-num}_{repeat_num}.txt`, and the generation time is saved in `./result/create_nodes.txt`.

### 3.3.2 Analysis: Basic PRM
Basic PRM is used as the control group of the paper.
Params to perform a specific experiment:
map_name (**map_i**): choose a map. *{map1, map2, map_lib, map_pkl}*
node_group_num (**node_group_i**): different node number. *{1, 2, 3} -> {node-num-1, node-num-2, node-num-3}*
nodes_repeat_num (**node_repeat_i**): different node distribution under same node number. *{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}*
**cd_i**: different connection distance under same nodes. *[0.1, 1.0]*
In `./my_exp_BasicPRM.m`:
```matlab
for map_i = 1:size(map_names,2)
  ...
  for node_group_i = 1:nodes_group_num
    ...
    for node_repeat_i = 1:nodes_repeat_num
      ...
      for cd_i = 1:size(connectDis,2)
        ...
      end
    end
  end
end
```
Press 'F5' in the Editor Window to do the experiment, results are saved in `./result/basic/{map-name}_bas.xlsx`.

### 3.3.3 Analysis: Connection Distance
Network construction and edge analysis were performed in the paper, do the experiment in `./my_exp_ConnectDistance.m`. Params to perform a specific experiment are the same as [3.3.2 Analysis: Basic PRM](#332-analysis-basic-prm).
Press 'F5' in the Editor Window to do the experiment, results are saved in `./result/connect_distance/{map-name}_cd.xlsx`.
![chart con dis](https://user-images.githubusercontent.com/62129013/221076989-4041f5c7-bbfd-4ea5-b097-0792a1de1ea9.png)

### 3.3.4 Analysis: iPRM
We proposed iPRM method in the paper, do the experiment in `./my_exp_CollisionSmooth.m`. Params to perform a specific experiment are the same as [3.3.2 Analysis: Basic PRM](#332-analysis-basic-prm).
Press 'F5' in the Editor Window to do the experiment, results are saved in `./result/collision_smooth/{map-name}_cs.xlsx`.
![chart iPRM](https://user-images.githubusercontent.com/62129013/221077218-9ebf590d-275e-4bf1-b467-af4c1ea0f6ff.png)
![chart opt](https://user-images.githubusercontent.com/62129013/221077222-0c34e03d-51fb-4cb0-8ef7-261a2bc54134.png)

# 4 Output Results
Statistic and analysis results are organized and recorded in `./result/{experiment-name}/{map-name}_{abbr}.xlsx`.
## 4.1 Basic PRM
Table structure as follows:
| exp_i | node_num | repeat_num | con dis | edge num | time         |             |          |         |         | update count | length_cf | length_sm1 | length_sm2 | success? |
| ----- | -------- | ---------- | ------- | -------- | ------------ | ----------- | -------- | ------- | ------- | ------------ | --------- | ---------- | ---------- | -------- |
|       |          |            |         |          | create edges | search path | get path | smooth1 | smooth2 |              |           |            |            |          |

## 4.2 Connection Distance
Table structure as follows:
| exp_i | node_num | repeat_num | connection dis | edge num      |              |                  | time         | length | sm_length |         | success? |
| ----- | -------- | ---------- | -------------- | ------------- | ------------ | ---------------- | ------------ | ------ | --------- | ------- | -------- |
|       |          |            |                | good edge num | bad edge num | skipped edge num | create edges |        | smooth1   | smooth2 |          |

## 4.3 iPRM
Table structure as follows:
| exp_i | node_num | repeat_num | con dis | edge num | time         |             |          |         |         | update count | length_cf | length_sm1 | length_sm2 | success? |
| ----- | -------- | ---------- | ------- | -------- | ------------ | ----------- | -------- | ------- | ------- | ------------ | --------- | ---------- | ---------- | -------- |
|       |          |            |         |          | create edges | search path | get path | smooth1 | smooth2 |              |           |            |            |          |
