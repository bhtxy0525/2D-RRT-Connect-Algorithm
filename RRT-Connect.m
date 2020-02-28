%  /*RRT-Connect algoritm in 2D with collision avoidance
% *Version: MATLAB 2017b
% *Time: 2020.02.27
% *Author: Rot_Tianers*/

clear all;close all;clc;
%% 自由建图
map=~zeros(500,500);
map(1:450,70:120) = 0;
map(50:500,350:400) = 0;
imshow(map);
[m,n] = size(map);

%% 定义初始化参数，以左上角为原点计数，矩阵第一个数表示y，第二个数表示x
qinit = [100,20];          %起始点
qgoal = [480,480];         %目标点
stepsize = 20;             %搜索步长
disTh = 20;                %阈值
maxFailAttempts = 2000;    %搜索上限
counter = 0;               %计数次数
display = 1;               %显示上述数据为真

%% 判断起始点和目标点是否符合要求
if feasiblePoint(qinit,map) ==0
	error('起始点不符合地图要求');%%error后面的程序不会被执行
end

if feasiblePoint(qgoal,map) ==0
	error('目标点不符合地图要求');%%error后面的程序不会被执行
end
%% 显示地图
if display
	imshow(map);
    rectangle('position',[1 1 size(map)-1],'LineWidth', 2,'edgecolor','k');
    hold on;
    scatter(qinit(2),qinit(1),'sm','filled');
    hold on;
    scatter(qgoal(2),qgoal(1),'sb','filled');
end
%% 程序开始并计时
tic;
RRTree1 = double([qinit 0]);   %%起始点随机树
RRTree2 = double([qgoal -1]);   %%终止点随机树
Tree1ExpansionFail = 0;
Tree2ExpansionFail = 0;
while ~Tree1ExpansionFail || ~Tree2ExpansionFail
    %% 扩展第一课树
    if ~Tree1ExpansionFail
        [RRTree1,pathFound,Tree1ExpansionFail] = rrtExtend(RRTree1,RRTree2,qgoal,stepsize,maxFailAttempts,disTh,map);%扩展Tree1，Tree1ExpansionFail的返回值永远为0
         if ~Tree1ExpansionFail && isempty(pathFound) && display   %%如果扩展树成功，同时没有到达最终点
           %% Tree1新节点与父节点连线
             line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b','linewidth',3);
             counter=counter+1;M(counter)=getframe;
         end
    end
    %% 扩展第二课树
    if ~Tree2ExpansionFail
        [RRTree2,pathFound,Tree2ExpansionFail] = rrtExtend(RRTree2,RRTree1,qinit,stepsize,maxFailAttempts,disTh,map);
       %% 互换位置,保持一致，以便下文编程
        if ~isempty(pathFound)
            pathFound(3:4)=pathFound(4:-1:3); 
        end 
    end
    %% 交换搜索次序
    Swap(RRTree1,RRTree2);
    %% 找到路径
    if ~isempty(pathFound)       
        if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green','linewidth',3);
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3);     % add nodes from RRT 1 first
        while prev > 1
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4);     % then add nodes from RRT 2
        while prev > 0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3)
        end
        break;
    end
end
line(path(:,2),path(:,1),'linestyle','--','linewidth',2,'color','y');
toc;
title('2D RRT-Connect Algorithm');