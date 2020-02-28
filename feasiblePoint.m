%%feasiblePoint.m 判断起始点和目标点是否符合要求
function flag = feasiblePoint(point,map)
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
	flag = 0;
else
    flag = 1;
end

%%注：
%% ~()里的条件必须要全部满足才可以
%% map是以左上角为原点开始计数的