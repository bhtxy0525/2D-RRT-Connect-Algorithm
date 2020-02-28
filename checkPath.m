
%% checkPath 检测extend一段距离后是否与障碍物相撞
function feasible = checkPath(n,newPos,map) %输入为qnearst和qextend
	feasible =1;
	dir = atan2(newPos(1)-n(1),newPos(2)-n(2));
	for r=0:0.5:sqrt(sum((n-newPos).^2))
		posCheck = n+r.*[sin(dir),cos(dir)];
		if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map)&& ...
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
			feasible = 0;
			break;
		end
		if ~feasiblePoint(newPos,map)
			feasible = 0;
        end
    end
end
%floor：向下取整
%ceil：向上取整