%%distanceCost 输入随机树矩阵，就是两点之间的最小距离
function h = distanceCost(a,b)
	h = sqrt(sum((a-b).^2,2)); %对差值的每一行求和，然后开放根,
end
