function [RRTree1,RRTree2] = Swap(RRTreeA,RRTreeB)
    LA = length(RRTreeA);
    LB = length(RRTreeB);
    if  LB < LA
        RRTree1 = RRTreeB; 
        RRTree2 = RRTreeA;
    else
        RRTree1 = RRTreeA; 
        RRTree2 = RRTreeB;
    end
end
