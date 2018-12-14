function All_Vw = vectors2Vw(vectors)
    All_Vw = [];
    for it=1:size(vectors,2)
        Vw = robotKins(vectors(:,it));
        All_Vw = [All_Vw Vw];
    end
end
