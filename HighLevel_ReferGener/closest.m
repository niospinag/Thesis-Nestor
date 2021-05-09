function NH = closest(pos, zel, vh);
distancia_x = pos-pos(vh);
distancia_y = abs(zel-zel(vh));
distancia = sqrt(distancia_x.^2 + distancia_y.^2);
distancias = sort(distancia, 'ascend');
NH = [];
% j= ;
for j = 2:5
    for i= 1:size(pos,1)
        if distancias(j) == distancia(i) && i~=vh 
            NH = [NH i];
        end
    end
end

end
 