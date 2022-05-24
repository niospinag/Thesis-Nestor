function NH = closest(pos, zel, vh);
distancia_x = abs(pos-pos(vh));
distancia_y = abs(zel-zel(vh))*10;
distancia = sqrt(distancia_x.^2 + distancia_y.^2);
distancias = sort(distancia, 'ascend');
NH = [];
for i= 1:size(pos,1)
%     if (zel(vh)- zel(i) <= 1) & (zel(vh)- zel(i) >= -1 )
        if distancia(i) <= distancias(4) & i~= vh
            NH = [NH i];
        end
%     end
end

end
