function [cobertura] = cobertura(M2)
    
    [l,c] = size(M2);
    total = l*c;
    
    area_de_interesse = l*c;
    
%             Gree informa o voronoi
%             Red e Blue informa se é buraco
    [x,y] = find(M2(:,:,2)~=0 & M2(:,:,1)==0 & M2(:,:,3)==0);
    cobertura = (length(x)/area_de_interesse)*100; 

end