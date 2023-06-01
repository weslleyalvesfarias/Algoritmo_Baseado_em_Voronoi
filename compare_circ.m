function [Points] =  compare_circ(P,visitados,xc,yc) 
% 
% Pos =  [103  67  34  140; %    103.3822
%         150  12  137  65];
%     
%  P = [1 3];  
%  P = [1 3 4 2];
    xc = round(xc); % arredondando x para facilitar o cálculo
    yc = round(yc); % arredondando x para facilitar o cálculo
%     visitados = [];
%     if(length(visitados)>=1)
%        for k =1:length(P)
%            
%        end
%     end
    vizinhos = [];
    if(length(P)==1 && length(visitados)==0)
        visitados = [visitados P(1)];
        vizinhos = [vizinhos; P(1) 0 0];
    elseif(length(P)==2 && length(visitados)==1)
        visitados = [visitados P(2)];
        xx = find(abs(xc(1,:)-xc(2,:))<1);
        yy = find(abs(yc(1,:)-yc(2,:))<1);
        if(length(xx)==1 && length(yy)==1)
            vizinhos = [vizinhos; P(1) P(2) 0 xc(xx) yc(yy)];
        elseif(length(xx)==2 && length(yy)==2)
            vizinhos = [vizinhos; P(1) P(2) 0 mean(xc(xx(:))) mean(yc(yy(:)))];
        end
    else % Para mais de 3 nós
        for i=1:length(P)
            for j = 1:length(P)
                if(i~=j)
                    if(i==1)
                        visitados = [visitados i j];
                     
                    elseif(i>1 && length(find(visitados==j))==0)
                        visitados =[visitados j];
                        xx = find(abs(xc(i,:)-xc(j,:))<1);
                        yy = find(abs(yc(i,:)-yc(j,:))<1);
                        izinhos = [vizinhos; i j 0];

                    end
                end
            end
        end
    end
end