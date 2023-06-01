%% Bidding Protocols for Deploying Mobile Sensors

clear all
close all
clc

% nf = 3; % número de nós
rc = 150; % raio de ocomunicação
% Pos_nodes = 150*rand(2,nf);
% Pos_nodes =  [ 103.3822   67.5812   34.3465; %    103.3822
%                 150.2227   12.5732  137.0006];
Pos_nodes =  [103   67  34 140; %    103.3822
              150   12  137 65];
          
nf = size(Pos_nodes,2); % número de nós          
%% Diagrama de Voronoi


% Identificação dos vizinhos
N = zeros(nf,nf); % cojunto de vizinhos de N
for i=1:nf
    for j = 1:nf
        if(i~=j && sqrt((Pos_nodes(1,i)-Pos_nodes(1,j))^2+(Pos_nodes(2,i)-Pos_nodes(2,j))^2)<rc)
            N(i,j) = 1; % vizinhos            
        end
    end
end


% % Triangulação de Delaunay
% delaunay = [];
% for i=1:nf
%     aux = find(N(i,:)==1);
%     n_vizinhos = length(aux);
%     if(n_vizinhos>=3)
%         for j = 1:factorial(n_vizinhos)/factorial(n_vizinhos-1)
% %             Falta encontrar as combinaçoes de delaunay
%         end
%     end
% end


% Identificação dos vértices
V = []; % referências para as arestas de voronoi (que dará a inclunação das retas perpencidculares)
coeficientes = []; % coeficiente de retas enre vizinhos
visitados = zeros(nf,nf); % nó visitado
info = []; % sequência de nó analizado
info_retas = [];
for i =1:nf
    for j = 1:nf
        if(i~=j && N(i,j)==1 && visitados(i,j)==0 && visitados(j,i)==0)
            V = [V [(Pos_nodes(1,i)+Pos_nodes(1,j))/2; (Pos_nodes(2,i)+Pos_nodes(2,j))/2]]; 
            aux = find(N(i,:)==1);
            aux = [aux i];
            centroide = [sum(Pos_nodes(1,aux))/length(aux); sum(Pos_nodes(2,aux))/length(aux)]; 
            visitados(i,j) = 1;
            visitados(j,i) = 1;
            coeficientes = [coeficientes  (-Pos_nodes(2,i)+Pos_nodes(2,j))/(-Pos_nodes(1,i)+Pos_nodes(1,j))]; % coeficiente angular
%             coeficientes = [coeficientes  (centroide(2)-V(2,end))/(centroide(1)+V(1,end))]; % coeficiente angular
            info_retas = [info_retas; [i,j]];
            cont=0; % verificação do nó contabilizado [talves não seja a melhor forma de fazer]
            if(length(info)==0)
                info =[info i];
            else
                for k =1:length(info)
                    if(info(k)~=i)
                        cont=cont+1;
                    end
                end
                if(cont==length(info))
                    info = [info i]; % indice dos vértices visitados
                else
                    info = [info j]; % indice dos vértices visitados
                end
            end
        end
    end
end

% encontrando as retas perpendiculares (seus coeficientes angulares e lineares)
% coeficiente agular da reta entre dois vizinhos
coeficientes;
retas = [];
for i=1:length(coeficientes)
    m = -1/coeficientes(i);
    retas = [retas [m; V(2,i)-(m*V(1,i))]];
%  retas = [retas [m; centroide(2)-(m*centroide(1))]];
end




min_max=[]; % calculo das interseções de retas para interrupção do plot
for i = 1:size(info_retas,1)
    for j = 1:size(info_retas,1)
        if(i~=j && (retas(1,i)-retas(1,j))~=0)
           xx = (retas(2,j)-retas(2,i)) / (retas(1,i)-retas(1,j));
           yy = (retas(1,i)*xx)+retas(2,i);
           min_max = [min_max; [xx yy]];
        end
   end
end
% remover repetições no min_max
min_max2 = min_max(1,:); % atribuição do primeiro elemento
for i = 1:size(min_max,1)
    cont0 = 0;
    for j = 1:size(min_max2,1)
        if(round(min_max(i,1))~=round(min_max2(j,1)) && round(min_max(i,2))~=round(min_max2(j,2)))
           cont0 = cont0+1; 
        end
    end
    if(cont0==size(min_max2,1))
        min_max2 = [min_max2; min_max(i,:)];
    end
end

cruzamento_triplo = [];
for i = 1:size(min_max2,1)
    cont1=0;
    i
   for j=1:size(retas,2) 
%        if(i~=j)
           round((retas(1,j)*min_max2(i,1))+retas(2,j)-min_max2(i,2))
           if(round((retas(1,j)*min_max2(i,1))+retas(2,j)-min_max2(i,2))==0)
               cont1=cont1+1;    
           end
%        end
   end
   if(cont1>=3)
       cruzamento_triplo = [cruzamento_triplo; min_max2(i,:)];
   end
end






%  Plots
% [X,Y] = meshgrid(1:size(Pos_nodes,1),1:size(Pos_nodes,2));
figure
axis('equal')
hold on

for i=1:nf % plot dos nós
    hold on
    scatter(Pos_nodes(1,i),Pos_nodes(2,i),40,'MarkerEdgeColor','k', 'MarkerFaceColor','k')
    text(Pos_nodes(1,i)+1,Pos_nodes(2,i),char(['N0',int2str(i)]));
end
for i=1:size(V,2)
    scatter(V(1,i),V(2,i),10,'MarkerEdgeColor','r', 'MarkerFaceColor','r')
end


y=[];
ya=[];
for i=1:size(V,2) % plot das retas
    
%     if(V(1,i)<=min_max(i,1))
%         x = 0:1:min_max(i,1);
%     else
%         x = min_max(i,1):1:rc;
%     end
    x = 0:1:rc;
    ya = (coeficientes(i).*x)+ ( ( -coeficientes(i) * Pos_nodes(1,info(i)) ) + Pos_nodes(2,info(i)) );
    y = (retas(1,i).*x)+retas(2,i); 
    plot(x,y,'r')
    plot(x,ya,'c')
    x=[];  y = [];  ya=[];
    scatter(min_max2(:,1),min_max2(:,2),15,'MarkerEdgeColor','g', 'MarkerFaceColor','g')

end

for i=1:size(cruzamento_triplo,1) % plot das retas
    scatter(cruzamento_triplo(i,1),cruzamento_triplo(i,2),15,'MarkerEdgeColor','g', 'MarkerFaceColor','b')
end
% x = 0:1:rc;
% y = (retas(1,2).*x)+retas(2,2); 
% plot(x,y,'r')
% y = (retas(1,4).*x)+retas(2,4); 
% plot(x,y,'r')
% y = (retas(1,5).*x)+retas(2,5); 
% plot(x,y,'r')
% y = (retas(1,3).*x)+retas(2,3); 
% plot(x,y,'r')