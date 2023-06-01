% voronoi Matlab
clc
clear all
close all 

%% Inputs
nf = 16;
% mapa;
% Pos % posição dos nós sensores
% [l,c] = size(mapa);
figure()
Pos = round(300*rand(16,2));
% X = [-1.5 3.2; 1.8 3.3; -3.7 1.5; -1.5 1.3; ...
%       0.8 1.2; 3.3 1.5; -4.0 -1.0;-2.3 -0.7; ...
%       0 -0.5; 2.0 -1.5; 3.7 -0.8; -3.5 -2.9; ...
%      -0.9 -3.9; 2.0 -3.5; 3.5 -2.25];
%  
[XV,YV] = voronoi(Pos(:,1),Pos(:,2));
voronoi(Pos(:,1),Pos(:,2)); %voronoi plot of matlab 
% Assign labels to the points.
nump = size(Pos,1);
plabels = arrayfun(@(n) {sprintf('X%d', n)}, (1:nump)');
hold on
scatter(Pos(:,1), Pos(:,2),40,'MarkerEdgeColor','r', 'MarkerFaceColor','r')
Hpl = text(Pos(:,1)+10, Pos(:,2), plabels, 'FontWeight', ...
      'bold', 'HorizontalAlignment','center', ...
      'BackgroundColor', 'none');
% Exibição do raio de comunicação  
Rc = 50; % raio de comunicação
theta = 0:0.1:2*pi; 
hold on
for i=1:size(Pos,1)
    xref = Pos(i,1) + ( Rc*cos(theta) );
    yref = Pos(i,2) + ( Rc*sin(theta) );
    plot(xref,yref,'r')
end
axis([0 300 0 300])
% for i=1:size(XV,2)
%     plot(XV(1,i),XV(2,i),'*r');
% end
% hold off
  


% Modelo WAF de estração do diagrama de Voronoi baseado em sua definição
M = zeros(300,300,3);
for i = 1:size(M,1)
    for j = 1:size(M,2)
        dmin = 300*300;
        for k = 1:nf
            dist = sqrt((Pos(k,1)-i)^2+(Pos(k,2)-j)^2);
            if(dist<=dmin)
                dmin=dist;
                n = k;
                x1 = size(M,1)+1-j;
                y1 = i;
            end
        end
        M(x1,y1,2) = n*(1/nf);
    end
end
% Tamanho de cada área
tam_vor_no = [];
for i=1:nf
    tam_vor_no = [tam_vor_no; length(find(M(:,:,2)==i/nf))];
end
tam_vor_no
% Preenchimento cobertura
for k=1:nf
    for i = 1:size(M,1) % line
        for j = 1:size(M,2) % colun
            dist_pixel = sqrt((Pos(k,1)-i)^2+(Pos(k,2)-j)^2);
            if(dist_pixel<=Rc)
                M(size(M,2)+1-j,i,1)=0.2;
                M(size(M,1)+1-j,i,3)=0.2;
            end
        end
    end
end
% Tamanho do buraco de cada área
tam_buraco = [];
for i=1:nf
    tam_buraco = [tam_buraco; length( find(M(:,:,2)==i/nf & M(:,:,1)~=0.2 ) )];
end
tam_buraco


figure
imshow(M);
nump = size(Pos,1);
plabels = arrayfun(@(n) {sprintf('N%d', n)}, (1:nump)');
hold on
% plot nodes
scatter(Pos(:,1), size(M,1)-Pos(:,2),40,'MarkerEdgeColor','r', 'MarkerFaceColor','r')
% plot ID of nodes
Hpl = text(Pos(:,1)+15, size(M,1)-Pos(:,2), plabels, 'FontWeight', ...
      'bold', 'HorizontalAlignment','center', ...
      'BackgroundColor', 'none');
% Exibição do raio de comunicação  
Rc = 50; % raio de comunicação
theta = 0:0.1:2*pi; 

% for i=1:size(Pos,1)
%     xref = Pos(i,1) + ( Rc*cos(theta) ); xref = round(xref);
%     yref = Pos(i,2) + ( Rc*sin(theta) ); yref = round(yref);
%     
%     plot(xref,yref,'r')
% end  
hold off    
  
  