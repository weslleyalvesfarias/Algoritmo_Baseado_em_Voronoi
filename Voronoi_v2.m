% voronoi v2

%% Bidding Protocols for Deploying Mobile Sensors

clear all
close all
clc

global yref FA FB;
% nf = 3; % número de nós
rc = 150; % raio de ocomunicação
% Pos_nodes = 150*rand(2,nf);
% Pos_nodes =  [ 103.3822   67.5812   34.3465; %    103.3822
%                 150.2227   12.5732  137.0006];
Pos =  [103  67  34  140; %    103.3822
        150  12  137  65];
          
nf = size(Pos,2); % número de nós   




% Pos -> positions of nodes
% range - alcance de comunicação

yMin = min(Pos(2,:)-20);
yMax = max(Pos(2,:)+20);

xMin = min(Pos(1,:)-20);
xMax = max(Pos(1,:)+20);


x1 = xMax+abs(xMin);
y1 = yMax+abs(yMin);


%% Plot
figure
hold on
x = xMin:1:xMax; % linha de varredura [x,yref]
yref = yMax*ones(1,length(x));
axis([xMin xMax yMin yMax])
FA = plot(x,yref,'.r'); % objeto para exibir o plot da linha dinamicamente
thetac = 0:(2*pi)/20:2*pi;
xc = ones(nf,length(thetac));
yc = ones(nf,length(thetac));
FB = plot(xc(1,:),yc(1,:),'-.r');
for i=1:nf % Exibição dos nós com nomeação sequencial [1...nf]
    scatter(Pos(1,i),Pos(2,i),40,'MarkerEdgeColor','k', 'MarkerFaceColor','k')
    text(Pos(1,i)+3,Pos(2,i)+1,char(['N0',int2str(i)])); % Nomeação dos nos
end



retas = [];
V = []; % referências para as arestas de voronoi (que dará a inclunação das retas perpencidculares)
coeficientes = []; % coeficiente de retas enre vizinhos
P = []; % pontos encontrados
while(yref(1)>=yMin)  
    set(FA,'Xdata',x,'Ydata',yref);
    
    
%     detecção das linhas
    flag_novo_ponto = 0;
    for i=1:nf
       if(yref==Pos(2,i))
           P = [P i];
           scatter(Pos(1,i),Pos(2,i),40,'MarkerEdgeColor','r', 'MarkerFaceColor','r')
        flag_novo_ponto = 1;
       end
    end
    for k=1:1:length(P) % Plot funcionando
        dist = sqrt((Pos(2,P(k))-(yref(1)))^2);
        xc(k,:) = Pos(1,P(k)) + dist*(ones(1,length(thetac)).*cos(thetac));
        yc(k,:) = Pos(2,P(k)) + dist*(ones(1,length(thetac)).*sin(thetac));
%         plot(xc,yc,'-.r');
        hold on
        set(FB,'Xdata',xc(k,:),'Ydata',yc(k,:)); 
    end
%     hold off
%     if(length(P)>1)
%%        Remover ponto muito afastado da linha
        i=1;
        auxP = length(P);
        while(i<=auxP)
            i
           if((Pos(2,P(i))-yref)>rc)
               scatter(Pos(1,P(i)),Pos(2,P(i)),40,'MarkerEdgeColor','k', 'MarkerFaceColor','k')
               P(i)=[];
               auxP = length(P);
           end
               i=i+1;
        end
%% verificar vizinho para encontrar ponto de referência
    if(flag_novo_ponto)
       % XXXXXXXX
       flag_novo_ponto=0;
       if(length(P)==2) % para dois pontos
           % Cálculo da reta bissetriz entre esses pontos
            V = [V [(Pos(1,P(1))+Pos(1,P(2)))/2; (Pos(2,P(1))+Pos(2,P(2)))/2]]; 
            coeficientes = [coeficientes  (-Pos(2,P(1))+Pos(2,P(2)))/(-Pos(1,P(1))+Pos(1,P(2)))]; % coeficiente angular
            m = -1/coeficientes;
            retas = [retas [m; V(2,1)-(m*V(1,1))]];
            % PLOT
            x = xMin:1:xMax;
            y = (retas(1,1).*x)+retas(2,1); 
            plot(x,y,'r');
       end
       if(length(P)>2) % para mais de dois pontos
%%                Encontrar as triangulações possíveis
            for j=1:length(P)

            end
           % Cálculo da reta bissetriz entre esses pontos

       end
    end
    
%     end
    
    
    
    yref = yref-1; % atualização do passo de varredura;
    pause(0.03)
    
end
set(FA,'Xdata',[],'Ydata',[]); % Removendo a linha no fim e deixando nós em preto
for i=1:nf % Exibição dos nós com nomeação sequencial [1...nf]
    scatter(Pos(1,i),Pos(2,i),40,'MarkerEdgeColor','k', 'MarkerFaceColor','k')
end