% plot experimento
clc 
clear all
close all

q = 0;  % 1 - inicical e 0 - final
if(q)
    load('init_22.mat');
else
    load('exp203.mat');
end


if(q)
    % plot inicial
%     load('init_1.mat')
    %% PARAMETRIZAÇÃO de rede e exibição
    nm = 4; % Número de nós móveis
    nf = 16;
    N = nm+nf; % número total de nós 
    L = 50; % 150 raio de comunicação 
    mapa = imread('mapa.bmp');
    vel = [ 0 0 0 ]; % [ vx vy vz ] m/s
    wind = [ 0 30 ]; % média e desvio padrão 
    mapa_setup = mapa;
    mapa = rgb2gray(mapa); [Ay , Ax] = find(mapa~=255); % carregando o mapa quadrado do ambiente
    C = size(mapa,1); % Comprimento da área quadrada em estudo
    afastamento = C/4;
    K = 1; % fator de escala da imagem para melhorar a resolução
    lo = 0.5; % inicialização da grade de ocupação
    destino = [0;0]; % Inicialização do destino/estrela

    N = nm+nf; % number of nodes
    C = size(mapa,1); % Comprimento da área quadrada em estudo
    afastamento = C/4;
    lo = 0.5; % inicialização da grade de ocupação
    alcance_sensor = L/2;
    flag_0=0;


    % Atualização em escala das posições
    C = K*C;
    L = K*L;
    Pos_n = K*Pos_n;



    %% Criar e checar os agrupamentos da distribuição aleatória

%     [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % definição dos agrupamentos iniciais
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,0); % definição dos agrupamentos iniciais


    % EXIBIÇÃO DOS AGRUPAMENTOS
    M = mapa;
    [c,l,z] = size(M);
    M2 = im2double(M);


    % inicialization of M2
    for l=1:size(M2,1) % Y de cima para baixo
        for c=1:size(M2,2) % X
            M2(l,c,1) = lo;%lo;
            M2(l,c,2) = lo;%lo;
            M2(l,c,3) = lo;%lo;
        end
    end

    % Definição do maior grupo
    maior = 0;
    indice = 0;
    for k=1:1:nf
        if(maior<(length(G(k).g)-1))
            maior = length(G(k).g)-1;
            indice = k;
        end
    end
    network_nodes = maior;

    % plota fronteira;
    Pos_G = [];
    for k = 2:maior+1
        if(G(indice).g(k)<=nf) % remove nós móveis da fronteira e do voronoi
            Pos_G = [Pos_G Pos_n(:,G(indice).g(k))];
        end
    end

    % identificação do nó sink - Métrica: maior número de vizinhos no
    % agrupamento
    indice_sink = 2;
    cont = zeros(1,length(G(indice).g));
    for k=2:length(G(indice).g)
        for j=2:length(G(indice).g)
            if(k~=j && sqrt((Pos_n(2,G(indice).g(k)) - Pos_n(2,G(indice).g(j)))^2+((Pos_n(1,G(indice).g(k)) - Pos_n(1,G(indice).g(j)))^2))<L)
                cont(k) = cont(k)+1;
            end
        end
    end
    if(length(cont)>2)
        [maxx,indice_sink] = max(cont);
    else
        maxx=0;
        indice_sink = 2;
    end
    % //////////////////////////////////////////////////////
    indice_sink = G(indice).g(indice_sink);

    
    disp(['Nó Sink: ', num2str(indice_sink)])
    G(indice).g

    %Exibição e classificação dos nós (MAPA COMUM) 
%     subplot(1,2,2)
    hold on
%     plot(Pos_n(1,1:nf),Pos_n(2,1:nf),'db');  % plot da distribuição nós móveis
    NF = scatter(Pos_n(1,1:nf),Pos_n(2,1:nf),32,'MarkerEdgeColor','b', 'MarkerFaceColor','b');
    NS = scatter(Pos_n(1,indice_sink),Pos_n(2,indice_sink),32,'MarkerEdgeColor','c', 'MarkerFaceColor','c');
    axis([-10 C+(10) -10 C+(10)])
%     plot(Pos_n(1,nf+1:end),Pos_n(2,nf+1:end),'sr');  % plot da distribuição
    NM1 = scatter(Pos_n(1,nf+1),Pos_n(2,nf+1),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM2 = scatter(Pos_n(1,nf+2),Pos_n(2,nf+2),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM3 = scatter(Pos_n(1,nf+3),Pos_n(2,nf+3),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM4 = scatter(Pos_n(1,nf+4),Pos_n(2,nf+4),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
%     % limites da imagem
%     L1 = zeros(1,c);
%     L2 = c.*ones(1,c);
%     L3 = 0:1:c-1;
%     PL1 = plot(L1,L3,'k');
%     PL2 = plot(L2,L3,'k');
%     PL3 = plot(L3,L1,'k');
%     PL4 = plot(L3,L2,'k');

%     legend('Nó fixo','Nó móvel','Fronteira','Área de Interesse','Location','southoutside')
    legend('Fixed Node','Mobile Node','Frontier','Region of Interest (RoI)')
    % Definição dos nomes dos nós (alcance 0-999)
    Nos = [];
    for k=1:N
        if(N<10) % restrito até 9 nós
            if(k<10)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        elseif(N>=10 && N<=100) % restrito até 99 nós
            if(k<10)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        else % restrito a 999 nós
            if(k<10)
                Nos = [Nos; ['00',int2str(k)]];
            elseif(k>=10 && k<100)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        end
        text(Pos_n(1,k)+3,Pos_n(2,k)-4,char(Nos(k,:)));
        if(k==indice_sink)
           text(Pos_n(1,k)+K,Pos_n(2,k)-(15*K),'sink');
        end
    end


    Pos_G = []; % Aloca os nó móveis e dentro do maior agrupamento para a definição de nova fronteira
    Pos_G1 = [];
    for k = 1:length(G(indice).g)-1
        if(G(indice).g(k+1)<=nf)
            Pos_G1 = [Pos_G1 Pos_n(:,G(indice).g(k+1))];
        end
        Pos_G = [Pos_G Pos_n(:,G(indice).g(k+1))];
    end
    
    % VORONOI
    hold on
    voronoi(Pos_G(1,:),Pos_G(2,:));

    [M2,bid,cobertura] = voronoi_w2(M,Pos_G,nf,nm,Pos_G1,afastamento,0); % VORONOI


    % Plot nós móveis e nós fixos (círculos para representar os nós)
    for k = 1:1:l % linha 
      for j = 0:1:c-1 % coluna   
        for p = 1:1:length(Pos_n)% plot nó vermelho (móvel) e azul (fixo)
            if(sqrt((Pos_n(2,p)-(j+1))^2+((Pos_n(1,p)-(k))^2))<=K+1 && p<=nf)
               M2(C-j,k,1) = 0.0;
               M2(C-j,k,2) = 0.0;
               M2(C-j,k,3) = 255.0;
           elseif(sqrt((Pos_n(2,p)-(j+1))^2+((Pos_n(1,p)-(k))^2))<=K+1) 
               M2(C-j,k,1) = 255.0;
               M2(C-j,k,2) = 0.0;
               M2(C-j,k,3) = 0.0;
           end
           if(p==indice_sink && sqrt((Pos_n(2,p)-(j+1))^2+((Pos_n(1,p)-(k))^2))<=K+1)
               M2(C-j,k,1) = 0.0;
               M2(C-j,k,2) = 255.0;
               M2(C-j,k,3) = 255.0;
           end    
        end
      end
    end
    % Plot área de interesse
    afastamento = C/4;
    hold on
    X1 = afastamento:1:C-afastamento-1;
    Y1 = X1;
    X2 = afastamento*ones(1,C-(2*afastamento));
    Y2 = X2;
    plot(X1,3*Y2,':k')
    plot(X1,Y2,':k')
    plot(X2,Y1,':k')
    plot(3*X2,Y1,':k')
    text(afastamento,(3*afastamento)+10,'RoI');

    [M2,fronteira] = occupancy_grid_mapping_fronteira(M2,L,Pos_G,lo);
%     fronteira = ordenar_fronteira3(fronteira);
 
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,0); % definição dos agrupamentos iniciais
    maior = 0;
    indice = 0;
    for i = 1:1:nG
        if(maior<(length(G(i).g)-1))
            maior = length(G(i).g)-1;
            indice = i;
            network_nodes = maior;
        end
    end
    scatter(fronteira(1,:),C-fronteira(2,:),1,'MarkerEdgeColor','k', 'MarkerFaceColor','k');
    xlabel('x')
    ylabel('y')

else
    
   
    % plot final
%     load('exp1.mat')
        %% PARAMETRIZAÇÃO de rede e exibição
 
%     mapa_setup = mapa;
%     mapa = rgb2gray(mapa); [Ay , Ax] = find(mapa~=255); % carregando o mapa quadrado do ambiente
    alcance_sensor = L/2;
    flag_0=0;



    %% Criar e checar os agrupamentos da distribuição aleatória

%     [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % definição dos agrupamentos iniciais
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % definição dos agrupamentos iniciais


    % EXIBIÇÃO DOS AGRUPAMENTOS
    M = mapa;
    [c,l,z] = size(M);
    M2 = im2double(M);


    % inicialization of M2
    for l=1:size(M2,1) % Y de cima para baixo
        for c=1:size(M2,2) % X
            M2(l,c,1) = lo;%lo;
            M2(l,c,2) = lo;%lo;
            M2(l,c,3) = lo;%lo;
        end
    end

    % Definição do maior grupo
    maior = 0;
    indice = 0;
    for k=1:1:nf
        if(maior<(length(G(k).g)-1))
            maior = length(G(k).g)-1;
            indice = k;
        end
    end
    network_nodes = maior;

     % plota fronteira;
    Pos_G = [];
    for i = 2:network_nodes+1
%        if(G(i)<=nf || (G(i)>nf && bid(G(i))>0)) % remove nós móveis da fronteira e do voronoi     
%        if(G(indice).g(i)<=nf || (G(indice).g(i)>nf && bid(G(indice).g(i))>0)) % remove nós móveis da fronteira e do voronoi
            Pos_G = [Pos_G Pos_n(:,G(indice).g(i))];
%        end
    end
    
    
    
    % identificação do nó sink - Métrica: maior número de vizinhos no
    % agrupamento
    indice_sink = ceil(length(G(indice).g)/2);
    cont = zeros(1,length(G(indice).g));
    for k=2:length(G(indice).g)
        for j=2:length(G(indice).g)
            if(k~=j && G(indice).g(k)<=nf && G(indice).g(j)<=nf && sqrt((Pos_n(2,G(indice).g(k)) - Pos_n(2,G(indice).g(j)))^2+((Pos_n(1,G(indice).g(k)) - Pos_n(1,G(indice).g(j)))^2))<=L)
                cont(k) = cont(k)+1;
            end
        end
    end
    if(length(cont)>2)
        [maxx,indice_sink] = max(cont);
    else
        maxx=0;
        indice_sink = ceil(length(G(indice).g)/2);
    end

    indice_sink = G(indice).g(indice_sink);

    disp(['Nó Sink: ', num2str(indice_sink)])
    G(indice).g

    %Exibição e classificação dos nós (MAPA COMUM) 
%     subplot(1,2,2)
    hold on
%     plot(Pos_n(1,1:nf),Pos_n(2,1:nf),'db');  % plot da distribuição nós móveis
    NF = scatter(Pos_n(1,1:nf),Pos_n(2,1:nf),32,'MarkerEdgeColor','b', 'MarkerFaceColor','b');
    NS = scatter(Pos_n(1,indice_sink),Pos_n(2,indice_sink),32,'MarkerEdgeColor','c', 'MarkerFaceColor','c');
    axis([-10 C+(10) -10 C+(10)])
%     plot(Pos_n(1,nf+1:end),Pos_n(2,nf+1:end),'sr');  % plot da distribuição
    NM1 = scatter(Pos_n(1,nf+1),Pos_n(2,nf+1),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM2 = scatter(Pos_n(1,nf+2),Pos_n(2,nf+2),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM3 = scatter(Pos_n(1,nf+3),Pos_n(2,nf+3),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM4 = scatter(Pos_n(1,nf+4),Pos_n(2,nf+4),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
%     % limites da imagem
%     L1 = zeros(1,c);
%     L2 = c.*ones(1,c);
%     L3 = 0:1:c-1;
%     PL1 = plot(L1,L3,'k');
%     PL2 = plot(L2,L3,'k');
%     PL3 = plot(L3,L1,'k');
%     PL4 = plot(L3,L2,'k');

%     legend('Nó fixo','Nó móvel','Fronteira','Área de Interesse','Location','southoutside')
    legend('Fixed Node','Mobile Node','Frontier','Region of Interest (RoI)')
    % Definição dos nomes dos nós (alcance 0-999)
    Nos = [];
    for k=1:N
        if(N<10) % restrito até 9 nós
            if(k<10)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        elseif(N>=10 && N<=100) % restrito até 99 nós
            if(k<10)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        else % restrito a 999 nós
            if(k<10)
                Nos = [Nos; ['00',int2str(k)]];
            elseif(k>=10 && k<100)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        end
        text(Pos_n(1,k)+3,Pos_n(2,k)-4,char(Nos(k,:)));
        if(k==indice_sink)
           text(Pos_n(1,k)+K,Pos_n(2,k)-(15*K),'sink');
        end
    end


    Pos_G = []; % Aloca os nó móveis e dentro do maior agrupamento para a definição de nova fronteira
    for k = 1:length(G(indice).g)-1
        Pos_G = [Pos_G Pos_n(:,G(indice).g(k+1))];
    end


    % Plot área de interesse
    afastamento = C/4;

    X1 = afastamento:1:C-afastamento-1;
    Y1 = X1;
    X2 = afastamento*ones(1,C-(2*afastamento));
    Y2 = X2;
    plot(X1,3*Y2,':k')
    plot(X1,Y2,':k')
    plot(X2,Y1,':k')
    plot(3*X2,Y1,':k')
    text(afastamento,(3*afastamento)+10,'RoI');

%     [M2,fronteira] = occupancy_grid_mapping_fronteira(M2,L,Pos_G,lo);
%     fronteira = ordenar_fronteira3(fronteira);

  [Mx,fronteira] = occupancy_grid_mapping_fronteira(M2,L,Pos_G,lo);
  
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,0); % definição dos agrupamentos iniciais
    maior = 0;
    indice = 0;
    for i = 1:1:nG
        if(maior<(length(G(i).g)-1))
            maior = length(G(i).g)-1;
            indice = i;
            network_nodes = maior;
        end
    end
    scatter(fronteira(1,:),C-fronteira(2,:),1,'MarkerEdgeColor','k', 'MarkerFaceColor','k');
    xlabel('x')
    ylabel('y')
    hold on
    voronoi(Pos_G(1,:),Pos_G(2,:));
    hold off

    hold on
    for a=1:nm
        scatter(mobile_node(a).P(1,1),mobile_node(a).P(2,1),32,'MarkerEdgeColor','r', 'MarkerFaceColor','w');
        plot(mobile_node(a).P(1,:),mobile_node(a).P(2,:),'.k');
    end
    
    
    
    
    
    
%     subplot(1,2,1) 
    
%     [M2,bid,cobertura] = voronoi_w2(M,Pos_G,nf,nm,Pos_G1,afastamento,0); % VORONOI
    

end

