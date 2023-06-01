% plot experimento parciais
clc 
clear all
close all

%%Experimento 1
% load('init_3.mat')
% Pos_i = Pos_n;
% load('exp203.mat');
% Pos_f = Pos_n;
% indice_sink
% numero_movimentos
%     
% ordem_mov = [1 2 3 4];

%% Experimento 2
load('init_1_obst.mat')
Pos_i = Pos_n;
load('exp52.mat');
Pos_f = Pos_n;
indice_sink
numero_movimentos
    
ordem_mov = [1 2 3 4];

%     load('init_1.mat')
%% PARAMETRIZA��O de rede e exibi��o
nm = 4; % N�mero de n�s m�veis
nf = 16;
N = nm+nf; % n�mero total de n�s 
L = 50; % 150 raio de comunica��o 
mapa = imread('mapa.bmp');
vel = [ 0 0 0 ]; % [ vx vy vz ] m/s
wind = [ 0 30 ]; % m�dia e desvio padr�o 
mapa_setup = mapa;
mapa = rgb2gray(mapa); [Ay , Ax] = find(mapa~=255); % carregando o mapa quadrado do ambiente
C = size(mapa,1); % Comprimento da �rea quadrada em estudo
afastamento = C/4;
K = 1; % fator de escala da imagem para melhorar a resolu��o
lo = 0.5; % inicializa��o da grade de ocupa��o
destino = [0;0]; % Inicializa��o do destino/estrela

N = nm+nf; % number of nodes
C = size(mapa,1); % Comprimento da �rea quadrada em estudo
afastamento = C/4;
lo = 0.5; % inicializa��o da grade de ocupa��o
alcance_sensor = L/2;
flag_0=0;


% Atualiza��o em escala das posi��es
C = K*C;
L = K*L;
Pos_n = K*Pos_n;


for i=1:numero_movimentos
    %% Criar e checar os agrupamentos da distribui��o aleat�ria

    %     [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % defini��o dos agrupamentos iniciais
    if(i==1)
        for j=1:nm
            if(ordem_mov(j)==i)
                Pos_i(:,j+nf) = Pos_f(:,j+nf);
            end
        end
        [G,nG,selec] = agrupamentos(L,Pos_i,nf,nm); % defini��o dos agrupamentos iniciais
    elseif(i==2)
        for j=1:nm
            if(ordem_mov(j)==i)
                Pos_i(:,j+nf) = Pos_f(:,j+nf);
            end
        end
        [G,nG,selec] = agrupamentos(L,Pos_i,nf,nm); % defini��o dos agrupamentos iniciais
    elseif(i==3)
        for j=1:nm
            if(ordem_mov(j)==i)
                Pos_i(:,j+nf) = Pos_f(:,j+nf);
            end
        end
        [G,nG,selec] = agrupamentos(L,Pos_i,nf,nm); % defini��o dos agrupamentos iniciais
    else
        for j=1:nm
            if(ordem_mov(j)==i)
                Pos_i(:,j+nf) = Pos_f(:,j+nf);
            end
        end
        [G,nG,selec] = agrupamentos(L,Pos_i,nf,nm); % defini��o dos agrupamentos iniciais
    end
    figure


    % EXIBI��O DOS AGRUPAMENTOS
    M = mapa;
    [c,l,z] = size(M);
    M2 = im2double(M);

    imshow(M2)
    % inicialization of M2
    for l=1:size(M2,1) % Y de cima para baixo
        for c=1:size(M2,2) % X
            M2(l,c,1) = lo;%lo;
            M2(l,c,2) = lo;%lo;
            M2(l,c,3) = lo;%lo;
        end
    end

    % Defini��o do maior grupo
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
       Pos_G = [Pos_G Pos_i(:,G(indice).g(k))];
    end

    % identifica��o do n� sink - M�trica: maior n�mero de vizinhos no
    % agrupamento

    disp(['N� Sink: ', num2str(indice_sink)])
    G(indice).g

    %Exibi��o e classifica��o dos n�s (MAPA COMUM) 
    %     subplot(1,2,2)


    hold on
    NF = scatter(Pos_i(1,1:nf),C-Pos_i(2,1:nf),32,'MarkerEdgeColor','b', 'MarkerFaceColor','b');
%     NS = scatter(Pos_i(1,indice_sink),C-Pos_i(2,indice_sink),32,'MarkerEdgeColor','b', 'MarkerFaceColor','b');
    axis([-10 C+(10) -10 C+(10)])
    NM1 = scatter(Pos_i(1,nf+1),C-Pos_i(2,nf+1),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM2 = scatter(Pos_i(1,nf+2),C-Pos_i(2,nf+2),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM3 = scatter(Pos_i(1,nf+3),C-Pos_i(2,nf+3),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    NM4 = scatter(Pos_i(1,nf+4),C-Pos_i(2,nf+4),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
        % limites da imagem
%         L1 = zeros(1,c);
%         L2 = c.*ones(1,c);
%         L3 = 0:1:c-1;
%         PL1 = plot(L1,L3,'k');
%         PL2 = plot(L2,L3,'k');
%         PL3 = plot(L3,L1,'k');
%         PL4 = plot(L3,L2,'k');


%     legend('Fixed Node','Mobile Node','Frontier','Region of Interest (RoI)')
    legend('Fixed Node','Mobile Node','Frontier and Trajectory','Region of Interest (RoI)')
% Defini��o dos nomes dos n�s (alcance 0-999)
    Nos = [];
    for k=1:N
        if(N<10) % restrito at� 9 n�s
            if(k<10)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        elseif(N>=10 && N<=100) % restrito at� 99 n�s
            if(k<10)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        else % restrito a 999 n�s
            if(k<10)
                Nos = [Nos; ['00',int2str(k)]];
            elseif(k>=10 && k<100)
                Nos = [Nos; ['0',int2str(k)]];
            else
                Nos = [Nos; ['',int2str(k)]];
            end
        end
        text(Pos_i(1,k)+3,C-Pos_i(2,k)-4,char(Nos(k,:)));
        if(k==indice_sink)
           text(Pos_i(1,k)+K,C-Pos_i(2,k)+(10*K),'sink');
        end
    end

    % Plot n�s m�veis e n�s fixos (c�rculos para representar os n�s)
    for k = 1:1:l % linha 
      for j = 0:1:c-1 % coluna   
        for p = 1:1:length(Pos_i)% plot n� vermelho (m�vel) e azul (fixo)
            if(sqrt((Pos_i(2,p)-(j+1))^2+((Pos_i(1,p)-(k))^2))<=K+1 && p<=nf)
               M2(C-j,k,1) = 0.0;
               M2(C-j,k,2) = 0.0;
               M2(C-j,k,3) = 255.0;
           elseif(sqrt((Pos_i(2,p)-(j+1))^2+((Pos_i(1,p)-(k))^2))<=K+1) 
               M2(C-j,k,1) = 255.0;
               M2(C-j,k,2) = 0.0;
               M2(C-j,k,3) = 0.0;
           end
           if(p==indice_sink && sqrt((Pos_i(2,p)-(j+1))^2+((Pos_i(1,p)-(k))^2))<=K+1)
               M2(C-j,k,1) = 0.0;
               M2(C-j,k,2) = 255.0;
               M2(C-j,k,3) = 255.0;
           end    
        end
      end
    end
    % Plot �rea de interesse
    afastamento = C/4;

    X1 = afastamento:1:C-afastamento-1;
    Y1 = X1;
    X2 = afastamento*ones(1,C-(2*afastamento));
    Y2 = X2;
    plot(X1,3*Y2,':k')
    plot(X1,Y2,':k')
    plot(X2,Y1,':k')
    plot(3*X2,Y1,':k')
    text(afastamento,C-(3*afastamento)-10,'RoI');

    [M2,fronteira] = occupancy_grid_mapping_fronteira(M2,L,Pos_G,lo);
    fronteira = ordenar_fronteira3(fronteira);

    [G,nG,selec] = agrupamentos(L,Pos_i,nf,nm); % defini��o dos agrupamentos iniciais
    maior = 0;
    indice = 0;
    for p = 1:1:nG
        if(maior<(length(G(p).g)-1))
            maior = length(G(p).g)-1;
            indice = p;
            network_nodes = maior;
        end
    end
    scatter(fronteira(1,:),fronteira(2,:),1,'MarkerEdgeColor','k', 'MarkerFaceColor','k');
    xlabel('x (cm)')
    ylabel('y (cm)')
    axis
    hold on
    scatter(mobile_node(ordem_mov(i)).P(1,1),C-(mobile_node(ordem_mov(i)).P(2,1)),32,'MarkerEdgeColor','r', 'MarkerFaceColor','w');
    plot(mobile_node(ordem_mov(i)).P(1,:),C-mobile_node(ordem_mov(i)).P(2,:),'.k');
    scatter(mobile_node(ordem_mov(i)).P(1,end),C-mobile_node(ordem_mov(i)).P(2,end),32,'MarkerEdgeColor','r', 'MarkerFaceColor','r');
end

