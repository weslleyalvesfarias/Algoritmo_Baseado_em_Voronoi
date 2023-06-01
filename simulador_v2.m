%% COMENTÁRIOS

% Desvio de obstáculos do outro algoritmo
% Métricas
% Ver detalhe da relação do algoritmo de atribuição de tarefa com base na proximidade 
% Cobertura
% avaliar o buraco deivado com a movimentação requerida

function simulador_v2()
%%
clc
clear all
close all

disp('#################################')
disp('           INICIALIZOU')
disp('#################################')


%% inicialzação das variáveis globais

global mapa i angs tamos K L RREP RREQ RERR hop_cont afastamento lo;        % variáveis para robô e função de controle_e_navegação

%% Taxonomia da Rede Híbrida

%% PARAMETRIZAÇÃO de rede e exibição
nm = 4; % Número de nós móveis
nf = 16;
N = nm+nf; % número total de nós 
L = 50; % raio de comunicação 
mapa = imread('mapa.bmp'); 
% mapa = rgb2gray(mapa); 
[Ay , Ax] = find(mapa~=255); % carregando o mapa quadrado do ambiente
C = size(mapa,1); % Comprimento da área quadrada em estudo
afastamento = C/4;
K = 1; % fator de escala da imagem para melhorar a resolução
lo = 0.5; % inicialização da grade de ocupação
destino = [0;0]; % Inicialização do destino/estrela

tic
[  Pos_n , Pos_G , M , bid , G , indice , indice_sink  ] = inicializacao2(nm,nf);
[c,l,z] = size(M);
M2 = im2double(M);

%% CARACTERÍSTICAS DO EXPERIMENTO (dinâmica de simulação e variáveis de suporte)
     
tamos = 0.25; % sampling period of simulation [s]
tempo(1) = 0;  % time control 
i = 0;  % contador

% Comunicação AODV frames
RREP = zeros(nm,7); %envia RREP para o nó S (7 parametros na mensagem)
RREQ = zeros(nm,7); %envia RREQ para o nó S
RERR = zeros(nm,7); %envia RERR para vixinhos
hop_cont = 0;

finalizou = 0; % flag para armazenar os individuos que finalizou a tarefa pois estão numa condição de não mobilidade
def_no = 1; % aponta definição de novo destino e nó
numero_movimentos = 0; % contabiliza o número de rounds do algoritmo

%% Inicialização dos robôs e criação de uma estrutura de dados para eles
% 
% Posições iniciais e de destino para os nós móveis
for k=1:nm 
    Pi = [ Pos_n(1,nf+k) ; Pos_n(2,nf+k) ; 0 ]; % initial position of mobile nodes
    Pd = [ Pos_n(1,nf+k) ; Pos_n(2,nf+k) ]; % final position    
    % prealocação de memória para plot de cada robô
    Vel = [ 0; 0 ]; % speedd command [ V ; W ]; 
    com = 0; % communication flag
    bid_mobile = 0; % preço para movimentação do nó 
    % Saves and creates data struture named "mobile_node" 
    mobile_node(k) = struct('P',Pi,'Vel',Vel,'Pd',Pd,'com',com,'bid_mobile',bid_mobile);
end


%% Critério de parada
% =========================================================================
bid_mob = [mobile_node(1:nm).bid_mobile];
if( (def_no ==1) && ( max(bid(:,1))<=min(bid_mob(:)) || sum(bid(:,1))==0 || length(G(indice).g)-1==N ) )
    finalizou = 1;
    saveas(gcf,char('finish'),'png');
    saveas(gcf,char('finish'),'eps');
end

robo_em_mov = 0;
new_bid = 0; % contador de tarefas atribuídas
while  (~finalizou)
    % actualization of the time control variables
    i = i+1;     
    tempo = [tempo tempo(end)+tamos]; % time of global system    
    
    
%     CHECAR SE HÁ BURACOS E ATRIBUI AO NÓ MAIS PRÓXIMO SE I==1 CASO
%     CONTRÁRIO DEFINE O DE MENOR BID_M
    if(def_no)
        numero_movimentos = numero_movimentos + 1;
        [M2,bid,cobertura] = voronoi_w2(M2,Pos_G,nf,afastamento); % VORONOI
        
        [~,bid_request_ind] = max(bid(:,1));
        id_node_bid = G(indice).g(1+bid_request_ind); % nó requerente
        
        %#################################################################################
        % seleção do robô baseado no menor bid
        menor_bid = l*c;
        for k=1:nm
            if(mobile_node(k).bid_mobile<menor_bid)
                menor_bid = mobile_node(k).bid_mobile;
                robo_em_mov = k;
            end
        end
        %#################################################################################

        % destino
        v = [-Pos_n(1,id_node_bid)+bid(bid_request_ind,2); -Pos_n(2,id_node_bid)+(bid(bid_request_ind,3))];
        v = v / sqrt(v(1)^2+v(2)^2);
        destino = ( ( sqrt(3) * (L/2) ) * v ) + Pos_n(:,id_node_bid)  % Obedecendo o destino fora da área de sensoriamento
    %     theta = atan2(Pos_n(2,id_node_bid)-(bid(bid_request_ind,3)),Pos_n(1,id_node_bid)-bid(bid_request_ind,2));
    %     destino = Pos_n(:,id_node_bid) + [L*cos(theta) ; L*sin(theta)];
        
%         % destino_2
%         vizinhos = Pos_n(:,id_node_bid);
%         for x = 1:length(Pos_G) % Identificando nós próximos
%             if(G(indice).g(1+x) ~= id_node_bid)
%                 dist = sqrt((Pos_n(1,id_node_bid)-Pos_G(1,x))^2+(Pos_n(2,id_node_bid)-Pos_G(2,x))^2);
%                 if(dist<=1.5*L)
%                     vizinhos = [vizinhos Pos_G(:,x)];
%                 end
%             end
%         end
%         [XV,YV] = voronoi(vizinhos(1,:),vizinhos(2,:)); % Identificando vértices próximos
%         hold on
%         subplot(1,2,2)
%         scatter(XV(:),YV(:),20,'MarkerEdgeColor','g', 'MarkerFaceColor','g')
%         mais_distante = 0;
%         vertice_mais_dist = [0 ; 0];
%         for x=1:length(XV) % Identificando o vértice mais distante      
%             dist = sqrt((Pos_n(1,id_node_bid)-XV(x))^2+(Pos_n(2,id_node_bid)-YV(x))^2);
%             if(dist>mais_distante)
%                 mais_distante = dist;
%                 vertice_mais_dist = [XV(x); YV(x)];
%             end  
%         end
%         v = [-Pos_n(1,id_node_bid)+vertice_mais_dist(1); -Pos_n(2,id_node_bid)+vertice_mais_dist(2)];
%         v = v / sqrt(v(1)^2+v(2)^2);
%         destino = ( ( sqrt(3) * (L/2) ) * v ) + Pos_n(:,id_node_bid)  % Obedecendo o destino fora da área de sensoriamento
%         
        %#################################################################################
        % seleção do robô baseada na menor distância
%         menor_dist = l*c;
%         for k=1:nm
%             distancia = sqrt((mobile_node(k).P(1,end)-destino(1))^2+(mobile_node(k).P(2,end)-destino(2))^2);
%             if(distancia<menor_dist)
%                 menor_dist = distancia;
%                 robo_em_mov = k;
%             end
%         end
        %#################################################################################

        robo_em_mov
        def_no = 0;
        
        
        bid_para_robo = pi*( sqrt((destino(1)-bid(bid_request_ind,2))^2+(destino(2)-bid(bid_request_ind,3))^2) - (L/2) )^2;
        mobile_node(robo_em_mov).bid_mobile = bid_para_robo; %bid(bid_request_ind);

        mobile_node(robo_em_mov).Pd = destino; 
        
        % Save picture
        new_bid = new_bid + 1;
        if(new_bid==1)
%             saveas(gcf,char('start'),'png');
%             saveas(gcf,char('start'),'eps');
        else
            saveas(gcf,char(['bid ',num2str(new_bid-1)]),'png');
            saveas(gcf,char(['bid ',num2str(new_bid-1)]),'eps');
        end
    end
    [Pii, V, W] = robot(mobile_node(robo_em_mov).P(:,end),mobile_node(robo_em_mov).Pd(:));
    P = [mobile_node(robo_em_mov).P, Pii];
    Vel = [mobile_node(robo_em_mov).Vel, [V ; W] ];
    mobile_node(robo_em_mov).P = P; 
    mobile_node(robo_em_mov).Vel = Vel; 
    % Actualizations of mobile nodes positions at environment
%     AQUI
    Pos_n(:,nf+robo_em_mov) = mobile_node(robo_em_mov).P(1:2,end); 
    dist_dest = sqrt((mobile_node(robo_em_mov).P(1,end)-mobile_node(robo_em_mov).Pd(1))^2+(mobile_node(robo_em_mov).P(2,end)-mobile_node(robo_em_mov).Pd(2))^2);
    if(dist_dest<=3);
        def_no = 1;
    end
    

    
%%     Check groups 
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % definição dos agrupamentos iniciais
    maior = 0;
    indice = 0;
    for j = 1:1:nG
        if(maior<(length(G(j).g)-1))
            maior = length(G(j).g)-1;
            indice = j;
%             network_nodes = maior;
        end
    end
    Pos_G = []; % Nós do maior agrupamento para a definição de nova fronteira
    for k = 2:length(G(indice).g)
        Pos_G = [Pos_G Pos_n(:,G(indice).g(k))];
    end
    % =========================================================================
    % PLOT ONLINE
    [M2 , cobertura] = plotonline3(M2,L,Pos_n,destino,nf,nm,maior,G(indice).g,indice_sink,bid);
    % =========================================================================    
    if(i==1)
        saveas(gcf,char('start'),'png');
        saveas(gcf,char('start'),'eps');
    end
    
    pause(0.01);
    
  

%% Critério de parada
% =========================================================================
    bid_mob = [mobile_node(1:nm).bid_mobile]
    if( (def_no ==1) && ( max(bid(:,1))<=min(bid_mob(:)) || sum(bid(:,1))==0 || length(G(indice).g)-1==N ) )
        finalizou = 1;
        saveas(gcf,char('finish'),'png');
        saveas(gcf,char('finish'),'eps');
    end
    


end % Fim do while - simulação
%% Métricas
% =========================================================================
% tic
tempo_operacao = toc
dist_total_percorrida = 0;
for i = 1:nm
    for j=1:length(mobile_node(i).P(1,:))-1
        distancia = sqrt((mobile_node(i).P(1,j+1) - mobile_node(i).P(1,j))^2+(mobile_node(i).P(2,j+1) - mobile_node(i).P(2,j))^2);
        dist_total_percorrida = dist_total_percorrida + distancia;
    end
end
dist_total_percorrida
numero_movimentos 
conectividade = (size(Pos_G,2)/(N))*100
% tempo_operacao = toc
cober = cobertura

disp('#################################')
disp('           FINALIZOU')
disp('#################################')
% % =========================================================================
save('experimento.mat')% Salva Experimento
% % =========================================================================