%% COMENT�RIOS

% Desvio de obst�culos do outro algoritmo
% M�tricas
% Ver detalhe da rela��o do algoritmo de atribui��o de tarefa com base na proximidade 
% Cobertura
% avaliar o buraco deivado com a movimenta��o requerida

function simulador_v2_artigo_experimentosVoronoi()
%%
clc
clear all
close all

disp('#################################')
disp('           INICIALIZOU')
disp('#################################')


%% inicialza��o das vari�veis globais

global mapa i angs tamos K L RREP RREQ RERR hop_cont afastamento lo robo_em_mov Pos_G1;        % vari�veis para rob� e fun��o de controle_e_navega��o

%% Taxonomia da Rede H�brida

%% PARAMETRIZA��O de rede e exibi��o
nm = 4; % N�mero de n�s m�veis
nf = 16;
N = nm+nf; % n�mero total de n�s 
L = 50; % raio de comunica��o 
mapa = imread('mapa.bmp'); 
exp = 52;
% mapa = rgb2gray(mapa); 
[Ay , Ax] = find(mapa~=255); % carregando o mapa quadrado do ambiente
C = size(mapa,1); % Comprimento da �rea quadrada em estudo
afastamento = C/4;
K = 1; % fator de escala da imagem para melhorar a resolu��o
lo = 0.5; % inicializa��o da grade de ocupa��o
destino = [0;0]; % Inicializa��o do destino/estrela

tic
% [  Pos_n , Pos_G , M , bid , G , indice , indice_sink  ] = inicializacao2(nm,nf);
[  Pos_n , Pos_G , M , bid , G , indice , indice_sink  ] = inicializacao3(nm,nf);
[c,l,z] = size(M);
M2 = im2double(M);
Pos_G1 = Pos_G;
%% CARACTER�STICAS DO EXPERIMENTO (din�mica de simula��o e vari�veis de suporte)
     
tamos = 0.2; % sampling period of simulation [s]
tempo(1) = 0;  % time control 
i = 0;  % contador

% Comunica��o AODV frames
RREP = zeros(nm,7); %envia RREP para o n� S (7 parametros na mensagem)
RREQ = zeros(nm,7); %envia RREQ para o n� S
RERR = zeros(nm,7); %envia RERR para vixinhos
hop_cont = 0;

finalizou = 0; % flag para armazenar os individuos que finalizou a tarefa pois est�o numa condi��o de n�o mobilidade
def_no = 1; % aponta defini��o de novo destino e n�
numero_movimentos = 0; % contabiliza o n�mero de rounds do algoritmo
cobertura_inicial = 0;
cobertura_final = 0;
tamanho_do_agrupamento = 0; % tamanho do agrupamento atual
trajetoria = []; % posi��o de todos os n�s m�veis durante o experimento
%% Inicializa��o dos rob�s e cria��o de uma estrutura de dados para eles
% 
% Posi��es iniciais e de destino para os n�s m�veis
for k=1:nm 
    Pi = [ Pos_n(1,nf+k) ; Pos_n(2,nf+k) ; 2*pi*rand(1) ]; % initial position of mobile nodes
    Pd = [ Pos_n(1,nf+k) ; Pos_n(2,nf+k) ]; % final position    
    % prealoca��o de mem�ria para plot de cada rob�
    Vel = [ 0; 0 ]; % speedd command [ V ; W ]; 
    com = 0; % communication flag
    bid_mobile = 0; % pre�o para movimenta��o do n� 
    % Saves and creates data struture named "mobile_node" 
    mobile_node(k) = struct('P',Pi,'Vel',Vel,'Pd',Pd,'com',com,'bid_mobile',bid_mobile);
end


%% Crit�rio de parada
% =========================================================================
bid_mob = [mobile_node(1:nm).bid_mobile];
if( (def_no ==1) && ( max(bid(:,1))<=min(bid_mob(:)) || sum(bid(:,1))==0 || length(G(indice).g)-1==N ) )
    finalizou = 1;
    saveas(gcf,char(['finish_exp',num2str(exp)]),'png');
    saveas(gcf,char(['finish_exp',num2str(exp)]),'eps');
end

robo_em_mov = 0;
new_bid = 0; % contador de tarefas atribu�das

while  (~finalizou)
    % actualization of the time control variables
    i = i+1;    
    i
    tempo = [tempo tempo(end)+tamos]; % time of global system    
    
    
%     CHECAR SE H� BURACOS E ATRIBUI AO N� MAIS PR�XIMO SE I==1 CASO
%     CONTR�RIO DEFINE O DE MENOR BID_M
    if(def_no)
        def_no = 0;
        if(i>1 & robo_em_mov~=0)
            mobile_node(robo_em_mov).Pd = mobile_node(robo_em_mov).P(:,end); 
            mobile_node(robo_em_mov).Vel = [0;0];
            robo_em_mov = 0;
        end
        numero_movimentos = numero_movimentos + 1;
        
        
         % sele��o do rob� baseado no menor bid
        menor_bid = l*c;
        for k=1:nm
            if(mobile_node(k).bid_mobile<menor_bid)
                menor_bid = mobile_node(k).bid_mobile;
                robo_em_mov = k;
            end
        end
        
        
        [M2,bid,cobertura] = voronoi_w2(M2,Pos_G,nf,nm,Pos_G1,afastamento,robo_em_mov); % VORONOI
        
%         if(i==1)
%             [~,bid_request_ind] = max(bid(1:size(Pos_G,2),1));
%         else
            [~,bid_request_ind] = max(bid(:,1));
%         end
%         if(bid_request_ind>size(Pos_G,2))
%             id_node_bid = bid_request_ind-(size(Pos_G,2)+1)+nf ; % n� requerente
%         else
%             cont00=0;
%             auxxx=0;
%             for m = 1:length(G(indice).g(:))
%                 if(G(indice).g(m)<=nf)
%                     cont00=cont00+1;
%                 end
%                 if(cont00==bid_request_ind)
%                    auxxx=m;  
%                 end
%             end
            auxxx=find(G(indice).g(:)<=nf); 
            id_node_bid = G(indice).g(auxxx(bid_request_ind+1));
%         end
            %#################################################################################
%         % verificando comunica��o do rob� para receber tarefa
%         menor_dist = l*c;
%         dist_robo_rede = (l*c)*ones(1,nm);
%         for k=1:nm  
%             for m=1:nf
%                 distancia = sqrt((mobile_node(k).P(1,end)-Pos_n(1,m))^2+(mobile_node(k).P(2,end)-Pos_n(2,m))^2);
%                 if(distancia<dist_robo_rede(k))
%                     dist_robo_rede(k) = distancia;
%                 end
%             end
%         end
        %#################################################################################

        %#################################################################################
       
%         [~,robo_sem_com] = min(dist_robo_rede); % ####### removendo o rob� sem com da possibilidade de movimenta��o
%         if(robo_em_mov==robo_sem_com)
%             mobile_node(k).bid_mobile = l*c;
%             menor_bid = l*c;
%             for k=1:nm
%                 if(mobile_node(k).bid_mobile<menor_bid)
%                     menor_bid = mobile_node(k).bid_mobile;
%                     robo_em_mov = k;
%                 end
%             end
%         end
        %#################################################################################
        
        % destino
        v = [-Pos_n(1,id_node_bid)+bid(bid_request_ind,2); -Pos_n(2,id_node_bid)+(bid(bid_request_ind,3))];
%         v = [-Pos_G1(1,id_node_bid)+bid(bid_request_ind,2); -Pos_G1(2,id_node_bid)+(bid(bid_request_ind,3))];
        v = v / sqrt(v(1)^2+v(2)^2);
        destino = ( ( sqrt(3) * (L/2) ) * v ) + Pos_n(:,id_node_bid);  % Obedecendo o destino fora da �rea de sensoriamento
%         destino = ( ( sqrt(3) * (L/2) ) * v ) + Pos_G1(:,id_node_bid);  % Obedecendo o destino fora da �rea de sensoriamento
    %     theta = atan2(Pos_n(2,id_node_bid)-(bid(bid_request_ind,3)),Pos_n(1,id_node_bid)-bid(bid_request_ind,2));
    %     destino = Pos_n(:,id_node_bid) + [L*cos(theta) ; L*sin(theta)];
        
%         % destino_2
%         vizinhos = Pos_n(:,id_node_bid);
%         for x = 1:length(Pos_G) % Identificando n�s pr�ximos
%             if(G(indice).g(1+x) ~= id_node_bid)
%                 dist = sqrt((Pos_n(1,id_node_bid)-Pos_G(1,x))^2+(Pos_n(2,id_node_bid)-Pos_G(2,x))^2);
%                 if(dist<=1.5*L)
%                     vizinhos = [vizinhos Pos_G(:,x)];
%                 end
%             end
%         end
%         [XV,YV] = voronoi(vizinhos(1,:),vizinhos(2,:)); % Identificando v�rtices pr�ximos
%         hold on
%         subplot(1,2,2)
%         scatter(XV(:),YV(:),20,'MarkerEdgeColor','g', 'MarkerFaceColor','g')
%         mais_distante = 0;
%         vertice_mais_dist = [0 ; 0];
%         for x=1:length(XV) % Identificando o v�rtice mais distante      
%             dist = sqrt((Pos_n(1,id_node_bid)-XV(x))^2+(Pos_n(2,id_node_bid)-YV(x))^2);
%             if(dist>mais_distante)
%                 mais_distante = dist;
%                 vertice_mais_dist = [XV(x); YV(x)];
%             end  
%         end
%         v = [-Pos_n(1,id_node_bid)+vertice_mais_dist(1); -Pos_n(2,id_node_bid)+vertice_mais_dist(2)];
%         v = v / sqrt(v(1)^2+v(2)^2);
%         destino = ( ( sqrt(3) * (L/2) ) * v ) + Pos_n(:,id_node_bid)  % Obedecendo o destino fora da �rea de sensoriamento
%         
        %#################################################################################
        % sele��o do rob� baseada na menor dist�ncia
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
        
        
        
        bid_para_robo = pi*( sqrt((destino(1)-bid(bid_request_ind,2))^2+(destino(2)-bid(bid_request_ind,3))^2) - (L/2) )^2;
        mobile_node(robo_em_mov).bid_mobile = bid_para_robo; %bid(bid_request_ind);

        mobile_node(robo_em_mov).Pd = destino; 
        
        % Save picture
        new_bid = new_bid + 1;
        if(new_bid==1)
%             saveas(gcf,char('start'),'png');
%             saveas(gcf,char('start'),'eps');
        else
            saveas(gcf,char(['bid_exp',num2str(exp),'_',num2str(new_bid-1)]),'png');
            saveas(gcf,char(['bid_exp',num2str(exp),'_',num2str(new_bid-1)]),'eps');
        end
    end
    [Pii, V, W] = robot(mobile_node(robo_em_mov).P(:,end),mobile_node(robo_em_mov).Pd(:));
    P = [mobile_node(robo_em_mov).P, Pii];
    Vel = [mobile_node(robo_em_mov).Vel, [V ; W] ];
    mobile_node(robo_em_mov).P = P; 
    mobile_node(robo_em_mov).Vel = Vel;
    trajetoria = [trajetoria, [mobile_node(1).P(1:2,end);mobile_node(2).P(1:2,end);mobile_node(3).P(1:2,end);mobile_node(4).P(1:2,end)]];
    % Actualizations of mobile nodes positions at environment
%     AQUI
    Pos_n(:,nf+robo_em_mov) = mobile_node(robo_em_mov).P(1:2,end);
%     Pos_G1(:,nf+robo_em_mov) = mobile_node(robo_em_mov).P(1:2,end); 
    dist_dest = sqrt((mobile_node(robo_em_mov).P(1,end)-mobile_node(robo_em_mov).Pd(1))^2+(mobile_node(robo_em_mov).P(2,end)-mobile_node(robo_em_mov).Pd(2))^2);
    
    if(dist_dest<=3)
        def_no = 1;
    end
    if(i==1)
        cobertura_inicial = cobertura;
    end

    
%%     Check groups 
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % defini��o dos agrupamentos iniciais
%     [G,nG,selec] = agrupamentos(L,Pos_G1,nf,nm); % defini��o dos agrupamentos iniciais
    maior = 0;
    indice = 0;
    for j = 1:1:nG
        if(maior<(length(G(j).g)-1))
            maior = length(G(j).g)-1;
            indice = j;
%             network_nodes = maior;
        end
    end
    Pos_G = []; % N�s do maior agrupamento para a defini��o de nova fronteira
    Pos_G1 = [];
    for k = 2:length(G(indice).g)
        Pos_G = [Pos_G Pos_n(:,G(indice).g(k))];
        if(G(indice).g(k)<=nf)
            Pos_G1 = [Pos_G1 Pos_n(:,G(indice).g(k))];
        end
    end
    [M2,bid,cobertura] = voronoi_w2(M,Pos_G,nf,nm,Pos_G1,afastamento,robo_em_mov); % VORONOI
    %#####################################################################################
    % Identifica��o de novo agrupamento
    if(i==1)
       tamanho_do_agrupamento = maior; 
    end
    if(maior>tamanho_do_agrupamento) % habilita o rec�lculo de buracos e ativa��o de outro rob� 
        tamanho_do_agrupamento = maior;
        def_no = 1;
        saveas(gcf,char(['roteador_exp',num2str(exp),'_',num2str(numero_movimentos)]),'png');
        saveas(gcf,char(['roteador_exp',num2str(exp),'_ ',num2str(numero_movimentos)]),'eps');
    end    
    if(maior<tamanho_do_agrupamento)
        flag_robo_na_rede = 0;
        for z=1:length(G(indice).g)
            if(G(indice).g(z)==(robo_em_mov+nf))
                flag_robo_na_rede = 1;
            end
        end
        tamanho_do_agrupamento = maior;
        if(~flag_robo_na_rede)
            def_no = 1;
            mobile_node(robo_em_mov).Pd = mobile_node(robo_em_mov).P(:,end); 
            mobile_node(robo_em_mov).Vel = [0;0];
            bid_mob = [mobile_node(1:nm).bid_mobile];
            mobile_node(robo_em_mov).bid_mobile = max(bid_mob);
            saveas(gcf,char(['roteador_exp',num2str(exp),'_',num2str(numero_movimentos)]),'png');
            saveas(gcf,char(['roteador_exp',num2str(exp),'_ ',num2str(numero_movimentos)]),'eps');
        end
    end
    %#####################################################################################
    
    % =========================================================================
    % PLOT ONLINE
    [M2 , cobertura] = plotonline4(M2,L,Pos_n,destino,nf,nm,maior,G(indice).g,indice_sink,bid);
    % =========================================================================    
    if(i==1)
        saveas(gcf,char(['start_exp',num2str(exp)]),'png');
        saveas(gcf,char(['start_exp',num2str(exp)]),'eps');
    end
    
    pause(0.01);
    
  

%% Crit�rio de parada
% =========================================================================
    bid_mob = [mobile_node(1:nm).bid_mobile]
%     if( (def_no ==1) && ( max(bid(:,1))<=min(bid_mob(:)) || sum(bid(:,1))==0 || length(G(indice).g)-1==N ) )
    if( (def_no ==1) && ( numero_movimentos==4 || length(G(indice).g)-1==N ) )    
        finalizou = 1;
        saveas(gcf,char(['finish_exp',num2str(exp)]),'png');
        saveas(gcf,char(['finish_exp',num2str(exp)]),'eps');
        cobertura_final = cobertura;
    end
    


end % Fim do while - simula��o


%% M�tricas
% =========================================================================
% tic
tempo_operacao = toc
tempo_operacao_min = tempo_operacao/60
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
cober = cobertura_final
relacao_de_cobertura = cober/cobertura_inicial
% velocidade_media = [mean(mobile_node(1).Vel(1,:)) mean(mobile_node(2).Vel(1,:)); mean(mobile_node(3).Vel(1,:)); mean(mobile_node(4).Vel(1,:));
%                     mean(mobile_node(1).Vel(2,:)) mean(mobile_node(2).Vel(2,:)) mean(mobile_node(3).Vel(2,:)) mean(mobile_node(4).Vel(2,:))]
disp('#################################')
disp('           FINALIZOU')
disp('#################################')
% % =========================================================================
save(['exp',num2str(exp),'.mat'])% Salva Experimento
% % =========================================================================