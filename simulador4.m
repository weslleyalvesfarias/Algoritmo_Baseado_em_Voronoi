function simulador4()
%%
clc
clear all
close all


%% inicializa��o das vari�veis globais

global mapa i angs L E E2 X X2 F Ft T E4 n_est dist_robo RREP RREQ RERR hop_cont dpar;        % vari�veis para rob� e fun��o de controle_e_navega��o

%% Taxonomia da Rede H�brida

%% PARAMETRIZA��O de rede e exibi��o
nm = 4; % N�mero de n�s m�veis
nf = 16;
N = nm+nf; % n�mero total de n�s 
L = 50; % raio de comunica��o 
mapa = imread('mapa.bmp'); mapa = rgb2gray(mapa); [Ay , Ax] = find(mapa~=255); % carregando o mapa quadrado do ambiente
C = size(mapa,1); % Comprimento da �rea quadrada em estudo
afastamento = C/4;
K = 1; % fator de escala da imagem para melhorar a resolu��o
lo = 0.5; % inicializa��o da grade de ocupa��o
destino = [0;0]; % Inicializa��o do destino/estrela

flag_busca = 1; % flag para gerar novo destino de busca na borda;
min_robo_dest = nm+1; % indice do rob� mais pr�ximo a este destino;
flag_new_node = 0; % identify new conection with new node
flag_miss_node = 0; % identify miss conection with a node
flag_rescue = 0; % identify that there is robot to rescue other robot
network_nodes=0;
flag_grupo = 0; % check the number of nodes of network


%% INICIALIZA��O ALEAT�RIA DOS N�S FIXOS E M�VEIS COM EXIBI��O
Pos_n = zeros(2,N-nm);  
for i=1:nf
    x1 = 0;
    y1 = 0;
%     while((x1<L || x1>(C-L)) || (y1<L || y1>(C-L))) % restri��o para alocar a regi�o de cobertura do n� totalmente dentro da �rea em estudo (mapa)
    while((x1<afastamento || x1>(C-afastamento)) || (y1<afastamento || y1>(C-afastamento))) % restri��o para alocar a regi�o de cobertura do n� totalmente dentro da �rea em estudo (mapa)    
        x1 = C*rand;
        y1 = C*rand;
    end
    Pos_n(:,i) = [x1; y1]; % rand uniforme
end

% Atualiza��o em escala das posi��es
C = K*C;
L=K*L;
Pos_n = K*Pos_n;



%% Criar e checar os agrupamentos da distribui��o aleat�ria

[G,nG,selec] = agrupamentos(L,Pos_n,nf,0); % defini��o dos agrupamentos iniciais


% EXIBI��O DOS AGRUPAMENTOS
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
   Pos_G = [Pos_G Pos_n(:,G(indice).g(k))];
end

% [M2,fronteira] = occupancy_grid_mapping_fronteira(M2,L,Pos_G,lo);
% % figure
% imshow(M2)

% identifica��o do n� sink - M�trica maior n�mero de vizinhos no
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
    [maxx,indice_sink]=max(cont);
else
    maxx=0;
    indice_sink=2;
end
% cont
indice_sink = G(indice).g(indice_sink);

disp(['N� Sink: ', num2str(indice_sink)])

%Sorteio dos n�s m�veis dentro do maior agrupamento
flag0 = 0;
for ii = 1:nm
    flag0 = 0;
    while(~flag0)
        for q = 2:1:length(G(indice).g) % plota �rea de cobertura
            x1 = C*rand;
            y1 = C*rand;
            if(sqrt((Pos_n(2,G(indice).g(q))-(y1))^2+((Pos_n(1,G(indice).g(q))-(x1))^2))<L && ~flag0 && (x1>L) && (x1<(C-L)) && (y1>L) && (y1<(C-L)))
               Pos_n = [Pos_n [x1; y1]]; % rand uniforme
               flag0 = 1;
            end
        end
    end
end


%Exibi��o e classifica��o dos n�s 
subplot(2,2,[2,4])
  
plot(Pos_n(1,1:nf),Pos_n(2,1:nf),'db');  % plot da distribui��o n�s m�veis
axis([-10 C+(10) -10 C+(10)])
hold on
plot(Pos_n(1,nf+1:end),Pos_n(2,nf+1:end),'sr');  % plot da distribui��o
% limites da imagem
L1 = zeros(1,c);
L2 = c.*ones(1,c);
L3 = 1:1:c;
plot(L1,L3,'k');
plot(L2,L3,'k');
plot(L3,L1,'k');
plot(L3,L2,'k');
for k=1:N  
  x = -L:1:L;
  c_plus = sqrt(L^2-x.^2);
  c_minus = -sqrt(L^2-x.^2);
  hold on
  if(k<=nf)
    plot(x+Pos_n(1,k),c_plus+Pos_n(2,k),':b')
    plot(x+Pos_n(1,k),c_minus+Pos_n(2,k),':b')
  else
    plot(x+Pos_n(1,k),c_plus+Pos_n(2,k),':r')
    plot(x+Pos_n(1,k),c_minus+Pos_n(2,k),':r')
  end
end
% legend('N� fixo','N� m�vel','Location','southoutside')
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
    else % restrito at� 999 n�s
        if(k<10)
            Nos = [Nos; ['00',int2str(k)]];
        elseif(k>=10 && k<100)
            Nos = [Nos; ['0',int2str(k)]];
        else
            Nos = [Nos; ['',int2str(k)]];
        end
    end
    text(Pos_n(1,k),Pos_n(2,k)-4,char(Nos(k,:)));
end





% % Destino definido aleatoriamente para o rob� atrav�s da lista de pixeis fronteira
% indice_dest = ceil(rand(1,1)*(length(fronteira)));
% destino = [fronteira(1,indice_dest); fronteira(2,indice_dest)];   % destino na fronteira [Y,X]
% % plot(destino(1,1),destino(2,1),'g*')
% if(destino(1,1)>1 && destino(1,1)<size(M,2) && destino(2,1)>1 && destino(2,1)<size(M,1))
%     M(destino(2,1),destino(1,1),1) = 255.0;
%     M(destino(2,1),destino(1,1),2) = 255.0;
%     M(destino(2,1),destino(1,1),3) = 0.0;
%     
%     M(destino(2,1)-1,destino(1,1),1) = 255.0;
%     M(destino(2,1)-1,destino(1,1),2) = 255.0;
%     M(destino(2,1)-1,destino(1,1),3) = 0.0;
%     M(destino(2,1)+1,destino(1,1),1) = 255.0;
%     M(destino(2,1)+1,destino(1,1),2) = 255.0;
%     M(destino(2,1)+1,destino(1,1),3) = 0.0;
%     M(destino(2,1),destino(1,1)+1,1) = 255.0;
%     M(destino(2,1),destino(1,1)+1,2) = 255.0;
%     M(destino(2,1),destino(1,1)+1,3) = 0.0;
%     M(destino(2,1),destino(1,1)-1,1) = 255.0;
%     M(destino(2,1),destino(1,1)-1,2) = 255.0;
%     M(destino(2,1),destino(1,1)-1,3) = 0.0;
% else
%     M(destino(2,1),destino(1,1),1) = 255.0;
%     M(destino(2,1),destino(1,1),2) = 255.0;
%     M(destino(2,1),destino(1,1),3) = 0.0;
% end

% indice_dest = ceil(rand(1,1)*(length(fronteira)));
% destino = [fronteira(1,indice_dest); fronteira(2,indice_dest)];   % destino na fronteira [Y,X]
% disp(['Destino: ', num2str(destino(2)), ', ',num2str(destino(1))])

Pos_G = []; % Aloca os n� m�veis e dentro do maior agrupamento para a defini��o de nova fronteira
for k = 1:length(G(indice).g)-1
%     if(k<=length(G(indice).g)-1)
        Pos_G = [Pos_G Pos_n(:,G(indice).g(k+1))];
%     else
%         Pos_G = [Pos_G Pos_n(:,nf+k-(length(G(indice).g)-1))];
%     end
end


% Plot n�s m�veis e nos fixos (c�rculos para representar os n�s)
for k = 1:1:l % linha 
  for j = 0:1:c-1 % coluna   
    for p = 1:1:length(Pos_n)% plot n� vermelho (m�vel) e azul (fixo)
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

subplot(2,2,[1,3])
[M2,fronteira] = occupancy_grid_mapping_fronteira(M2,L,Pos_G,lo);
% figure
% imshow(M2)

% Plot n�s m�veis e nos fixos (c�rculos para representar os n�s)
for k = 1:1:l % linha 
  for j = 0:1:c-1 % coluna   
    for p = 1:1:length(Pos_n)% plot n� vermelho (m�vel) e azul (fixo)
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

% subplot(2,2,[1,3])
imshow(M2)
hold on
for i=1:N
    text(Pos_n(1,i)+K,C-Pos_n(2,i)+K,char(Nos(i,:)));
    if(i==indice_sink)
       text(Pos_n(1,i)+K,C-Pos_n(2,i)+(6*K),'sink');
    end
end

%% CARACTER�STICAS DO EXPERIMENTO (din�mica de simula��o e vari�veis de suporte)
     
tamos = 0.25; % sampling period of simulation [s]
t_lim = 60*3; % limit of time to simulation [s]
tempo(1) = 0;  % time control 
i = 0;  % contador

ts = zeros(1,nm); % time without communication (visto pelo supervisor)
tn = zeros(1,nm); % time with communication (visto pelo n� m�vel)
t_sup = 0.25*4; % tempo de requerimento de amostragem do supervisor
t_no = 0.25*5; % tempo de requerimento de amostragem do rob�
buffer_sup = 10*ones(1,nm); % mensagem que chega do supervis�rio ao rob� - Check
buffer_no = 10*ones(1,nm); % mensagem que chega do rob� ao supervis�rio  - Check
ts_max = zeros(1,nm); % controle de tempo m�ximo medido para gera��o dos eventos de ativa��o dos rob�s de resgaste
tn_max = zeros(1,nm); % controle de tempo m�ximo medido para rob� parar - N�o precisaria para os n�s fixos
dpar = 100;  % dist�ncia m�nima de aproxima��o de pared/obst�culo fixo  [COMENTAR]
dist_robo = 100; % dist�ncia m�nima de aproxima��o entre rob�s
dist_dest = 2;  % [cm] dist�ncia m�nima para o destino
% Vsslf = zeros(1,nm); % Lyapunov quando ativa controladores Secund�rio [COMENTAR]
% V2 = zeros(1,nm); % Lyapunov quando sem Comunica��o
% norma1 = zeros(1,nm); % Norma usada no teorema de controle principal
% norma2 = [0 0 0 0];
% tssfl = 2*ones(1,5); % tempo de perman�ncia no controlador secund�rios
% tssf2 = [2 2 2 2]; % tempo sem cvomunica��o
% rho1 = 3; 
% rho2 = 3;
% thau1 = 5; % tempo de resid�ncia no controlador secund�rio [s]
% thau2 = 2; % tempo de resid�ncia do rob� sem comunica��o [s]  = (L/2)/vel
% t_evento = []; % tempo da ocorr�ncia de evento
% colidiu = []; % colis�o de cada rob�
% est = []; % evento de cada rob�
% t_sup_max = (L/2)/20;
% t_robo_max = (L/2)/20;
t_sup_max = zeros(1,nm); % (L/2)/Vmax tempo m�ximo do rob� sem comunica��o - medido pelo supervisor
t_no_max = zeros(1,nm); % (L/2)/Vmax tempo m�ximo do n� sem comunica��o - medido pelo n� - o n� fixo n�o necessita

t_sup_max(1,1:end)= L/22; % vmax = 22; % [cm/s] Velocidade limite de cada roda (22 cm/s V_max TRANSLACIONAL)
t_no_max(1,1:end) = L/22;

% Comunica��o AODV frames
RREP = zeros(nm,7); %envia RREP para o n� S (7 parametros na mensagem)
RREQ = zeros(nm,7); %envia RREQ para o n� S
RERR = zeros(nm,7); %envia RERR para vixinhos
hop_cont = 0;
% Pd_resg = zeros(2,nm); % armazena a �ltima posi��o do rob� antes de se desaconectar na rede  

finalizou = zeros(1,nm); % flag para armazenar os individuos que finalizou a tarefa pois est�o numa condi��o de n�o mobilidade
% robo_evento=0; % robo que gerou o evento
EVENTO = []; % armazena os eventos de todos os rob�s durante o experimento
EST = []; % armazena os estados de todos os rob�s durante o experimento
load('dados.mat'); % carrega dados de composi��o dos aut�matos - SUPERVIS�RIO


flag_busca = 1; % flag para gerar novo destino de busca na borda;
% min_robo_dest = nm+1; % indice do rob� mais pr�ximo a este destino;
% flag_new_node = 0; % identify new conection with new node
% flag_miss_node = 0; % identify miss conection with a node
% flag_rescue = 0; % identify that there is robot to rescue other robot
% network_nodes=0;
% flag_grupo = 0; % check the number of nodes of network
controlador = 0; % 0 = parado / 1 = seguir fronteira / 2 = desvio de obst�culo tangencial











%% Inicializa��o dos rob�s e cria��o de uma estrutura de dados para eles
% 
% Posi��es iniciais e de destino para os n�s m�veis
for k=1:nm 
    Pi = [Pos_n(1,k);Pos_n(2,k);0]; % initial position of mobile nodes
    Pd = [Pos_n(1,k);Pos_n(2,k)]; % final position    
    % prealoca��o de mem�ria para plot de cada rob�
    Vel = [0; 0]; % speedd command  
    est = 1; % actual state
    e = 0; % flag of event detected
    ev = 0; % event (string)
    est_ant = est; % previous state
    restricao = 1; % restriction of each robot 
    com = 0; % communication flag
    Lyap=0; % Lyapunov value
    dest=0;
    obs=0; % obstacle detection
    % Saves and creates data struture named "mobile_node" 
    mobile_node(k) = struct('P',Pi,'Vel',Vel,'Pd',Pd,'est',est,'ev',ev,'e',e,'est_ant',est_ant,'restricao',restricao,'com',com,'Lyap',Lyap,'dest',dest,'obs',obs);
end

% Autmata parameters - description]
X = [1, 2, 3, 4, 5, 6, 7, 8, 9]; % states of automata
Xo = 1; % initial state
Xm = [1, 2, 9];
X3 = [1, 2, 3, 4, 5, 6, 7, 8, 9]; % estados do aut�mato restri��o
E=['c', 'd', 'e', 'f', 'g', 'm', 'n', 'o', 'r', 's', 't', 'x']; % aut�mato ativo
% E2=['a', 'b', 'e', 'f']; % aut�mato passivo
E3=['c1', 'd1', 'e1', 'f1', 'g1', 'm1', 'n1', 'o1', 'r1', 's1', 't1', 'x1', 'c2', 'd2', 'e2', 'f2', 'g2', 'm2', 'n2', 'o2', 'r2', 's2', 't2', 'x2', 'c3', 'd3', 'e3', 'f3', 'g3', 'm3', 'n3', 'o3', 'r3', 's3', 't3', 'x3', 'c4', 'd4', 'e4', 'f4', 'g4', 'm4', 'n4', 'o4', 'r4', 's4', 't4', 'x4'];
Ex=E;
X3m = [1, 2, 3, 4, 5, 6, 7, 8, 9];
X3o = 1;


evento = 0; % start automata (no event occurrence)
for k=1:nm
    est = automatonomovel(X,E,X(1),evento,Xm,Xo); %  it indicates robot initial state
    mobile_node(k).est = est;
    mobile_node(k).est_anterior = est;
end
restricao = automatorestricao(X3,E3,X3(1),evento,X3m,X3o); %  it indicates restriction initial state

estado = buscarestado2(Ft,E3,length(E),0,1,1); % state = 1;



while  (~flag_grupo)
    % actualization of the time control variables
    i = i+1;     
    tempo = [tempo tempo(end)+tamos]; % time of global system
    ts = ts + ones(1,nm).*tamos;
    tn = tn + ones(1,nm).*tamos;
    
       
    for k=1:nm    
        [Pi, V, W, obs, Ly] = robot(mobile_node(k).P(:,end),mobile_node(k).Pd,controlador,controlador);
        P = [mobile_node(k).P, Pi];
        Vel = [mobile_node(k).Vel, [V;W]];
        mobile_node(k).P = P; 
        mobile_node(k).Vel = Vel; 
        mobile_node(k).ev = 0;  
%         mobile_node(k).com = 0; % flag comunica��o 
        Lyap = [mobile_node(k).Lyap Ly];
        mobile_node(k).Lyap = Lyap;
        mobile_node(k).obs = obs; 
        
%         % VARI�VBEIS PARA O CONTROLE DE OBJETIVO PRINCIPAL
%         % C�lculo do erro de orienta��o para o destino (theta_e)
%         theta_d = atan2((mobile_node(k).Pd(2)-mobile_node(k).P(2,end)),(mobile_node(k).Pd(1)-mobile_node(k).P(1,end))); % �ngulo de destino de -pi a pi
%         % converte theta para -pi a pi
%         if theta_d > pi, theta_d = theta_d - 2*pi; end
%         if theta_d < -pi, theta_d = theta_d + 2*pi; end
%         theta_e = - mobile_node(k).P(3,end) + theta_d;
%         norma(k) = (abs(sqrt((mobile_node(k).Pd(1)-mobile_node(k).P(1,end))^2 + (mobile_node(k).Pd(2)-mobile_node(k).P(2,end))^2)))+(abs(theta_e));


        mobile_node(k).dest = sqrt((mobile_node(k).P(1,end)-destino(1,1))^2+(mobile_node(k).P(2,end)-(C-destino(2,1)))^2);
        
    end
    
    % Actualizations of mobile nodes positions at environment
        Pos_n(1:2,end-k) = mobile_node(nm-k).P(1:2,end); 
    end
    
%%     Check groups 
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % defini��o dos agrupamentos iniciais
    maior = 0;
    indice = 0;
    for i = 1:1:nG
        if(maior<(length(G(i).g)-1))
            maior = length(G(i).g)-1;
            indice = i;
%             network_nodes = maior;
        end
    end
    % plotonline(Pos_all,Front_all,mapa,nm+nf,nf,Pos_n,L,0);
    [M2,fronteira] = plotonline2(M2,L,Pos_n,destino,nf,nm,maior,G(indice).g,indice,indice_sink,lo);
%%    
%     mob=[]; % checa os rob�s que podem se mover
%     id = [];
%     for j=1:nm
%         if( ~finalizou(j) && (buffer_sup(j+nf)==3 || buffer_sup(j+nf)==10) ) % seleciona os rob�s que podem se mover
%             mob=[mob mobile_node(j).dest];
%             id = [id j];
%         end 
%     end 
    
%     [robo(1).Pd robo(2).Pd]
%     [robo(1).Lyap(end)-V1(1) -rho1*norma(1); robo(2).Lyap(end)-V1(2) -rho1*norma(2); robo(3).Lyap(end)-V1(3) -rho1*norma(3); robo(4).Lyap(end)-V1(4) -rho1*norma(4); robo(5).Lyap(end)-V1(5) -rho1*norma(5)]
    
   


    %% Supervis�rio   
   
    
    % Occurency event definition
    if(i>=1)   
        % Identify atual state of supervisory 
%         estado = 0; 
%         for k = 1:size(Ft,1)
%             if(robo(1).est == F(k,1) && robo(2).est == F(k,2)  && robo(3).est == F(k,3) && robo(4).est == F(k,4) && restricao == F(k,5))
%                 estado = k;
%             end
%         end
%         estado = buscarestado(F,mobile_node(1).est,mobile_node(2).est,mobile_node(3).est,mobile_node(4).est,restricao);
        
        
        if (network_nodes<length(G(indice).g)-1) % check if occured new conections in network
            flag_new_node = 1;
            network_nodes = length(G(indice).g)-1;
        end
        if (network_nodes>length(G(indice).g)-1) % check if occured new conections in network
            flag_miss_node = 1;
            network_nodes = length(G(indice).g)-1;
        end
        
        flag_rescue = 0;
        for l=1:nm
            if(l ~= k && mobile_node(k).est == 2)
               flag_rescue = 1;
            end
        end
        
        for k=1:nm  % k is the indicate mobile node
            
            if(~flag_grupo) 
                %% ROB�
                if(flag_busca) % Identifica��o de qual rob� est� mais pr�ximo do destino
                    flag_busca=0;
                    if(~flag_rescue)
                        indice_dest = ceil(rand(1,1)*(length(fronteira)));
                        destino = [fronteira(1,indice_dest); fronteira(2,indice_dest)];   % destino na fronteira [Y,X] j� coleta a linha e coluna corretamente
    %                         disp(['Destino: ', num2str(destino(1)), ', ',num2str(destino(2))])

                        dist_destino = C*ones(1,nm);
                        min_robo_dest = nm+1;   % indica o rob� mais pr�ximo do destino

                        for n = 1:nm
                            dist_destino(n) = sqrt((mobile_node(n).P(1,end)-destino(1))^2+(mobile_node(n).P(2,end)-(C-destino(2)))^2); 
                        end
                        min_robo_dest = find(dist_destino == min(dist_destino));
                        min_robo_dest = min_robo_dest + nf;
%                  
                    else%if(flag_rescue) % Rescue condition
                        for n = 1:nm
                            dist_destino(n) = sqrt((mobile_node(n).P(1,end)-destino(1))^2+(mobile_node(n).P(2,end)-(C-destino(2)))^2); 
                        end
                        min_robo_dest = find(dist_destino == min(dist_destino));
                        min_robo_dest = min_robo_dest + nf;
                    end
                end
                    
                % Communication between nodes
                if(((ts(k) >=t_sup || tn(k) >= t_no ) || tempo(end)==tamos))
                    buffer_sup(k+nf)=communicationAODV_Supervisor(L,indice_sink,k+nf,Pos_n(1:2,:));
                    flag_com = 1;
                    if(buffer_sup(k+nf)==3) % 2 with communication 
                        mobile_node(k).com = 1;
                    else  % without communication
                        mobile_node(k).com = 0;
                    end
                end    
                    
                %% Evento C (Evento N�o Crontrol�vel) - Supervis�rio checa
                % comunica�ao com o rob�
                if(mobile_node(k).e == 0 && mobile_node(k).com && ((ts(k) >=t_sup || tn(k) >= t_no )) && flag_com == 1) % estouro de tempo do supervisor ou primeira amostra   CHECAR SE O NO EST� NO MAIOR AGRUPAMENTO
                    if(T(estado,((size(E,2)*(k-1))+1))==1) 
                        estado = Ft(estado,(size(E,2)*(k-1))+1);
%                             ts(k) = 90; % reseta o tempo de amostragem 
%                             ts_max(k) = 0; % reseta o tempo de amostragem aguardo para comunica��o
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'c'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,Xm,Xo); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,mobile_node(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento c: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end
                end
                % Evento t (Evento Crontrol�vel) - passa tarefa de navega��o - seguir fronteira 
                if(mobile_node(k).e == 0 && min_robo_dest == k && length(G(indice).g)-1<N && (buffer_sup(k)==3) && ~finalizou(k))
%                     if(robo(k).e == 0 && min_robo_dest==(k+nf) && length(G(indice).g)<(nm+nf+1) && k==indice_min && (buffer_sup(k+nf)==3) && ~finalizou(k)) % aponta que o supervis�rio tem acesso ao rob� (h� caminho) - indice_min indica qual robo deve se mover
%                     if(robo(k).e == 0 && (buffer_sup(k)==2)) % aponta que o supervis�rio tem acesso ao rob� (h� caminho)
                    if(T(estado,((size(E,2)*(k-1))+11))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+11);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 't'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,Xm,Xo); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,mobile_node(k).restricao,[evento num2str(k)],Xm3,Xo3); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento t: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])      
                    end 
                end

                % Evento S (Evento N�o-control�vel) - Sem comunica��o
                % com a rede
                % AODV (A vari�vel 'com' funcionar� como flag para armazenar se est� na rede '3' ou n�o '1')                      
                if(mobile_node(k).e == 0 && mobile_node(k).com==0 && flag_com == 1)% && robo(k).est==3)
                    if(T(estado,((size(E,2)*(k-1))+10))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+10);
%                             ts_max(k) = 0;
%                             tr_max(k) = 0;
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 's'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev=evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,mobile_node(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento s: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                      
                end
%                 % Evento D (Evento N�o-control�vel) - Rob� cumpriu a
%                 % tarefa
%                 if(mobile_node(k).e == 0 && sqrt((mobile_node(k).P(1,end)-mobile_node(k).Pd(1))^2 + (mobile_node(k).P(2,end)-mobile_node(k).Pd(2))^2)<=dist_dest) % [2cm]
%                     if(T(estado,((size(E,2)*(k-1))+2))==1)
%                         estado = Ft(estado,(size(E,2)*(k-1))+2);
%                         mobile_node(k).e = 1; % flag de evento para o rob�
%                         evento = 'd'; % identificador do evento pra o aut�mato
%                         mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
%                         mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
%                         mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,mobile_node(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
%                         disp(['No R',num2str(k+nf),' - evento d: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
% %                             finalizou(k) = 1; disp('FIM rob� 1')
%                     end                     
%                 end

                % Evento x (Evento N�o-control�vel) - Tempo m�ximo sem
                % comunica��o atingido
                if(mobile_node(k).e == 0 && (tn_max(k)>=t_no_max(k)) && (ts_max(k)>=t_sup_max(k))) % VER SE NECESSITO DE TEMPORIZADORES INDIVIDUAIS ROB� E SUPERVISOR
                    if(T(estado,((size(E,2)*(k-1))+12))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+12);
%                             tr_max(k) = 0; % reseta o tempo de amostragem
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'x'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,Xm,Xo); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,mobile_node(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento x: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end
                end
                
                % Evento D (Evento N�o-control�vel) - Rob� executou 
                % volta de 360�
                if(mobile_node(k).e == 0 && sqrt((mobile_node(k).P(1,end)-mobile_node(k).Pd(1))^2 + (mobile_node(k).P(2,end)-mobile_node(k).Pd(2))^2)<dist_dest && mobile_node(k).com == 1)   % FALTA INCLUIR INFORMA��O DE QUE ENCONTROU N�
                    if(T(estado,((size(E,2)*(k-1))+2))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+2);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'd'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,Xm,Xo); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,mobile_node(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento d: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
%                             finalizou(k) = 1; disp('FIM rob� 1')
                    end                     
                end
                    
                % Evento E (Evento N�o-control�vel) - Rob� encontrou n�(s)
                if(mobile_node(k).e == 0 && flag_new_node)   % FALTA INCLUIR INFORMA��O DE QUE ENCONTROU N�
                    if(T(estado,((size(E,2)*(k-1))+3))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+3);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'e'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,Xm,Xo); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,mobile_node(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento e: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                     
                end
                    
                % Evento F (Evento N�o-control�vel) - Rob� cumpriu o
                % desvio de obst�culo
                if(mobile_node(k).e == 0 && obs == 0)  % Talves incluir a informa��o de fronteira
                    if(T(estado,((size(E,2)*(k-1))+4))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+4);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'f'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,robo(k).restricao,evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento f: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                     
                end 
                    
                                        
                % Evento G (Evento Control�vel) - Rob� deixa de ser
                % roteador
                if(mobile_node(k).e == 0 && obs == 0)  
                    if(T(estado,((size(E,2)*(k-1))+5))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+5);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'g'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,robo(k).restricao,evento,X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento g: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                     
                end  
                    
                % Evento M (Evento N�o-control�vel) - H� rob� de suporte para resgate
                if(mobile_node(k).e == 0 && flag_rescue==1)  
                    if(T(estado,((size(E,2)*(k-1))+6))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+6);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'm'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,robo(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento m: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                     
                end 
                
                % Evento N (Evento N�o-control�vel) - N�o h� rob� de suporte para resgate
                if(mobile_node(k).e == 0 && flag_rescue==0)  
                    if(T(estado,((size(E,2)*(k-1))+7))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+7);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'n'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,robo(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento n: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                     
                end
                
                % Evento O (Evento N�o-control�vel) - Desvio de obst�culo tangencial
                if(mobile_node(k).e == 0 && obs == 1)  
                    if(T(estado,((size(E,2)*(k-1))+8))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+8);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'o'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,robo(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento o: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                     
                end
                
                % Evento r (Evento N�o-control�vel) - Roteador
                if(mobile_node(k).e == 0 && flag_new_node)
                    if(T(estado,((size(E,2)*(k-1))+9))==1)
                        estado = Ft(estado,(size(E,2)*(k-1))+9);
                        mobile_node(k).e = 1; % flag de evento para o rob�
                        evento = 'r'; % identificador do evento pra o aut�mato
                        mobile_node(k).ev = evento; % armazena �ltimo evento do rob�
                        mobile_node(k).est_ant = mobile_node(k).est; % armazena o estado anterior do rob�
                        mobile_node(k).est = automatonomovel(X,E,mobile_node(k).est,evento,X(1),X(1)); % indica o estado do rob�
%                         restricao = automatorestricao(X3,E3,robo(k).restricao,[evento num2str(k)],X3m,X3o); % indica o estado da transi��o do rob�
                        disp(['No R',num2str(k+nf),' - evento r: ' num2str(mobile_node(k).est_ant) ' to ' num2str(mobile_node(k).est)])
                    end                     
                end 
                
                estado = buscarestado2(Ft,E3,length(E),mobile_node(k).ev,estado,k);
            end % N�O AINTGIU O AGRUAENTO M�XIMO  
% 

           % Exibi��o
%            [robo(1).est,robo(2).est,robo(3).est,robo(4).est]
%            estado
%            [tr_max(1) tr_max(2) tr_max(3) tr_max(4); ts_max(1) ts_max(2) ts_max(3) ts_max(4)]
           
           
           
           %% Tempos de rob�s no secund�rio e sem comunica��o
%            if(robo(k).flagC~=0 && robo(k).flagC~=2 ) % Temporizador de controle secund�rio ativo
%            tssfl(k) = tssfl(k) + tamos;
%             else
%                 tssfl(k) = 0;
%             end
            %% Tempos de rob�s no secund�rio e sem comunica��o
            if(buffer_sup(k)==1) % Temporizador de rob� sem comunica��o
                ts_max(k) = ts_max(k) + tamos; % controle de tempo m�ximo medido para gera��o dos eventos de ativa��o dos rob�s de resgaste
            end
            if(buffer_no(k)==1) % Temporizador de rob� sem comunica��o
                tn_max(k) = tn_max(k) + tamos; % controle de tempo m�ximo medido para retornar ao �ltimo ponto de contato
%                 ts_max(k) = ts_max(k) + tamos; % controle de tempo m�ximo medido para gera��o dos eventos de ativa��o dos rob�s de resgaste
            end
            
            
           
        end % end for
             
    end % end if
        
         
% end % end experimento
%     
%     % Condi��es para encerrar a simula��o caso todos os rob�s batam ou
%     % concluam a tarefa
%     col=[];
%     for k=1:n
%         if(robo(k).colidiu)
%             col=[col, 1];
%         else
%             col=[col, 0];
%         end
% %         if(i>2)
% %             finalizou(k)=robo(k).est;
% %         end
%     end
    


  
    
    
%% Na ocorr�ncia de um 'evento controlado' (do SUPERVIS�RIO) � enviado ao rob� novos direcionamentos (Modela a Comunica��o)  
%% Mobile Node
    for k=1:nm % checa se o supervisor gerou algum evento de acordo com o estado em que ele se encontra
       
        switch mobile_node(k).est_ant
            
            case X(1) % Sem comunica��o (Marcado)
                % Descri��o das a��es dos eventos que partem desse estado
                if((mobile_node(k).e) && (mobile_node(k).ev=='s'))
                    controlador = 0;
                    flag_com = 0;
                    buffer_sup(k) = 1;
                    buffer_no(k) = 1;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 0;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
%                 if((mobile_node(k).e) && (mobile_node(k).ev=='a'))
%                      % buffer_robo(k) deve ser preenchido com um flag indicando resposta do rob�  
%                      % se s = 3 tenho vizinhos (RREQ), caso s = 1, n�o h� vizinhos (RERR)
%                      ts(k+nf) = 0; % reseta o tempo de amostragem
% 
% %                      for i=1:nm
%                          buffer_sup(k+nf)=communicationAODV_Supervisor(L,indice_sink,k+nf,Pos_n(1:2,:));
% %                          buffer_sup(i)=communicationAODV_send(L,5,i,[robo(1).P(1:2,end) robo(2).P(1:2,end) robo(3).P(1:2,end) robo(4).P(1:2,end) Pos_n(1:2,:)]);
%                          if(buffer_sup(k+nf)==3) % 2 Com comunica��o
%                              mobile_node(k).com = 1;
%                          else  % Sem comunica��o
%                              mobile_node(k).com = 0;
%                          end
%                     % VARI�VBEIS PARA O CONTROLE DE OBJETIVO PRINCIPAL
%                     % S� ocorre no estado 1
%                     % C�lculo do erro de orienta��o para o destino (theta_e)
%                     theta_d = atan2((mobile_node(k).Pd(2)-mobile_node(k).P(2,end)),(mobile_node(k).Pd(1)-mobile_node(k).P(1,end))); % �ngulo de destino de -pi a pi
%                     % converte theta para -pi a pi
%                     if theta_d > pi, theta_d = theta_d - 2*pi; end
%                     if theta_d < -pi, theta_d = theta_d + 2*pi; end
%                     theta_e = - mobile_node(k).P(3,end) + theta_d;
%                     Vsslf(k) = (((sqrt((mobile_node(k).Pd(1)-mobile_node(k).P(1,end))^2 + (mobile_node(k).Pd(2)-mobile_node(k).P(2,end))^2)))^2+(theta_e)^2)/2;
%                 end  
                
                                          
            case X(2) % Com comunica��o (Marcado)
                
                %% Rob� ativo
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 0;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='r'))
                    controlador = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='t'))
                    controlador = 0;
                end
               
                
%                 if((mobile_node(k).e) && (mobile_node(k).ev=='e'))
%                     % rob� sem conex�o
%                     ts_max(k+nf) = 0;
%                     tn_max(k+nf) = 0;
%                     t_sup_max(k+nf) = (L/2)/mean(mobile_node(k).Pvel(1,:));
%                     t_no_max(k+nf) = (L/2)/mean(mobile_node(k).Pvel(1,:)); 
% %                     Pd_g(1:2,k) = robo(k).P(:,end); % armazena �ltima posi��o do rob� na rede
%                     Pd_resg(:,k+nf) = mobile_node(k).P(1:2,end-2); % a posi��o de resgate do rob� � a �ltima posi��o conhecida do rob� desaparecido
%                     mobile_node(k).com = 0;
%                     
%                     
%                     
% %                     destino = ceil([Pd_resg(1,k+nf);Pd_resg(2,k+nf)]);   % destino na fronteira [Y,X]
%                     menor_dist_fronteira = L;
%                     indice_menor_dist_fronteira = 0;
%                     dist_fronteira = L;
%                     for m=1:length(fronteira) % identifica��o de ponto da fronteira mais pr�ximo ao desaparecimento
%                         dist_fronteira = sqrt((Pd_resg(1,k+nf)-fronteira(1,m))^2+(Pd_resg(2,k+nf)-(C-fronteira(2,m)))^2);
%                         if(menor_dist_fronteira>dist_fronteira)
%                             menor_dist_fronteira = dist_fronteira;
%                             indice_menor_dist_fronteira = m;
%                         end
%                     end
%                     destino = fronteira(:,indice_menor_dist_fronteira);
% %                     disp(['Destino: ', num2str(destino(1)), ', ',num2str(C-destino(2))])
%                     flag_busca=0; % busca da estrela n�o ativada
%                     dist_destino = C*ones(1,nm);
%                     min_robo_dest = nm+1;   % indica o rob� mais pr�ximo do destino
%                     for n =1:nm
%                         if(n~=k)
%                             dist_destino(n) = sqrt((mobile_node(n).P(1,end)-destino(1))^2+(mobile_node(n).P(2,end)-(C-destino(2)))^2); 
%                         end
%                     end
%                     [xxx,min_robo_dest] = min(dist_destino);
%                     min_robo_dest = min_robo_dest + nf;
%                     disp(['Destino: ', num2str(destino(1)),', ', num2str(C-destino(2))]);
%                     
%                 end


                
               
            case X(3) % Tarefa de navegar sobre a fronteira (N�o marcado)
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 0;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end 
                if((mobile_node(k).e) && (mobile_node(k).ev=='m'))
                    controlador = 1;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='n'))
                    controlador = 0;
                end


            case X(4) % H� rob� de resgate (N�o marcado)
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 1;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='r'))
                    controlador = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='o'))
                    controlador = 2;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='d'))
                    controlador = 0;
                    flag_busca = 1;
                end
%                 if((mobile_node(k).e) && (mobile_node(k).ev=='t'))
%                     % tempo sem comunica��o estoura - parar rob�
%                     tn_max(k+nf) = 0; % reseta o tempo de amostragem
%                     mobile_node(k).Pd = mobile_node(k).P(1:2,end); % a posi��o de destino do rob� � a posi��o do rob� desaparecido
%                     mobile_node(k).est_anterior = mobile_node(k).est;  
%                     
%                     ts_max(k+nf) = 0;
%                     tn_max(k+nf) = 0;
%                     t_sup_max(k+nf) = (L/2)/mean(mobile_node(k).Pvel(1,:));
%                     t_no_max(k+nf) = (L/2)/mean(mobile_node(k).Pvel(1,:));
% %                     Pd_resg(:,k+nf) = robo(k).P(1:2,end-1); % a posi��o de destino do rob� � a posi��o do rob� desaparecido
% %                     robo(k).com = 0;  
%                     
% %                     destino = ceil([Pd_resg(1,k+nf);Pd_resg(2,k+nf)]);   % destino na fronteira [Y,X]
%                     menor_dist_fronteira = L;
%                     indice_menor_dist_fronteira = 0;
%                     dist_fronteira = L;
%                     for m=1:length(fronteira) % identifica��o de ponto da fronteira mais pr�ximo ao desaparecimento
%                         dist_fronteira = sqrt((Pd_resg(1,k+nf)-fronteira(1,m))^2+(Pd_resg(2,k+nf)-(C-fronteira(2,m)))^2);
%                         if(menor_dist_fronteira>dist_fronteira)
%                             menor_dist_fronteira = dist_fronteira;
%                             indice_menor_dist_fronteira = m;
%                         end
%                     end
%                     destino = fronteira(:,indice_menor_dist_fronteira);
% %                     disp(['Destino: ', num2str(destino(1)), ', ',num2str(destino(2))])
%                     flag_busca=0; % busca da estrela n�o ativada
%                     dist_destino = C*ones(1,nm);
%                     min_robo_dest = nm+1;   % indica o rob� mais pr�ximo do destino
%                     for n =1:nm
%                         if(n~=k)
%                             dist_destino(n) = sqrt((mobile_node(n).P(1,end)-destino(1))^2+(mobile_node(n).P(2,end)-(C-destino(2)))^2); 
%                         end
%                     end
%                     [xxx,min_robo_dest] = min(dist_destino);
%                     min_robo_dest = min_robo_dest + nf;
%                     disp(['Destino: ',num2str(destino(1)),', ',num2str(C-destino(2))]);
%                     
%                 end


            case X(5) % Desvio de obst�culo (com com.) (N�o marcado)
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 2;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='f'))
                    controlador = 1;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='r'))
                    controlador = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='s'))
                    controlador = 2;
                    flag_com = 0;
                    destino = mobile_node(k).P(:,end);
                    buffer_sup(k) = 1;
                    buffer_no(k) = 1;
                end
                
            case X(6) % Desvio de obst�culo (sem com.) (N�o marcado)
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 1;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='x'))
                    controlador = 0;
                    flag_busca = 1;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='e'))
                    controlador = 2;
                end
            case X(7) % Encontrou n�(s) (N�o marcado)
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 0;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='x'))
                    controlador = 0;
                    flag_busca = 1;
                end
            case X(8) % Tempo estourado (N�o marcado)
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 0;
                    flag_com = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
            case X(9) % Roteador (Marcado)
                if((mobile_node(k).e) && (mobile_node(k).ev=='c'))
                    controlador = 0;
                    flag_com = 0;
                    ts(k) = 0; tn(k) = 0;
                    buffer_sup(k) = 0;
                    buffer_no(k) = 0;
                end
                if((mobile_node(k).e) && (mobile_node(k).ev=='g'))
                    controlador = 0;
                end
            otherwise
                disp('error - to send message after evet')
        end
    end
    
    
%% Crit�rio de parada
    if(length(G(indice).g)-1 == N) % check the number of nodes of network
        flag_grupo = 1;
    end
    aux_finalizacao=0;
    for k=1:nm % n�o podem se mover
        if(mobile_node.est==9)
            aux_finalizacao = aux_finalizacao+1;
        end
    end
    if(aux_finalizacao == nm-1) % check the number of nodes of network
        flag_grupo = 1;
    end
% 
% c1 = length(find(finalizou==1)); % rob�s cumpriram a tarefa
% if(c1==nm), fim=1; end
% if (tempo(end)>=t_lim) % tempo de simula��o atingido
%     fim=1;
% end


% flag_finalizou = 0;
% for p = 1:nm
%    if(finalizou(p)==1)  %% Colocar em fun��o de nm 
%     flag_finalizou = flag_finalizou+1;
%    end
% end

% flag_finalizou2 = 0;
% for p = 1:nm+nf
% %    if(p<=nf)
% %        if(nofixo(p).est==1)
% %            flag_finalizou2=flag_finalizou2+1;
% %        end
% %    elseif(robo(p-nf).est==1)
%        flag_finalizou2=flag_finalizou2+1;
% %    end
% end
% 

% if(flag_finalizou == nm ) % 
%     fim=1;
% end
% if(flag_finalizou2==(nf+nm) && tempo(end)>tamos) % 
%     fim=1;
% end
% if(nG==1) % nG n�mero de grupo igual a 1
%     fim=1;
% end


% =========================================================================


% =========================================================================
%% Plot online
% =========================================================================
figure(1)  % Plot Online do experimento 

Pos_all=[];
Front_all=[];

for k=1:nm
    Pos_all = [Pos_all; mobile_node(k).P];
    Front_all = [Front_all, mobile_node(k).Pd];
end




%%     Checar agrupamentos 
    [G,nG,selec] = agrupamentos(L,Pos_n,nf,nm); % defini��o dos agrupamentos iniciais
    % Defini��o do maior grupo
    maior = 0;
    indice = 0;
    for i=1:1:nf
        if(maior<(length(G(i).g)-1))
            maior = length(G(i).g)-1;
            indice = i;
        end
    end
    % plotonline(Pos_all,Front_all,mapa,nm+nf,nf,Pos_n,L,0);
    [M2,fronteira] = plotonline2(M2,L,Pos_n,destino,nf,nm,maior,G(indice).g,indice,indice_sink,lo);
%% 
% plotonline(Pos_all,Front_all,mapa,nm+nf,nf,Pos_n,L,0);
% M = plotonline2(M,L,Pos_n,destino,nf,nm,indice,indice_sink);



% Armazenamento dos Eventos de cada Rob�
aux=[];
for k=1:nm
    if(mobile_node(k).ev == Ex(1))
        if(mobile_node(k).e==1)
            aux=[aux; 1]; % a
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(2))
        if(mobile_node(k).e==1)
            aux=[aux; 2]; % b
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(3))
        if(mobile_node(k).e==1)
            aux=[aux; 3]; % c
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(4))
        if(mobile_node(k).e==1)
            aux=[aux; 4]; % d
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(5))
        if(mobile_node(k).e==1)
            aux=[aux; 5]; % e
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(6))
        if(mobile_node(k).e==1)
            aux=[aux; 6]; % f
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(7))
        if(mobile_node(k).e==1)
            aux=[aux; 7]; % g
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(8))
        if(mobile_node(k).e==1)
            aux=[aux; 8]; % h
        else
            aux=[aux; 0];
        end
    elseif(mobile_node(k).ev == Ex(9))
        if(mobile_node(k).e==1)
            aux=[aux; 9]; % t
        else
            aux=[aux; 0];
        end
%     elseif(robo(k).ev == Ex(10))
%         if(robo(k).e==1)
%             aux=[aux; 10]; % j
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(11))
%         if(robo(k).e==1)
%             aux=[aux; 11]; % k
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(12))
%         if(robo(k).e==1)
%             aux=[aux; 12]; % l
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(13))
%         if(robo(k).e==1)
%             aux=[aux; 13]; % m
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(14))
%         if(robo(k).e==1)
%             aux=[aux; 14]; % n
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(15))
%         if(robo(k).e==1)
%             aux=[aux; 15]; % o
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(16))
%         if(robo(k).e==1)
%             aux=[aux; 16]; % p
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(17))
%         if(robo(k).e==1)
%             aux=[aux; 17]; % q
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(18))
%         if(robo(k).e==1)
%             aux=[aux; 18]; % r
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(19))
%         if(robo(k).e==1)
%             aux=[aux; 19]; % s
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(20))
%         if(robo(k).e==1)
%             aux=[aux; 20]; % t
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(21))
%         if(robo(k).e==1)
%             aux=[aux; 21]; % u
%         else
%             aux=[aux; 0];
%         end
%     elseif(robo(k).ev == Ex(22))
%         if(robo(k).e==1)
%             aux=[aux; 22]; % v
%         else
%             aux=[aux; 0];
%         end    
%     elseif(robo(k).ev == Ex(23))
%         if(robo(k).e==1)
%             aux=[aux; 23]; % x
%         else
%             aux=[aux; 0];
%         end    
    else
        aux=[aux; 0];
    end
end
EVENTO=[EVENTO, aux];
% 

% eventosatuais=[robo(1).e robo(2).e robo(3).e robo(4).e]
% estadosatuais=[robo(1).est robo(2).est robo(3).est robo(4).est]
% [robo(1).est,robo(2).est,robo(3).est,robo(4).est,robo(5).est]%,robo(1).restricao,robo(2).restricao,robo(3).restricao,robo(4).restricao]
for k=1:nm+nf
%     if(k<=nf)
%         nofixo(k).e=0;
%         nofixo(k).ev=0;
%     else
        mobile_node(k-nf).e=0;
        mobile_node(k-nf).ev=0;
%     end
        
end
% 

% Armazenamento dos estados
% robo_evento=0;
v = [];
for k = 1:nm+nf
    if(k<=nf)
        v = [v nofixo(k).est];
    else
        v = [v mobile_node(k-nf).est];
    end
end
    EST = [EST, v'];
    v
    
    pause(0.5)
end % Fim do while - simula��o


% Plot estado e eventos dos rob�s

% figure(2) % Estado Evento
% % imshow(uint8(MG*255));
% title('Estados')
% plot(EVENTO(1,:),'ob')
% hold on
% plot(EST(1,:),'.b')
% hold off
% legend('Evento R1','Estado R1')
% % title('Rob� 1')
% 
% figure(3) % Plot Mapa Explorado
% plot(EVENTO(2,:),'ob')
% hold on
% plot(EST(2,:),'.b')
% hold off
% legend('Evento R2','Estado R2')
% % title('Evento Rob� 2')
% 
% figure(4) % Plot Mapa Explorado
% plot(EVENTO(3,:),'ob')
% hold on
% plot(EST(3,:),'.b')
% hold off
% legend('Evento R3','Estado R3')
% % title('Rob� 3')
% 
% figure(5) % Plot Mapa Explorado
% plot(EVENTO(4,:),'ob')
% hold on
% plot(EST(4,:),'.b')
% hold off
% legend('Evento R4','Estado R4')
% % title('Rob� 4')
% axis auto

%% Plot Fun��es de Lyapunov individuais dos rob�
% figure(6) % Fun�ao de Lyapunov Rob� 1
% hold on
% plot(tempo, robo(1).Lyap,'-b')
% % figure(7) % Fun�ao de Lyapunov Rob� 2
% plot(tempo, robo(2).Lyap,'-.g')
% % figure(8) % Fun�ao de Lyapunov Rob� 1 3 
% plot(tempo, robo(3).Lyap,'--k')
% % figure(9) % Fun�ao de Lyapunov Rob� 4
% plot(tempo, robo(4).Lyap,':r')
% legend('V_R_1','V_R_2','V_R_3','V_R_4')
% xlabel('t [s]');
% ylabel('V');
% title('Fun��o de Lyapunov')
% hold off
% % end

% M�tricas



% % N�mero de comunica��es agente-supervis�rio
% N1 = i*n;
% % Numero de comunica��o supervis�rio agente por rob�
% N11=length(find(EVENTO(1,:)~=0));
% N22=length(find(EVENTO(2,:)~=0));
% N33=length(find(EVENTO(3,:)~=0));
% % N�mero de comunica��es supervis�rio-agente
% N2=N11+N22+N33;
% 
% disp(['Com A-S:',num2str(N1), ' (total 3 rob�s, ',num2str(i), ' cada)']);
% disp('A-S pode ser monitorada')
% disp(['Com S-A:',num2str(N2), ' (total 3 rob�s) ']);
% disp(['Com S-A R1:',num2str(N11)]);
% disp(['Com S-A R2:',num2str(N22)]);
% disp(['Com S-A R3:',num2str(N33)]);
% 
% disp('########');
% disp(['cont_evento_SR:',num2str(cont_evento_SR)]);
% disp(['cont_evento_RS:',num2str(cont_evento_RS)]);

% =========================================================================
% save('experimento.mat')% Salva Experimento