function [S,R]=communicationAODV_Supervisor(q,E,R,Pos_n)

global RREP RREQ RERR hop_cont;
 N = size(Pos_n,2); % número de nós     
RREQ(:,:)=zeros(N,7);
RREP(:,:)=zeros(N,7);
RERR(:,:)=zeros(N,7);
% S nó fonte
% D nó de destino
% RT tabela de roteamento
% q = 200; % raio de comunicação
% mapa = imread('mapa2.bmp'); mapa = rgb2gray(mapa); [Ay , Ax] = find(mapa~=255);
% % Posições dos nós fixos
% Pos_n = [200, 400, 600, 800, 200, 400, 600, 800;
%          325, 325, 325, 325, 175, 175, 175, 175];
% % Pos_n = [100, 200, 300, 400, 600;
% %          175, 325, 175, 325, 325];
% Pi1=[300+230;300+275;0]; % posição inícial
% Pi2=[115;375;0]; %115  375 posição inícial
% Pi3=[115;175;0]; % posição inícial
% Pi4 = [100; 550; 0]; % posição inícial
% 
% 
% Pos_n = [Pi1(1:2,1) Pi2(1:2,1) Pi3(1:2,1) Pi4(1:2,1) Pos_n(1:2,:)];


     
     
% plot(Pos_n(1,:),Pos_n(2,:),'or');
% axis([0 1000 -250 750])
% 
% for i=1:N
%     x = -q:5:q;
%     c_plus = sqrt(q^2-x.^2);
%     c_minus = -sqrt(q^2-x.^2);
%     hold on
%     plot(x+Pos_n(1,i),c_plus+Pos_n(2,i),':m')
%     plot(x+Pos_n(1,i),c_minus+Pos_n(2,i),':m')
% end
% 
% % Definição dos nomes dos nós
% Nos=[];
% for i=1:N
%     if(i<10)
%         Nos = [Nos; ['S','0',int2str(i)]];
%     else
%         Nos = [Nos; ['S',int2str(i)]];
%     end
%     text(Pos_n(1,i)+7,Pos_n(2,i)+7,char(Nos(i,:)));
% end
% 
% Nos


%% Disparo de mensagem
% E se comunica com R, somente se RT contém um caminho até R
% E=4; % nó emissor
% R=12; % nó receptor
E2=[]; % nó emissor vizinho
R2=[]; % nó receptor vizinho
% RT=[];
hop_cont=0;
seq = [];
V =[]; % vizinho
s=0;
ip=[];
% RREP = zeros(N,7); %envia RREP para o nó S (7 parametros na mensagem)
% RREQ = zeros(N,7); %envia RREQ para o nó S
% RERR = zeros(N,7); %envia RERR para vixinhos
seq=[];
for i=1:N
    buffer(i)= struct('ip',ip);
    buffer(i).ip=0;
    RT(i) = struct('E',E,'E2',E2,'R2',R2,'R',R,'hop_cont',hop_cont,'s',s,'seq',seq);
    RT(i).E=0;
    RT(i).E2=0;
    RT(i).R2=0;
    RT(i).R=0;
    RT(i).hop_cont=0;
    RT(i).s=0;
    RT(i).seq=0;    
end
% inicalização identificação dos vizinhos
for i=1:N
    if(i~=E)
        dist = sqrt((Pos_n(1,E)-Pos_n(1,i))^2+(Pos_n(2,E)-Pos_n(2,i))^2);
        if(dist<=q);
            V = [V i];
        end
    end
end
if(size(V,2)==0)
%     flag=0;
    RERR(E,:) = [E,E,E,E,hop_cont,2,0]; % send message to he self - s=2 refere-se a msg de erro
    S = 1; %Sem comunicação
else
    % send message to neiborhoods
    for i=1:length(V)
        hop_cont = 0;
        % Há em RT um caminho entre E e R
        RREQ(V(i),:) = [E,E,V(i),R,hop_cont+1,0,0]; % send message to neiborhoods
%         disp(['sent to ', num2str(V(i))]);
        RT(E).E=[RT(E).E E];
        RT(E).E2=[RT(E).E2 E];
        RT(E).R2=[RT(E).R2 V(i)];
        RT(E).R=[ RT(E).R  R];
        RT(E).hop_cont=[RT(E).hop_cont hop_cont+1];
        RT(E).s=[RT(E).s 0];
        RT(E).seq=[RT(E).seq 0];
    end
    S = 2; % robô com comunicação com vizinho
end

flag=1;
while(flag)
    
for ID=1:N % Checa se chegou alguma mensagem 
    if(RREQ(ID,3)==ID || RREP(ID,3)==ID || RERR(ID,3)==ID) % identifica se é receptor de mensagem
        buffer(ID).ip=1; % buffer do id
    end
    if(buffer(ID).ip==1 && RREQ(ID,3)==ID ) % atualiza sua matriz requerimento 
    %     RT = [RT; RREQ(1),RREQ(2),RREQ(3),RREQ(4),RREQ(5)]
        RT(ID).E=[RT(ID).E RREQ(ID,1)];
        RT(ID).E2=[RT(ID).E2 RREQ(ID,2)];
        RT(ID).R2=[RT(ID).R2 RREQ(ID,3)];
        RT(ID).R=[ RT(ID).R  RREQ(ID,4)];
        RT(ID).hop_cont=[RT(ID).hop_cont RREQ(ID,5)];
        RT(ID).s=[RT(ID).s RREQ(ID,6)];
        RT(ID).seq=[RT(ID).seq RREQ(ID,7)];
%         buffer(RREQ(ID,2)).ip=0; % Reseta o buffer do emissor
%         intermediário
        RREQ(ID,:)=zeros(1,7);
    elseif(buffer(ID).ip==1 && RREP(ID,3)==ID) % atualiza sua matriz resposta 
        RT(ID).E=[RT(ID).E RREP(ID,1)];
        RT(ID).E2=[RT(ID).E2 RREP(ID,2)];
        RT(ID).R2=[RT(ID).R2 RREP(ID,3)];
        RT(ID).R=[ RT(ID).R  RREP(ID,4)];
        RT(ID).hop_cont=[RT(ID).hop_cont RREP(ID,5)];
        RT(ID).s=[RT(ID).s RREP(ID,6)];
        RT(ID).seq=[RT(ID).seq RREP(ID,7)];
%         buffer(RREP(ID,2)).ip=0;
        RREP(ID,:)=zeros(1,7);
    elseif(buffer(ID).ip==1 && RERR(ID,3)==ID) % atualiza sua matriz erro 
        RT(ID).E=[RT(ID).E RERR(ID,1)];
        RT(ID).E2=[RT(ID).E2 RERR(ID,2)];
        RT(ID).R2=[RT(ID).R2 RERR(ID,3)];
        RT(ID).R=[ RT(ID).R  RERR(ID)];
        RT(ID).hop_cont=[RT(ID).hop_cont RERR(ID,5)];
        RT(ID).s=[RT(ID).s RERR(ID,6)];
        RT(ID).seq=[RT(ID).seq RERR(ID,7)];
%         buffer(RERR(ID,2)).ip=0;
        RERR(ID,:)=zeros(1,7);
    end

    if((RT(E).s(end)==1 || RT(E).s(end)==2) && flag==1) % indica chegada de resposta ao emissor
        flag=0; % esse flag zero entra no if acima quando RREP ocorre
        buffer(ID).ip=0;
        RREP(:,:)=zeros(N,7);
        RREQ(:,:)=zeros(N,7);
        RERR(:,:)=zeros(N,7);
%         S=3;
        if(RT(E).s(end)==1)
            S=3;
        end
        if(RT(E).s(end)==2)
            S=1;
        end
    end
    
end


for ID=1:N
    % Se chegou alguma mensagem
    if(buffer(ID).ip==1 && RT(ID).R2(end)==RT(ID).R(end)) % é o nó de destino 
%         disp(['sent to ', num2str(RT(ID).E2(end))]);
        RREP(RT(ID).E2(end),:) = [ID,ID,RT(ID).E2(end),RT(ID).E(end),RT(ID).hop_cont(end),1,ID]; %envia REQP para o nó S
        RREQ(ID,:) = zeros(1,7);
        buffer(ID).ip=0;
    elseif(buffer(ID).ip==1 && RT(ID).R2(end)~=RT(ID).R(end)) % Caso não seja o destino
    % ID cria um cópia do RREQ para os seus vizinho que não receberam esta msg em broadcast 
        V=[];
        for i=1:N % Identifica os Vizinhos
            if(i~=ID)
                dist = sqrt((Pos_n(1,ID)-Pos_n(1,i))^2+(Pos_n(2,ID)-Pos_n(2,i))^2);
                if(dist<=q);
                    V = [V i]; % vizinhos
                end
            end
        end
        
        if(size(V,2)==0) % ver este condicional de erro por não encontrar vizinhos 
%                 disp('sent broadcast');
%                 disp(['sent to ', num2str(V(j))]);
                RERR(E,:) = [ID,ID,ID,E,hop_cont,2,0]; % send message to he self - s=2 refere-se a msg de erro
                buffer(ID).ip=0; % VER isso
                S=1;
        else   
            for j=1:length(V) % Envio em broadcast
 
                if(V(j)~=RT(ID).E2(end) && RREQ(V(j),1)==0 && buffer(V(j)).ip==0 && RT(ID).s(end)==0 && RT(V(j)).E(end)~=E && RT(V(j)).R(end)~=R)
%                     disp('sent broadcast');
%                     disp(['sent to ', num2str(V(j))]);
                    hop_cont=hop_cont+1;
                    RREQ(V(j),:) = [E,ID,V(j),R,hop_cont+1,0,0]; % [S,V,D,Hop,Seq] send message to neighborhood
                    buffer(ID).ip=0; % VER isso
                    RREQ(ID,:)=zeros(1,7);                
                    % RREQ contém o endereço de destino, número de sequencia e ID broadcast
                elseif(V(j)~=RT(ID).E2(end) && RREP(V(j),1)==0 && buffer(V(j)).ip==0 && RT(ID).s(end)==1)
%                     disp('sent broadcast');
%                     disp(['sent to ', num2str(V(j))]);
                    hop_cont=hop_cont+1;
                    RREP(V(j),:) = [R,ID,V(j),E,hop_cont,1,ID]; % [S,V,D,Hop,Seq] send message to neighborhood
                    buffer(ID).ip=0; % VER isso
                    RREP(ID,:)=zeros(1,7);
                elseif(V(j)==RT(ID).E2(end) && RREP(V(j),1)==0 && buffer(V(j)).ip==0 && RT(ID).s(end)==0)  % Teste
%                     disp('sent broadcast');
%                     disp(['sent to ', num2str(V(j))]);
                    hop_cont=hop_cont+1;
                    RERR(V(j),:) = [ID,ID,V(j),E,hop_cont,2,ID]; % [S,V,D,Hop,Seq] send message to neighborhood
                    buffer(ID).ip=0; % VER isso
                    RREP(ID,:)=zeros(1,7);

                end
               
            end
        end
        if(buffer(ID).ip==1 && j==length(V)), buffer(ID).ip=0; end
    end % if de recepção
end % id for
    cont=0;
    for i=1:N
        if(size(RT(i).E(:),1)>1), cont=cont+1; end
    end
    if(cont==N-1 && size(RT(R).E(:),1)==1)
        flag=0;
        S=1;
    end
end %while flag
%% Encontrar caminho de acordo com as RTs
R = zeros(N,N);
for i=1:N
    for j=2:size(RT(i).E,2)
        if(i~=RT(i).E2(j))
            R(i,RT(i).E2(j)) = RT(i).E2(j);
        end
    end
    for j=2:size(RT(i).E,2)
        if(i~=RT(i).R2(j))
            R(i,RT(i).R2(j)) = RT(i).R2(j);
        end
    end
end
S;


