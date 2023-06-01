




function [ Pos_n , Pos_G , M , bid , G , indice , indice_sink ] = inicializacao(nm,nf)

    global mapa afastamento K L lo;

    N= nm+nf;
    C = size(mapa,1); % Comprimento da �rea quadrada em estudo
    
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
    M = zeros(size(mapa,1),size(mapa,2),3);
    [c,l,z] = size(M);
    M2 = im2double(M);

    % inicialization of M2
    for l=1:size(M2,1) % Y de cima para baixo
        for c=1:size(M2,2) % X
            M2(l,c,1) = lo;
            M2(l,c,2) = lo; 
            M2(l,c,3) = lo; 
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
    % //////////////////////////////////////////////////
    network_nodes = maior; % number of node of the network

    % plota fronteira;
    Pos_G = [];
    for k = 2:network_nodes+1
       Pos_G = [Pos_G Pos_n(:,G(indice).g(k))];
    end
    
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
        [maxx,indice_sink] = max(cont);
    else
        maxx=0;
        indice_sink = 2;
    end
    % //////////////////////////////////////////////////////
    indice_sink = G(indice).g(indice_sink);

    
    disp(['N� Sink: ', num2str(indice_sink)])

    %Sorteio dos n�s m�veis dentro do maior agrupamento
    x1 = 0; y1 = 0;
    flag0 = 0;
    for ii = 1:nm
        flag0 = 0;
        while(~flag0)
            for q = 2:1:length(G(indice).g) % plota �rea de cobertura
                x1 = C*rand;
                y1 = C*rand;
                if(sqrt((Pos_n(2,G(indice).g(q))-(y1))^2+((Pos_n(1,G(indice).g(q))-(x1))^2))<L && ~flag0)
    %             if(sqrt((Pos_n(2,G(indice).g(q))-(y1))^2+((Pos_n(1,G(indice).g(q))-(x1))^2))<L && ~flag0 && (x1>L) && (x1<(C-L)) && (y1>L) && (y1<(C-L)))    
                   Pos_n = [Pos_n [x1; y1]]; % rand uniforme
                   flag0 = 1;
                end
            end
        end
    end

    %Exibi��o e classifica��o dos n�s 
    subplot(1,2,2)

    scatter(Pos_n(1,1:nf), Pos_n(2,1:nf),20,'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    % plot(Pos_n(1,1:nf),Pos_n(2,1:nf),'db');  % plot da distribui��o n�s m�veis
    axis([-10 C+(10) -10 C+(10)])
    hold on
    scatter(Pos_n(1,indice_sink), Pos_n(2,indice_sink),20,'MarkerEdgeColor','c', 'MarkerFaceColor','c')
    scatter(Pos_n(1,nf+1:end), Pos_n(2,nf+1:end),20,'MarkerEdgeColor','r', 'MarkerFaceColor','r')



    % plot(Pos_n(1,nf+1:end),Pos_n(2,nf+1:end),'sr');  % plot da distribui��o
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

    % Plot �rea de interesse
    hold on
    x = afastamento:1:3*afastamento;
    y = x;
    y1 = afastamento*ones(1,length(x));
    y2 = 3*afastamento*ones(1,length(x));
    xv1 = afastamento*ones(1,length(x));
    xv2 = 3*afastamento*ones(1,length(x));
    plot(x,y1,'k');
    plot(x,y2,'k');
    plot(xv1,y,'k');
    plot(xv2,y,'k');
    hold off

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



    Pos_G = []; % Aloca os n� m�veis e dentro do maior agrupamento para a defini��o de nova fronteira
    for k = 1:length(G(indice).g)-1
        Pos_G = [Pos_G Pos_n(:,G(indice).g(k+1))];
    end

    hold on
    voronoi(Pos_G(1,:),Pos_G(2,:));
    hold off

    [M2,bid] = voronoi_w(M,Pos_G,K,nf,afastamento); % VORONOI

    subplot(1,2,1)

    imshow(uint8(M2));
    nump = size(Pos_n,2);
    plabels = arrayfun(@(n) {sprintf('%d', n)}, (1:nump)');
    hold on
    % plot nodes
    scatter(Pos_n(1,1:nf), size(M,1)-Pos_n(2,1:nf),20,'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    scatter(Pos_n(1,nf+1:end), size(M,1)-Pos_n(2,nf+1:end),20,'MarkerEdgeColor','r', 'MarkerFaceColor','r')
    scatter(Pos_n(1,indice_sink), size(M,1)- Pos_n(2,indice_sink),20,'MarkerEdgeColor','c', 'MarkerFaceColor','c')
    % plot ID of nodes
    Hpl = text(Pos_n(1,:)+15, size(M,1)-Pos_n(2,:), plabels, 'FontWeight', ...
          'bold', 'HorizontalAlignment','center', ...
          'BackgroundColor', 'none');

    hold off 





end