function [M] = plotonline2(M,L,Pos_n,destino,nf,nm,tam_maior_agr,G,indice_sink)

global K afastamento lo;

%     destino = [ 0 ; 0 ];
%     [G,nG,selec] = agrupamentos(L,Pos,nf,nm);
    C = size(M,1);
    [c,l,z] = size(M);
    M = zeros(c,l,z);
    % número total de nós 
    N = nf+nm; 
    destino = ceil(destino); % arredondando o destino
    
    
    % plota fronteira;
    Pos_G = [];
    for i = 2:tam_maior_agr+1
       Pos_G = [Pos_G Pos_n(:,G(i))];
    end
    

    
    
    [Mx,fronteira] = occupancy_grid_mapping_fronteira(M,L,Pos_G,lo);
    
    %% Plot Navegação
    subplot(1,2,2)
    %Exibição e classificação dos nós   
%     plot(Pos(1,1:nf),Pos(2,1:nf),'db');  % plot da distribuição
    scatter(Pos_n(1,1:nf), Pos_n(2,1:nf),20,'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    axis([-10 C+(10) -10 C+(10)])
    hold on
%     plot(Pos(1,nf+1:end),Pos(2,nf+1:end),'sr');  % plot da distribuição
    scatter(Pos_n(1,nf+1:end), Pos_n(2,nf+1:end),20,'MarkerEdgeColor','r', 'MarkerFaceColor','r')
    scatter(Pos_n(1,indice_sink), Pos_n(2,indice_sink),20,'MarkerEdgeColor','c', 'MarkerFaceColor','c')
    scatter(destino(1),destino(2),20,'MarkerEdgeColor','g', 'MarkerFaceColor','g') 
    % limites da imagem
    L1 = zeros(1,c);
    L2 = c.*ones(1,c);
    L3 = 1:1:c;
    plot(L1,L3,'k');
    plot(L2,L3,'k');
    plot(L3,L1,'k');
    plot(L3,L2,'k');
    for i=1:length(Pos_G(1,:))  
      x = -L:1:L;
      c_plus = sqrt(L^2-x.^2);
      c_minus = -sqrt(L^2-x.^2);
      hold on
      if(i<=length(Pos_G(1,:)))
        plot(x+Pos_G(1,i),c_plus+Pos_G(2,i),':b')
        plot(x+Pos_G(1,i),c_minus+Pos_G(2,i),':b')
      else
        plot(x+Pos_G(1,i),c_plus+Pos_G(2,i),':r')
        plot(x+Pos_G(1,i),c_minus+Pos_G(2,i),':r')
      end
    end
    legend('Nó fixo','Nó móvel','Location','southoutside')
    % Definição dos nomes dos nós (alcance 0-999)
    Nos=[];
    for i=1:nf
        if(nf<10) % restrito até 9 nós
            Nos = [Nos; ['S0',int2str(i)]]; 
        elseif(nf>=10 && nf<100) % restrito até 99 nós
            if(i<10)
                Nos = [Nos; ['S0',int2str(i)]];
            else
                Nos = [Nos; ['S',int2str(i)]];
            end
        else % restrito até 999 nós
            if(i<10)
                Nos = [Nos; ['S00',int2str(i)]];
            elseif(i>=10 && i<100)
                Nos = [Nos; ['S0',int2str(i)]];
            end
        end
        text(Pos_n(1,i),Pos_n(2,i)-4,char(Nos(i,:)));
    end
    for i=nf+1:N
        if(N<10) % restrito até 9 nós
            if(i-nf<10)
                Nos = [Nos; ['R0',int2str(i-nf)]];
            else
                Nos = [Nos; ['R',int2str(i-nf)]];
            end
        elseif(N>=10 && N<100) % restrito até 99 nós
            if(i-nf<10)
                Nos = [Nos; ['R0',int2str(i-nf)]];
            else
                Nos = [Nos; ['R',int2str(i-nf)]];
            end
        else % restrito até 999 nós
            if(i-nf<10)
                Nos = [Nos; ['R00',int2str(i-nf)]];
            elseif(i>=10 && i<100)
                Nos = [Nos; ['R0',int2str(i-nf)]];
            end
        end
        text(Pos_n(1,i),Pos_n(2,i)-4,char(Nos(i,:)));
    end

    % Plot área de interesse
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

    hold on
    voronoi(Pos_G(1,:),Pos_G(2,:));
   
    
%     Plot Fronteira
    plot(fronteira(1,:),C-fronteira(2,:),'.k')
    
    hold off

    
    
    
    
    
    
    
    subplot(1,2,1) 
    
    [M2,bid] = voronoi_w(M,Pos_G,K,nf,afastamento); % VORONOI
    
    
%     plot fonteira
%     M2(fronteira(1,:),fronteira(2,:),:) = 0;
    
    
    imshow(uint8(M2));
    nump = size(Pos_n,2);
    plabels = arrayfun(@(n) {sprintf('%d', n)}, (1:nump)');
    hold on
    % plot nodes
    scatter(Pos_n(1,1:nf), size(M,1)-Pos_n(2,1:nf),20,'MarkerEdgeColor','b', 'MarkerFaceColor','b')
    scatter(Pos_n(1,nf+1:end), size(M,1)-Pos_n(2,nf+1:end),20,'MarkerEdgeColor','r', 'MarkerFaceColor','r')
    scatter(Pos_n(1,indice_sink), Pos_n(2,indice_sink),20,'MarkerEdgeColor','c', 'MarkerFaceColor','c')
    % plot ID of nodes
    Hpl = text(Pos_n(1,:)+15, size(M,1)-Pos_n(2,:), plabels, 'FontWeight', ...
          'bold', 'HorizontalAlignment','center', ...
          'BackgroundColor', 'none');
    
      
    scatter(destino(1),  size(M,1) - destino(2),20,'MarkerEdgeColor','g', 'MarkerFaceColor','g')  
    hold off 


end