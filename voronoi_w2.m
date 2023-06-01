function [M,tam_buraco,cobertura] = voronoi_w2(M,Pos_n,nf,nm,Pos_G1,afastamento,robo_em_mov)

% Output
% M - voronoi e gráfico de cobertura
% tam_buraco - requerimento (bid)

% Inputs
% M - mapa
% Pos_n distribuição dos nós (estáticos e móveis)
% K - fator de escala da célula do mapa
% nf = número de nós fixos
% afastamento - margem além da área de interesse quadrade presente na imagem


global L

    % Modelo WAF de estração do diagrama de Voronoi baseado em sua definição
    
    [c,l,z] = size(M);

    Pos_G = Pos_G1;
    
    Pos_G2 = Pos_n; % todos os nós do maior agrupamento para cálculo de fronteira
%     for i =1:nm % não entra no voronoi os nós móveis
%         if(i~=robo_em_mov)
%             Pos_G = [Pos_G Pos_G1(:,i+nf)];
%         end
%     end
   
    % definição do voronoi e limite da área de interesse
    for i = 1:size(M,1) 
        for j = 1:size(M,2)
            dmin = c*l;
            for k = 1:length(Pos_G(1,:)) % SOMENTE DOS NÓS FIXOS
                dist = sqrt((Pos_G(1,k)-i)^2+(Pos_G(2,k)-j)^2);
                if(dist<=dmin)
                    dmin=dist;
                    n = k;
                    x1 = size(M,1)+1-j;
                    y1 = i;
                end
            end
            if(x1>=afastamento && x1<(c-afastamento) && y1>=afastamento && y1<(c-afastamento))
                M(x1,y1,2) = (n/size(Pos_G,2)); % valor de atribuição de célula para o voronoi 
            else
                M(x1,y1,2) = 255; % plota de branco área externa á area de interesse
%                 M(x1,y1,2) = 255;
%                 M(x1,y1,3) = 255;
            end
        end
    end
    
    M(:,:,1) = 255;
    M(:,:,3) = 255;
    
    
    % Tamanho de cada área
    tam_vor_no = [];
    for i=1:size(Pos_G,2)
        tam_vor_no = [tam_vor_no; length(find(M(:,:,2)==i/size(Pos_G,2)))];
    end

%     % Preenchimento cobertura
%     for k=1:length(Pos_G(1,:))
%         for i = 1:size(M,1) % line
%             for j = 1:size(M,2) % colun
%                 dist_pixel = sqrt((Pos_G(1,k)-i)^2+(Pos_G(2,k)-j)^2);
%                 if(dist_pixel <= L)
%                     M(size(M,2)+1-j,i,1) = k/size(Pos_G,2);
%                     M(size(M,2)+1-j,i,3) = k/size(Pos_G,2);
% %                 elseif(dist_pixel>L && i>=afastamento && i<(c-afastamento) && j>=afastamento && j<(c-afastamento))
% %                      M(size(M,2)+1-j,i,1) = 0;
% %                     M(size(M,1)+1-j,i,3) = 0;
%                 end
%             end
%         end
%     end
    
    % Preenchimento cobertura
    for k=1:length(Pos_G2(1,:))
        for i = 1:size(M,1) % line
            for j = 1:size(M,2) % colun
                dist_pixel = sqrt((Pos_G2(1,k)-i)^2+(Pos_G2(2,k)-j)^2);
                if(dist_pixel <= L)
                    M(size(M,2)+1-j,i,1) = k/size(Pos_G2,2);
                    M(size(M,2)+1-j,i,3) = k/size(Pos_G2,2);
%                 elseif(dist_pixel>L && i>=afastamento && i<(c-afastamento) && j>=afastamento && j<(c-afastamento))
%                      M(size(M,2)+1-j,i,1) = 0;
%                     M(size(M,1)+1-j,i,3) = 0;
                end
            end
        end
    end
    
    % camada 2 - voronoi
    % camada 1 e 3 - cobertura
    [x,y] = find(M(:,:,2)<200 & M(:,:,1)<200); % voronoi + cobertura
    total = find(M(:,:,2)<200); % voronoi
%     cobertura = 100-((abs(aux)/length(total))*100)
    cobertura = (length(x)/length(total))*100;
    
    Maux = M;
%      % Preenchimento cobertura com base no raio do sensor
%     for k=1:length(Pos_n(1,:))
%         for i = 1:size(Maux,1) % line
%             for j = 1:size(Maux,2) % colun
%                 dist_pixel = sqrt((Pos_n(1,k)-i)^2+(Pos_n(2,k)-j)^2);
%                 if(dist_pixel <= L/2)
%                     Maux(size(Maux,2)+1-j,i,1) = k/size(Pos_n,2);
%                     Maux(size(Maux,1)+1-j,i,3) = k/size(Pos_n,2);
% %                 elseif(dist_pixel>L && i>=afastamento && i<(c-afastamento) && j>=afastamento && j<(c-afastamento))
% %                      M(size(M,2)+1-j,i,1) = 0;
% %                     M(size(M,1)+1-j,i,3) = 0;
%                 end
%             end
%         end
%     end
    
    % Tamanho do buraco de cada área
    tam_buraco = [];
    for i=1:length(Pos_G(1,:))
%         [x,y] = find(Maux(:,:,2)==i/size(Pos_n,2) & Maux(:,:,1)==0 & Maux(:,:,3)==0);
%         tam_buraco = [tam_buraco; [ length( find(Maux(:,:,2)==i/size(Pos_n,2) & Maux(:,:,1)==0 ) ) round(mean(y)) c-round(mean(x))]];
        [x,y] = find(Maux(:,:,2)==i/size(Pos_G,2) & Maux(:,:,1)==255 & Maux(:,:,3)==255);
        tam_buraco = [tam_buraco; [ length( find(Maux(:,:,2)==i/size(Pos_G,2) & Maux(:,:,1)==255 ) ) round(mean(y)) c-round(mean(x))]];
    end
    for i=1:length(Pos_G(1,:))
        if(tam_buraco(i,1)==0)
            tam_buraco(i,2) = 0;
            tam_buraco(i,3) = 0;
        end
    end
%     tam_buraco
    M(:,:,2) = M(:,:,2)*200; 
    
end