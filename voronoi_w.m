function [M,tam_buraco] = voronoi_w(M,Pos_n,K,nf,afastamento)

% Output
% M - voronoi e gr�fico de cobertura
% tam_buraco - requerimento (bid)

% Inputs
% M - mapa
% Pos_n distribui��o dos n�s (est�ticos e m�veis)
% K - fator de escala da c�lula do mapa
% nf = n�mero de n�s fixos
% afastamento - margem al�m da �rea de interesse quadrade presente na imagem


global L

    % Modelo WAF de estra��o do diagrama de Voronoi baseado em sua defini��o
    
    [c,l,z] = size(M);


    % defini��o do voronoi e limite da �rea de interesse
    for i = 1:size(M,1) 
        for j = 1:size(M,2)
            dmin = c*l;
            for k = 1:length(Pos_n(1,:)) % SOMENTE DOS N�S FIXOS
                dist = sqrt((Pos_n(1,k)-i)^2+(Pos_n(2,k)-j)^2);
                if(dist<=dmin)
                    dmin=dist;
                    n = k;
                    x1 = size(M,1)+1-j;
                    y1 = i;
                end
            end
            if(x1>=afastamento && x1<(c-afastamento) && y1>=afastamento && y1<(c-afastamento))
                M(x1,y1,2) = (n/nf); % valor de atribui��o de c�lula para o voronoi 
            else
                M(x1,y1,1) = 255; % plota de branco �rea externa � area de interesse
                M(x1,y1,2) = 255;
                M(x1,y1,3) = 255;
            end
        end
    end
    
    % Tamanho de cada �rea
    tam_vor_no = [];
    for i=1:nf
        tam_vor_no = [tam_vor_no; length(find(M(:,:,2)==i/nf))];
    end
%     tam_vor_no
    % Preenchimento cobertura
    for k=1:length(Pos_n(1,:))
        for i = 1:size(M,1) % line
            for j = 1:size(M,2) % colun
                dist_pixel = sqrt((Pos_n(1,k)-i)^2+(Pos_n(2,k)-j)^2);
                if(dist_pixel <= L)
                    M(size(M,2)+1-j,i,1) = k/nf;
                    M(size(M,1)+1-j,i,3) = k/nf;
%                 elseif(dist_pixel>L && i>=afastamento && i<(c-afastamento) && j>=afastamento && j<(c-afastamento))
%                      M(size(M,2)+1-j,i,1) = 0;
%                     M(size(M,1)+1-j,i,3) = 0;
                end
            end
        end
    end
    % Tamanho do buraco de cada �rea
    tam_buraco = [];
    for i=1:length(Pos_n(1,:))
        [x,y] = find(M(:,:,2)==i/nf & M(:,:,1)==0 & M(:,:,3)==0);
        tam_buraco = [tam_buraco; [ length( find(M(:,:,2)==i/nf & M(:,:,1)==0 ) ) round(mean(y)) c-round(mean(x))]];
    end
    for i=1:length(Pos_n(1,:))
        if(tam_buraco(i,1)==0)
            tam_buraco(i,2) = 0;
            tam_buraco(i,3) = 0;
        end
    end
%     tam_buraco
    M(:,:,2) = M(:,:,2)*200; 
    
end