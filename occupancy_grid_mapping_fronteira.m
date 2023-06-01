% fronteira
% clear all
% close all
% M = imread('mapa.bmp');
% M2 = im2double(M);
% L=20;
% Pos = [40.0 40.0 65.5 70.0; 40.0 60.0 65.5 70.0];
% lo = 0.5; % Ocupa��o inicial



function [M2,fronteira] = occupancy_grid_mapping_fronteira(M2,L,Pos,lo)

% Mapa - M
% Raio de comunica��o - L
% Posi��es dos n�s - Pos

% im2double(M); % Fazer isso antes de usar esta fun��o

resolucao = 1; % cada pixel representa 1cm
fronteira = [];  
  
% M2 = im2double(M);

M = lo*ones(size(M2,1),size(M2,2),3);
% % inicializa��o
%     for l=1:size(M,1) % Y de cima para baixo
%         for c=1:size(M,2) % X
%             M2(l,c,1) = lo;%lo;
%             M2(l,c,2) = lo;%lo;
%             M2(l,c,3) = lo;%lo;
%         end
%     end
M2 = M;
    for p = 1:size(Pos,2)
        for l=1:size(M2,1) % Y de cima para baixo
            for c=1:size(M2,2) % X
                % Atualiza��o da c�lula
%                 xi = resolucao*((2*l)-1)/2; % 5cm cada pixel da matriz de ocupa��o
%                 yi = resolucao*((2*c)-1)/2; 
           
                r = sqrt( (c-Pos(1,p))^2 + ((size(M2,1)-l)-Pos(2,p))^2 ); % dist�ncia da c�lula ao rob�
%                 if (r<=L)               
                    %% Algoritmo do Modelo Inverso do Sensor

                    
%                     if ( r > L )
% %                         lt = 0.5; % c�lula n�o mapeada
% %                         M2(l,c,:) = lt;
%                          M2(l,c,:) = lo;
%                     end
                    % verif. se esta dentro do range do sensor e se a dist�ncia da celula pro rob� esta perto da leitura do sensor
                    if ( r > L  && r<L+resolucao) %(resolucao/2) ) 
                        lt = 0.0; % c�lula ocupada 
%                         M2(l,c,:) = M2(l,c,:) + lt - lo;
%                         p2 = 1-(1/(1+exp(M2(l,c,1))));
%                         lt = log10(p2/(1-p2));
                        M2(l,c,1) = M2(l,c,1) + lt - lo;
                        M2(l,c,2) = M2(l,c,2) + lt - lo;
                        M2(l,c,3) = M2(l,c,3) + lt - lo;
%                         if(M2(l,c,1)==0.0)
%                             fronteira = [fronteira [c;l]];
%                         end
                    end
                    if ( r <= L ) 
                    % verifica se a c�lula se encontra a uma dist�ncia 'menor' que a leitura do sensor
                        lt = 2; % c�lula livre
%                         M2(l,c,:) = M2(l,c,:) + lt - lo;
%                         p2 = 1-(1/(1+exp(M2(l,c,1))));
%                         lt = log10(p2/(1-p2));
                        M2(l,c,1) = M2(l,c,1) + lt - lo;
                        M2(l,c,2) = M2(l,c,2) + lt - lo;
                        M2(l,c,3) = M2(l,c,3) + lt - lo;
                    end

            end
        end   
        
    end
    
   for l=1:size(M2,1) % Y de cima para baixo
       for c=1:size(M2,2) % X
            if(M2(l,c,2)==0.0) % tem que ser o green porque j� uso o azul e vermelho para exibi��o dos n�s
                fronteira = [fronteira [c;l]];
            end
       end
   end
% imshow(M2);


end


% aux = max(max(M2(:,:,1)));
% M2(:,:,:) = M2(:,:,:).*(255.0/aux);
% imshow(M2);