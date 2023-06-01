function [fronteira2] = ordenar_fronteira3(fronteira)
%     max = size(fronteira,2); % tamanho da fronteira
%     start = ceil(rand(1)*max); % ponto de início
%     fronteira2 = fronteira(:,start); % atribuição inicial
%     seq_fronteira = start; % sequência inicial
%     x1 = fronteira(1,start);
%     y1 = fronteira(2,start);
%     fronteira(:,start) = []; % remove o elemento
%     while(size(fronteira2,2)<max)
% %         min = size(fronteira,2) + 1; % definition initial
% %         dist_min = 10^5; % definition initial
%         x = x1 - fronteira(1,:);
%         y = y1 - fronteira(2,:);
%         x = x.^2;
%         y = y.^2;
%         d = sqrt(x+y);
%         [val,idc_min] = min(d);
%         fronteira2 = [fronteira2 fronteira(:,idc_min)];
%         fronteira(:,idc_min) = []; % remove o elemento
% %         seq_fronteira = [seq_fronteira idc_min];
%          
%     end
    start = 1; % ponto de início
    fronteira2 = fronteira(:,start); % atribuição inicial
    x1 = fronteira(1,start);
    y1 = fronteira(2,start);
    fronteira(:,start) = []; % remove o elemento
    x = x1 - fronteira(1,:);
    y = y1 - fronteira(2,:);
    x = x.^2;
    y = y.^2;
    d = sqrt(x+y);
    [d2,ind] = sort(d);
    fronteira2 = [fronteira2 fronteira(:,ind)];

end