function [G,nG,selec] = agrupamentos(L,Pos,nf,nm)


err = 0; % [cm] margem de erro do raio de comunica��o para mais

g0 = 0;
nos = zeros(1,nf+nm);
for i=1:nf+nm
    G(i) = struct('g',g0);
    G(i).g = 0;
    nos(i) = i; 
end
n = 1; % contador
selec = [];
i=1;
while(n<=(nf+nm))
    flag0 = 0; % avalia��o do i no selec
    flag1 = 0; 
    flag2 = 0; % avalia��o do j no selec
    flag3 = 0; % checa se o n� j� foi catalogado
    flag4 = 0;
    for j = 1:nf+nm
        flag2 = 0;
        if(j~=i)
            if(sqrt((Pos(1,i)-Pos(1,j))^2+(Pos(2,i)-Pos(2,j))^2)<(L+err)) % n� i consegue se comunicar com j
                for k=1:length(selec) % checa se i j� est� em algum agrupamento
                    if(i==selec(k))
                        flag0=1;
                    end
                end
                if(flag0==0) % se i n�o est�, � incluso
                    g = i;
                    G(g).g = [G(g).g i];
                    selec = [selec i];   
                end
                for k=1:length(selec) % verifica se j j� est� incluso
                    if(j==selec(k))
                        flag2=1;        
                    end
                end
                if(flag2==0) % se j n�o est�, � incluso
%                     g = i; % altera��o
                    G(g).g = [G(g).g j];
                    selec = [selec j];
                    flag1 = 1;
                end
            end   
        end    
    end
    if(flag1==1 || (flag1==0 && length(selec)>0 && i~=selec(end) )) % Novo n� integrante 
        for k = 1:length(selec)
            if(selec(k)==i && k<length(selec) && flag4==0)
                i = selec(k+1); % seleciona o �ltimo valor de selec como verificador i 
                flag4 = 1;
            end
        end
%     elseif(flag1==0 && i~=selec(end))
%         i = selec(k+1);
    else % quando n�o foi integrado nenhum n�
        flag3 = 0;
        for k = 1:length(nos)
            cont = 0;
            for m =1:length(selec)
                if(nos(k)~=selec(m) && flag3==0)
                    cont = cont + 1;
                end
            end
            if(cont==length(selec) && flag3==0 && length(selec)~=0)
                i=k;
                flag3 = 1;
                g = i;
                G(g).g = [G(g).g i];
                selec = [selec i];
            elseif(cont==length(selec) && flag3==0 && length(selec)==0)
                g = i;
                G(g).g = [G(g).g i];
                selec = [selec i];
                i=i+1;
                g = i;
                flag3 = 1;
            end
        end
    end
    if(~flag3)
        n=n+1; % incrementa o contador
    else
        n = length(selec)+1;
    end
end

nG=0; % n�mero de agrupamentos
% Group=[];
for i=1:nm+nf
   if(length(G(i).g)>=2)
       nG=nG+1;
       G(i).g;
%        for j=1:length(G(i).g)
%            if(G(i).g~=0 && G(i).g~=i)
%                
%        end
   end
end


