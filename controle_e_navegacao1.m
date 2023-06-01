function [V,W] = controle_e_navegacao1(Pos,Pdes)

%% Controlador de seguir caminho/fronteira

%% VARIÁVEIS DISPONÍVEIS (SOMENTE LEITURA!!! NÂO ALTERAR!!!)
global v_sensor s s2 angs Mapa Mapa2 i Vmax Wmax tempo tamos;


    %% Distância até o destino (d)
    d = sqrt((Pdes(1)-Pos(1))^2+(Pdes(2)-Pos(2))^2); % distance between robot and border points
    phi1 = atan2(Pdes(2)-Pos(2),Pdes(1)-Pos(1));  % angle entre alvo e ponto mais próximo do robô
    phi = phi1-Pos(3);
    if phi > pi, phi = phi - 2*pi; end
    if phi < -pi, phi = phi + 2*pi; end
%     disp(['Phi1'])
%     phi*(180/pi)

%     a1 = 0.4;
%     a2 = 8; 
%     K1 = 1.2; %0.8 0.4 0.2
%     K2 = 0.06; %0.8 0.4 >>0.2
%     u = 6;%10; %cm/s
% 
% 
%     %%
%     K1phi = K1/(a1+abs(phi));
%     K2dtil = K2/(a2+abs(dtil)); % como esse controle depende da distância para a parede usei ele ao invés 
%     if K1+(K2*u)<=Wmax
%     % if K1phi+(K2dtil*u)<=Wmax
%     %% Cáculo das velocidades linear (V) e angular (W) do robô (controle de posição final simples)
%     % V = Vmax*K1phi*cos(phi);
%     V = u; % velocidade constante
%     W = -(K1phi*phi)-(u*K2dtil*dtil*sinc(phi/pi));%(sin(phi)/phi))
% %     b = [phi Vpar W];
%     else
%         disp('Error')
%     end

    kv1 = Vmax/3;
    kv2 = 0.12;
    kw1 = Wmax*0.8;
    kw2 = 1;%0.24;

    W = kw1*tanh(kw2*phi);
    if(phi>pi/2), phi = pi/2; end
    if(phi<-pi/2), phi = -pi/2; end  
   
    if(d<=5)
        V = kv1*tanh(kv2*d)*cos(phi);
    else
         V = kv1; %*tanh(kv2*d)*cos(phi);
    end
%     V = kv1; %tanh(kv2*d)*cos(theta_e);




end






    