function [P,V,W]=robot(Pos,Pdes)

% Inputs
% Pos estado atual do robô Pos=[x,y,theta] (Coordenada local)
% Pdes - Posição de destino

%Outputs
% P - Pose do robô após ação
% V - velocidade linear
% W - velocidade angular


global tamos Vmax Wmax;

d = 16; % diameter of robot TURTLEBOT3 BURGER
raio_robo = d/2; % length between the whell and center of robot [cm]
r = 3.3; % wheel diameter [cm]
Vmax = 22; % m/s
Wmax = 2.84; % rad/s 
if(sqrt((Pdes(1,1)-Pos(1,end))^2 + (Pdes(2,1)-Pos(2,end))^2)>=3)

    % Início controlador 
    [V,W] = controle_e_navegacao1(Pos,Pdes); % Controlador de seguir fronteira
    % Fim controlador

    Ksir = [V ; 0 ; W];

    R = [cos(Pos(3)) sin(Pos(3)) 0 ; -sin(Pos(3)) cos(Pos(3)) 0 ; 0 0 1]; % matriz de rotação
    R_I = [cos(Pos(3)) -sin(Pos(3)) 0 ; sin(Pos(3)) cos(Pos(3)) 0 ; 0 0 1]; % matriz de rotação inversa
    % VW = [VW [Ksir(1); Ksir(3)]];
    Ksi_I = R_I*[Ksir(1) ; 0 ; Ksir(3)]; % velocidade desejada no SC do ambiente
    Pos = Pos + Ksi_I*tamos; % atualização da posição do robô
    % converte theta para -pi a pi
    if Pos(3) > pi, Pos(3) = Pos(3) - 2*pi; end
    if Pos(3) < -pi, Pos(3) = Pos(3) + 2*pi; end
    P = Pos; % atualização do vetor de posição
    
else
    
    V=0;
    W=0;
    P = Pos; % Vetor que armazena a posição do robô 
end

