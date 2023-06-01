clc
close all 
clear all

% figure()
Pos = round(300*rand(16,2));
voronoi(Pos(:,1),Pos(:,2))
[VX,VY] = voronoi(Pos(:,1),Pos(:,2));
hold on
scatter(Pos(:,1),Pos(:,2),20,'MarkerEdgeColor','b', 'MarkerFaceColor','b')
 scatter(VX(:),VY(:),20,'MarkerEdgeColor','r', 'MarkerFaceColor','r')