function [u1f,v1f,u2f,v2f] = RANSAC_caseiro(u1,v1,u2,v2)
%RANSAC caseiro

%Calcular distancia entre todos os pontos
[m, n] = size(u1)
for i= 1:n
    X = [u1(i),v1(i);u2(i),v2(i)];
    d(i) = pdist(X,'euclidean');
end
%Calcular a media da distancia 
M = mean(d);
th = 0.65;
%elimonar todos os pontos que tiverem mais que x fora da media
j = 1;
for i=1:n
    X = [u1(i),v1(i);u2(i),v2(i)];
    d(i) = pdist(X,'euclidean');
    if d(i)<M+th*M && d(i)>M-th*M;
        u1f(j) = u1(i);
        v1f(j) = v1(i);
        u2f(j) = u2(i);
        v2f(j) = v1(i);
        j=j+1;
    end
end
end