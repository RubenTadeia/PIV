%Projecto
clc;clear all;close all;
run C:\Users\João\Documents\SOFTWARE\Matlab2015\vlfeat-0.9.20-bin\vlfeat-0.9.20\toolbox\vl_setup;
nim = 5;%numero de imagens
load calib_asus.mat;
% K = Depth_cam.K;
K=[525 0 319.5;
    0 525 239.5;
    0 0 1];

im1=imread('rgb_image_1.png');
im2=imread('rgb_image_2.png');
im3=imread('rgb_image_3.png');
im4=imread('rgb_image_4.png');
im5=imread('rgb_image_5.png');
load depth_1.mat;
depth1 = depth_array;
load depth_2.mat;
depth2 = depth_array;
load depth_3.mat;
depth3 = depth_array;
load depth_4.mat;
depth4 = depth_array;
load depth_5.mat;
depth5 = depth_array;
clear depth_array;
im1g=rgb2gray(im1);
im2g=rgb2gray(im2);
im3g=rgb2gray(im3);
im4g=rgb2gray(im4);
im5g=rgb2gray(im5);
%%
%PointClouds
figure(1);
imagesc([im1 im2 im3]);
figure(2);
imagesc([depth1 depth2 depth3]);

xyz1=get_xyzasus(depth1(:),[480 640],1:640*480,K,1,0);
xyz2=get_xyzasus(depth2(:),[480 640],1:640*480,K,1,0);
xyz3=get_xyzasus(depth3(:),[480 640],1:640*480,K,1,0);

cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);
cl12 = [cl1;cl2];
cl3=reshape(im3,480*640,3);

p1=pointCloud(xyz1,'Color',cl1);
p2=pointCloud(xyz2,'Color',cl2);
p3=pointCloud(xyz3,'Color',cl3);

figure(3)
showPointCloud(p1);
figure(4)
showPointCloud(p2);
figure(5)
showPointCloud(p3);
%%
%Invalid Points due to depth camera
m1=depth1>0;
m2=depth2>0;
m3=depth3>0;

imaux1=double(repmat(m1,[1,1,3])).*double(im1)/255;%?
imaux2=double(repmat(m2,[1,1,3])).*double(im2)/255;
imaux3=double(repmat(m3,[1,1,3])).*double(im3)/255;
%%
%KeyPoints Detection
PeakThresh= 1;%REVER
edge_thresh = 3;%REVER
ws = 2;
[F, D] = vl_sift(single(im1g), 'edgethresh', edge_thresh, 'PeakThresh',PeakThresh ,'WindowSize', ws);
% [F, D] = vl_sift(single(im1g));
figure(6);
imagesc(imaux1);
plotKeyPoints(F);


[F2, D2] = vl_sift(single(im2g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);
% [F, D] = vl_sift(single(im2g));
figure(7);
imagesc(imaux2);
plotKeyPoints(F2);

[F3, D3] = vl_sift(single(im3g), 'edgethresh', edge_thresh, 'PeakThresh', PeakThresh, 'WindowSize', ws);
% [F, D] = vl_sift(single(im3g));
figure(8);
imagesc(imaux3);
plotKeyPoints(F3);
%%
%Find Matches 1 2 
th = 7.5;%REVER
[MATCHES, scores] = vl_ubcmatch(D, D2, th); 
[m, n] = size(MATCHES);
for i=1:n
u1(i) = F(1,MATCHES(1,i));
v1(i) = F(2,MATCHES(1,i));
u2(i) = F2(1,MATCHES(2,i));
v2(i) = F2(2,MATCHES(2,i));
end
n

%Find Matches 2 3 
th = 7.5;%REVER
[MATCHES2, scores] = vl_ubcmatch(D2, D3, th); 
[m2, n2] = size(MATCHES2);
for i=1:n2
u3(i) = F2(1,MATCHES2(1,i));
v3(i) = F2(2,MATCHES2(1,i));
u4(i) = F3(1,MATCHES2(2,i));
v4(i) = F3(2,MATCHES2(2,i));
end
n2
%%
%Print Matches antes do ransac 1 2
figure(6);imagesc(imaux1);hold on;plot(u1,v1,'*r');hold off;
figure(7);imagesc(imaux2);hold on;plot(u2,v2,'*r');hold off;
ind1=sub2ind([480 640],uint64(v1'),uint64(u1'));
ind2=sub2ind([480 640],uint64(v2'),uint64(u2'));

[u1,v1,u2,v2] = RANSAC_caseiro(u1,v1,u2,v2);

%Print Matches depois do ransac
figure(25);imagesc(imaux1);hold on;plot(u1,v1,'*r');hold off;
figure(26);imagesc(imaux2);hold on;plot(u2,v2,'*r');hold off;
ind1=sub2ind([480 640],uint64(v1),uint64(u1));
ind2=sub2ind([480 640],uint64(v2),uint64(u2));
[m, n] = size (u1);

%%
%Print Matches antes do ransac 2 3
figure(7);imagesc(imaux2);hold on;plot(u3,v3,'*g');hold off;
figure(9);imagesc(imaux3);hold on;plot(u4,v4,'*g');hold off;
ind3=sub2ind([480 640],uint64(v3),uint64(u3));
ind4=sub2ind([480 640],uint64(v4),uint64(u4));

[u3,v3,u4,v4] = RANSAC_caseiro(u3,v3,u4,v4);

%Print Matches depois do ransac 2 3
figure(25);imagesc(imaux2);hold on;plot(u3,v3,'*r');hold off;
figure(26);imagesc(imaux3);hold on;plot(u4,v4,'*r');hold off;
ind3=sub2ind([480 640],uint64(v3),uint64(u3));
ind4=sub2ind([480 640],uint64(v4),uint64(u4));
[m, n2] = size (u3);
%%
%Calculo Matriz Rotacao 1 2
cent1=mean(xyz1(ind1,:))';
cent2=mean(xyz2(ind2,:))';
pc1=xyz1(ind1,:)'-repmat(cent1,1,n);
pc2=xyz2(ind2,:)'-repmat(cent2,1,n);
[a b c]=svd(pc2*pc1');
R12=a*c';

%Calculo Matriz Rotacao 2 3
cent3=mean(xyz2(ind3,:))';
cent4=mean(xyz3(ind4,:))';
pc3=xyz2(ind3,:)'-repmat(cent3,1,n2);
pc4=xyz3(ind4,:)'-repmat(cent4,1,n2);
[a b c]=svd(pc4*pc3');
R23=a*c';

%%
%Juntar Pointclouds 1 2
xyzt1=R12*(xyz1'-repmat(cent1,1,length(xyz1)));%Estou a juntar a pointCould1 à pointCloud2 no ref 2!
xyzt2=xyz2'-repmat(cent2,1,length(xyz2));
T=cent2-R12*cent1;
xyzt12 = [xyzt1 xyzt2];

%Tenho de somar cent2 porque estou a aplicar uma rotacao em xyzt12
%em cujo referencial estava no centro de massa dos pontos pc2(cent2)
xyzt3=(R23*(xyzt12-repmat(cent3,1,length(xyzt12))+repmat(cent2,1,length(xyzt12))));%Estou a juntar a pointCould12 à pointCloud3 no ref 3!
xyzt4=xyz3'-repmat(cent4,1,length(xyz3));
T2=cent2-R23*cent3;%Nao garanto que esta seja a verdadeira translaccao...
%%
%PointCloud Final 1 2
ptotal=pointCloud(xyzt12','Color',cl12);
figure(17);
showPointCloud(ptotal);

%PointCloud Final 2 3
ptotal=pointCloud([xyzt3';xyzt4'],'Color',[cl12;cl3]);
figure(20);
showPointCloud(ptotal);
