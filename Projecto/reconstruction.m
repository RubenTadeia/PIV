% function [pcloud, transforms]=reconstruction( image_names, depth_cam, rgb_cam, Rdtrgb,Tdtrgb)

% image_names - an array of structures with the names of the images. Each element of the array is a structure

% image_name(k).depth - a string with the path of the .mat file with depth data of image k. 
% image_name(k).rgb - a string with the path of a jpeg/png file with rgb image k

% depth_cam - A structure with the intrinsic parameters of the depth camera
% depth_cam.K - a 3x3 matrix with the intrinsic parameters
% depth_cam.DistCoef - a 1x5 array with the lens distortion coeficients (you do not need to use these)

% rgb_cam - A structure with the intrinsic parameters of the rgb camera
% rgb_cam.K - a 3x3 matrix with the intrinsic parameters
% rgb_cam.DistCoef - a 1x5 array with the lens distortion coeficients 

% Rdtrgb and Tdrgb allow  transforming 3D coordinates represented in the depth camera  reference frame to the RGB camera frame
% Rdtrgb - is a 3x3 rotation matrix
% Tdrgb - is a 3x1 vector

clear all;

%% INPUTS %%

% criação dos arrays de imagens e profundidades - funciona para número
% finito de dados, mas se quisermos inserir mais images/ depths, há alguma
% forma de inserir novos dados de forma rápida? (ciclo while, maybe)

depth_array = struct ('depth_1', load('depth_1'), 'depth_2', load('depth_2'), 'depth_3', load('depth_3'), ...
'depth_4', load('depth_4'), 'depth_5', load('depth_5'), 'depth_6', load('depth_6'), 'depth_7', load('depth_7'), ...
'depth_8', load('depth_8'), 'depth_9', load('depth_9')); 

rgb_array = struct('rgb_image_1', imread('rgb_image_1.png'), 'rgb_image_2', imread('rgb_image_2.png'), ...
'rgb_image_3', imread('rgb_image_3.png'), 'rgb_image_4', imread('rgb_image_4.png'), ...
'rgb_image_5', imread('rgb_image_5.png'), 'rgb_image_6', imread('rgb_image_6.png'), ...
'rgb_image_7', imread('rgb_image_7.png'), 'rgb_image_8', imread('rgb_image_8.png'), ...
'rgb_image_9', imread('rgb_image_9.png'));

image_names = struct ('depth', depth_array, 'rgb', rgb_array);