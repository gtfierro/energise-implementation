clc; clear all; close all;

test = '33bal_T8-1_QmagPdel'; % change test/file name here
pathname = 'C:\Users\Leo\Documents\energise_ipynb\';
csv = '.csv';

filename = strcat(pathname,test,csv);
T = readtable(filename);
T(:,1) = [];

mat = '.mat';
matfile = strcat(pathname,test,mat);
save(matfile, 'T')