% ~~ INSTRUCTIONS ~~
% change name of .mat loaded in (line 20)
% Run once, there will be an error, open the FileData 1x1 struct and look
% at the name of the field (should be 'grid_result_XX')
% change the argument of FileData.___arg___ (line 22)to match this field
% from previous step
% change txt, plot_on, and  len_min values as desired then run

clc; clear all; close all;

txt = 1; % turn write to txt on/off
plot_on = 1; % turn plot on/off
len_min = 50; % INPUT length of sim in minutes

% convert .mat of monitoring output to a .xlsx with row header label so can
% interpret the results
% headers = readtable('ORT_monitoring_template.xlsx');
% headers = table2array(headers);
%copyfile 'IEEE13NF_bal_monitoring_template.xlsx' 'grid_result_1.xlsx'; % create copy of 13nf bal monitoring template
folder = 'C:\Users\Leo\Documents\energise_results\';
FileData = load(strcat(folder,'T3_1-R11_ORT.mat')); % contains grid_result matrixs ** Need to change according to .MAT file
%xlswrite('grid_result_1.xlsx',FileData.grid_result_1',1,'A2')
a=FileData.grid_result_25'; % rows are across time, cols are across nodes/phases ** May need to change according to field of FileData

a(1,:)=[];
a(:,2:34)=[];
sz = size(a);
a(:,60:sz(2))=[];


a2 = a;
a2(:,1) = a2(:,1)-a2(1,1);
sz = size(a2);

%a2(:,1) = a2(:,1)*1000;
%dur = milliseconds(a2(:,1));
%a2(:,1) = [];

ts = 0; 
tf = 60*len_min;
tstep = 1;
trange = (ts:tstep:ts+tf-1);

ai = interp1(a2(:,1),a2(:,1:sz(2)),trange);

% ~~ FOR TIMETABLE SYNC ~~ %
% tstart = seconds(0);
% tstep = seconds(1);
% tend = minutes(len_min);
% ~~~~~~~~~~~~~~~~~~~~~~ %

% ~~ FOR DROP DOUBLES ~~ %
% sz = size(a);
% 
% idx_drop = [];
% for i = 2:sz(1)
%     if (a(i,1) - a(i-1,1)) < 0.05
%         idx_drop = [idx_drop,i];
%     end
% end
% a(idx_drop,:) = [];
% ~~~~~~~~~~~~~~~~~~~~~~ %


% node labels:
% first 3? angles of what?
% LD_671/P1	LD_671/P2	LD_671/P3	LD_675/P1	LD_675/P2	LD_675/P3	LD_652/P1	
% LD_692/P1	LD_671/Q1	LD_671/Q2	LD_671/Q3	LD_675/Q1	LD_675/Q2	LD_675/Q3
% LD_652/Q1	LD_692/Q1	650_a/Vmag	650_b/Vmag	650_c/Vmag	650_a/Vang	650_b/Vang	
% 650_c/Vang	675_a/Vmag	675_b/Vmag	675_c/Vmag	675_a/Vang	675_b/Vang	
% 675_c/Vang	671_a/Vmag	671_b/Vmag	671_c/Vmag	671_a/Vang	671_b/Vang	
% 671_c/Vang	652_a/Vmag	652_a/Vang	692_a/Vmag	692_b/Vmag	692_c/Vmag
% 692_a/Vang	692_b/Vang	692_c/Vang	LD_675/Imag1	LD_675/Imag2	
% LD_675/Imag3	LD_675/Iang1	LD_675/Iang2	LD_675/Iang3	LD_671/Imag1
% LD_671/Imag2	LD_671/Imag3	LD_671/Iang1	LD_671/Iang2	LD_671/Iang3
% LD_652/Imag1	LD_652/Iang1	LD_692/Imag1	LD_692/Iang1


if txt == 1
    T = array2table(ai);
    writetable(T,strcat(folder,'myData.txt'),'Delimiter','tab') 
end

i = 1; %initialize starting column
if plot_on == 1 
    figure; plot(ai(:,i+1:i+3),'LineWidth',2); legend('671a','671b','671c'); title('P');
    figure; plot(ai(:,i+4:i+6),'LineWidth',2); legend('675a','675b','675c'); title('P');
    figure; plot(ai(:,i+7),'LineWidth',2); legend('652a'); title('P');
    figure; plot(ai(:,i+8),'LineWidth',2); legend('692a'); title('P');

    figure; plot(ai(:,i+9:i+11),'LineWidth',2); legend('671a','671b','671c'); title('Q');
    figure; plot(ai(:,i+12:i+14),'LineWidth',2); legend('675a','675b','675c'); title('Q');
    figure; plot(ai(:,i+15),'LineWidth',2); legend('652a'); title('Q');
    figure; plot(ai(:,i+16),'LineWidth',2); legend('692a'); title('Q');

    figure; plot(ai(:,i+17:i+19),'LineWidth',2); legend('650a','650b','650c'); title('Vmag');
    figure; plot(ai(:,i+20:i+22),'LineWidth',2); legend('650a','650b','650c'); title('Vang');
    figure; plot(ai(:,i+23:i+25),'LineWidth',2); legend('675a','675b','675c'); title('Vmag');
    figure; plot(ai(:,i+26:i+28),'LineWidth',2); legend('675a','675b','675c'); title('Vang');
    figure; plot(ai(:,i+29:i+31),'LineWidth',2); legend('671a','671b','671c'); title('Vmag');
    figure; plot(ai(:,i+32:i+34),'LineWidth',2); legend('671a','671b','671c'); title('Vang');
    figure; plot(ai(:,i+35),'LineWidth',2); legend('652a'); title('Vmag');
    figure; plot(ai(:,i+36),'LineWidth',2); legend('652a'); title('Vang');
    figure; plot(ai(:,i+37:i+39),'LineWidth',2); legend('692a','692b','692c'); title('Vmag');
    figure; plot(ai(:,i+40:i+42),'LineWidth',2); legend('692a','692b','692c'); title('Vang');

    figure; plot(ai(:,i+43:i+45),'LineWidth',2); legend('675a','675b','675c'); title('Imag');
    figure; plot(ai(:,i+46:i+48),'LineWidth',2); legend('675a','675b','675c'); title('Iang');
    figure; plot(ai(:,i+49:i+51),'LineWidth',2); legend('671a','671b','671c'); title('Imag');
    figure; plot(ai(:,i+52:i+54),'LineWidth',2); legend('671a','671b','671c'); title('Iang');
    figure; plot(ai(:,i+55),'LineWidth',2); legend('652a'); title('Imag');
    figure; plot(ai(:,i+56),'LineWidth',2); legend('652a'); title('Iang');
    figure; plot(ai(:,i+57),'LineWidth',2); legend('692a'); title('Imag');
    figure; plot(ai(:,i+58),'LineWidth',2); legend('692a'); title('Iang');
end
