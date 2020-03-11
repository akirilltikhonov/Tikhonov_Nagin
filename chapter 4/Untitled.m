clc
clear


% Y2 = [1 2 3 4 5]';
% phantomZ = [true true true true true]';
% 
% 
% Number_Z = 5;
% flag = 0;
% 
% for N = 1:1:Number_Z
%     if (phantomZ(N)==1 && N==1 && flag==0)
%         Y2_obs = Y2(N);
%         flag = 1;
%         
%     elseif (phantomZ(N)==1 && N~=1 && flag==0)
%         Y2_obs = Y2(N);
%         flag = 1;
%         
%     elseif (phantomZ(N)==1 && flag==1)
%          Y2_obs = [Y2_obs; Y2(N)];
%          
%     end      
% end
% 
% length(find(phantomZ==true))

% Q = [0.9 0.8 0 0]';
% Q = Q/norm(Q);
% R = rad2deg(q2rotv(Q))

a = [1; 0; 1; 1;];
aa = a/2




    
