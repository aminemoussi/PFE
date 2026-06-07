%
% Code Provided Courtesy of Hichem Bessafa
%  
% H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani, 
% "Nonlinear Observer Design Methods Based on High-Gain Methodology and LMIs with Application to Vehicle Tracking", 
% 2023 American Control Conference (ACC), San Diego, CA, USA, 2023,
% pp. 4735-4740, doi: 10.23919/ACC55779.2023.10156219.
%

function [out] = Proj(in,verticies)
if(length(in) ~= length(verticies))
   error('Something went wrong check the dimension of input vector with the verticies')
end 
for i=1:length(in)
mini=verticies{i}(1);
maxi=verticies{i}(2);
out(i)=min(maxi, max(mini, in(i)));
end
out=reshape(out,size(in));
end

