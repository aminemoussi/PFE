%
% Code Provided Courtesy of Hichem Bessafa
%  
% H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani, 
% "Nonlinear Observer Design Methods Based on High-Gain Methodology and LMIs with Application to Vehicle Tracking", 
% 2023 American Control Conference (ACC), San Diego, CA, USA, 2023,
% pp. 4735-4740, doi: 10.23919/ACC55779.2023.10156219.
%

function result = AbsoluteAngleDeg(angle0)
result=[];
for i=1:length(angle0)
   result =[result angle0(i) + 360];
end
result=mod(result,360);
end
