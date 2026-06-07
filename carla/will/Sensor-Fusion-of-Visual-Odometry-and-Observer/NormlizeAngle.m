%
% Code Provided Courtesy of Hichem Bessafa
%  
% H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani, 
% "Nonlinear Observer Design Methods Based on High-Gain Methodology and LMIs with Application to Vehicle Tracking", 
% 2023 American Control Conference (ACC), San Diego, CA, USA, 2023,
% pp. 4735-4740, doi: 10.23919/ACC55779.2023.10156219.
%

function result = NormalizeAngle(angle0)
result=[];
angle=mod(angle0,2*pi);
for i=1:length(angle)
if(angle(i) > pi)
    result =[result angle(i) - 2 * pi];
else
    result =[result angle(i)];
end
end
