%
% Code Provided Courtesy of Hichem Bessafa
%  
% H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani, 
% "Nonlinear Observer Design Methods Based on High-Gain Methodology and LMIs with Application to Vehicle Tracking", 
% 2023 American Control Conference (ACC), San Diego, CA, USA, 2023,
% pp. 4735-4740, doi: 10.23919/ACC55779.2023.10156219.
%

function value = Phi(x,v)
value=[-(1/v^2*(-x(5)*x(3)+x(2)*x(6)))^2*x(2); (1/v^2*(-x(5)*x(3)+x(2)*x(6)))^2*x(5)];
end

