%
% Code Provided Courtesy of Hichem Bessafa
%  
% H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani, 
% "Nonlinear Observer Design Methods Based on High-Gain Methodology and LMIs with Application to Vehicle Tracking", 
% 2023 American Control Conference (ACC), San Diego, CA, USA, 2023,
% pp. 4735-4740, doi: 10.23919/ACC55779.2023.10156219.
%

function dy = HighGainObserver_method2(t,x,A,B,C,T,K,M,y,v,tspan,varargin)
yc=interp1(tspan,y,t);
vc=interp1(tspan,v,t);
dy= A*x+B*Phi(x,vc)+T*K*(yc'-C*x)+T*M*(vc-sqrt(x(2)^2+x(5)^2))+T*M*(0-(x(2)*x(3)+x(5)*x(6)));
if ~isempty(varargin)
dy=Proj(dy,varargin{1});
end
end