x=linspace(0,2*pi,100);
plot(x,sin(x))
%% 
x=-2:0.1:2;
[X, Y]=meshgrid(x);
F = X.*exp(-X.^1-Y.^2);
surf(X, Y, F);
%% 
x=0:30;
odd=(mod(x,2)~=0) 

x(odd)

even=x(~odd)
%% 
myfunction(2,3)
