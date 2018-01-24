function [c,C,omega]=CI(a,A,b,B,H)
%
% function [c,C,omega]=CI(a,A,b,B,H)
%
% This function implements the CI algorithm and fuses two estimates
% (a,A) and (b,B) together to give a new estimate (c,C) and the value
% of omega which minimizes the determinant of C. The observation
% matrix is H.
Ai=inv(A);
Bi=inv(B);
% Work out omega using the matlab constrained minimiser function
% fminbnd().
f=inline('1/det(Ai*omega+H''*Bi*H*(1-omega))', ...
'omega', 'Ai', 'Bi', 'H');
omega=fminbnd(f,0,1,optimset('Display','off'),Ai,Bi,H);
% The unconstrained version of this optimisation is:
% omega = fminsearch(f,0.5,optimset('Display','off'),Ai,Bi,H);
% omega = min(max(omega,0),1);
% New covariance
C=inv(Ai*omega+H'*Bi*H*(1-omega));
% New mean
nu=b-H*a;
W=(1-omega)*C*H'*Bi;
c=a+W*nu;


figure(1)
error_ellipse('C',A,'mu',a,'style','r');
hold on
plot(a(1), a(2), 'rx')
error_ellipse('C',B,'mu',b,'style','b'); 
plot(b(1), b(2), 'bx')
error_ellipse('C',C,'mu',c,'style','g'); 
plot(c(1), c(2), 'gx')