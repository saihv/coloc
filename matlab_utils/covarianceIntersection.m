function [omega, C] = covarianceIntersection(A,B,H)
    Ai = inv(A);
    Bi = inv(B);
    
    f = inline('1/det(Ai*omega+H''*Bi*H*(1-omega))', ...
             'omega', 'Ai', 'Bi', 'H');
    omega = fminbnd(f,0,1,optimset('Display','off'),Ai,Bi,H);
    C = inv(Ai * omega + H' * Bi * H *(1-omega));
    %nu=b-H*a;
    %W=(1-omega)*C*H'*Bi;
    %c=a+W*nu;
    
end