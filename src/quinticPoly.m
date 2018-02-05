function quinticPoly(QInput) 
syms a0
syms a1
syms a2
syms a3
syms a4
syms a5

%p = [50, 60, 0, 0, 0, 0];

%start/stop time (seconds)
t_0      = p(1,1);
t_f     = p(1,2);
%start/stop velocity (degrees/s)
v_0      = p(1,3);
v_f     = p(1,4);
%start/stop accletation (degrees/s^2)
alpha_0 = p(1,5);
alpha_f = p(1,6);

%This is the main base matrix
QBaseM = [1,  t_0, (t_0)^2,   (t_0)^3,     (t0)^4,    (t_0)^5; 
          0,    1, 2*(t_0), 3*(t_0)^2,   4*(t0)^3,  5*(t_0)^4; 
          0,    0,       2,   6*(t_0),  12*(t0)^2, 20*(t_0)^3; 
          1,  t_f, (t_f)^2,   (t_f)^3,    (t_f)^4,    (t_f)^5; 
          0,    1, 2*(t_f), 3*(t_f)^2,  4*(t_f)^3,  5*(t_f)^4;
          0,    0,       2,   6*(t_f), 12*(t_f)^2, 20*(t_f)^3];
     
A = [a0; a1; a2; a3; a4; a5] 
 
%Matrix Output
MatrixOutput = (inv(QBaseM))* QInput
 
%Final Answer Output
AOutput = reshape(MatrixOutput,1,[])
 
end