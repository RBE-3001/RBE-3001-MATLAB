function quinticPoly(QInput) 
syms a0
syms a1
syms a2
syms a3
syms a4
syms a5
 
%In seconds
 t0 = 5 
 t_f = 10 
%Velocity in degrees/second
 v0 =  0
 v_f = 0
%In degrees
 p0 = 50
 p_f = 60
 
 %Initial and final acceleration
 alpha_0 = 10
 alpha_f = 10
 
 %These are the Input Values
QInput = [p0; v0; alpha_0; p_f; v_f; alpha_f;]
 
%This is the main base matrix
QBaseM = [1 (t0) (t0)^2 (t0)^3 (t0)^4 (t0)^5; 
          0 1 2*(t0) 3*(t0)^2 4*(t0)^3 5*(t0)^4; 
          0 0 2 6*(t0) 12*(t0)^2 20*(t0)^3; 
          1 t_f (t_f)^2 (t_f)^3 (t_f)^4 (t_f)^5; 
          0 1 2*(t_f) 3*(t_f)^2 4*(t_f)^3 5*(t_f)^4
          0 0 2 6*(t_f) 12*(t_f)^2 20*(t_f)^3 ]
     
A = [a0; a1; a2; a3; a4; a5] 
 
%Matrix Output
MatrixOutput = (inv(QBaseM))* QInput
 
%Final Answer Output
AOutput = reshape(MatrixOutput,1,[])
 
end