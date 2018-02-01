function cubicPoly(Input) 
syms a0
syms a1
syms a2
syms a3
 
%In seconds
 t0 = 5 
 t_f = 10 
%In degrees/second
 v0 =  0
 v_f = 0
%In degrees
 p0 = 50
 p_f = 60
 
 %These are the Input Values
Input = [p0; v0; p_f; v_f]
 
%This is the main base matrix
BaseM = [1 (t0) (t0)^2 (t0)^3; 0 1 2*(t0) 3*(t0)^2; 1 t_f (t_f)^2 (t_f)^3; 0 1 2*(t_f) 3*(t_f)^2]
A = [a0; a1; a2; a3] 
 
%Matrix Output
MOutput = (inv(BaseM))* Input
 
%Final Answer Output
AOutput = reshape(MOutput,1,[])
 
end