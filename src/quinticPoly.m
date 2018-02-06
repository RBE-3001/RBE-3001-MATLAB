function qT = quinticPoly(p, n, time, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initial trajectory
p = [ 250, 175,  50,  250;  % X-axis poistion values
     -200, -50, 250, -200;  % Y-axis poistion values
      300, -50, 250,  300];  % Z-axis poistion values

%number of points between two points
n = 10;

%time between points
time = 5;

d = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;


%Uses quintic function to build the quintic trajectory
qT = zeros(3, ((size(p,2)-1)*n)+1);

if DEBUG
    qT
end

for u = 1:size(p,1)
    
    timeOffset = 0;
    
    for y = 1:(size(p,2)-1)
        
        positions = [p(u,y); p(u,y+1)];
        
        if DEBUG
            positions
        end
        
        %start/stop positions (mm)
        q_0     = positions(1,1);
        q_f     = positions(2,1);
        %difference in position (mm)
        q_d     = q_f - q_0;
        %start/stop velocity (degrees/s)
        v_0     = 0;
        v_f     = 0;
        %start/stop accletation (degrees/s^2)
        alpha_0 = 0;
        alpha_f = 0;
        
        terms = [    q_0;
                     v_0;
                 alpha_0;
                     q_f;
                     v_f;
                 alpha_f];
        if q_d ~= 0  
        %This is the main base matrix
        QBaseM = [1,  q_0, (q_0)^2,   (q_0)^3,    (q_0)^4,    (q_0)^5;
                  0,    1, 2*(q_0), 3*(q_0)^2,  4*(q_0)^3,  5*(q_0)^4;
                  0,    0,       2,   6*(q_0), 12*(q_0)^2, 20*(q_0)^3;
                  1,  q_f, (q_f)^2,   (q_f)^3,    (q_f)^4,    (q_f)^5;
                  0,    1, 2*(q_f), 3*(q_f)^2,  4*(q_f)^3,  5*(q_f)^4;
                  0,    0,       2,   6*(q_f), 12*(q_f)^2, 20*(q_f)^3];
        
        %Quintic Polynomial Coeficents
        %A = [a0; a1; a2; a3; a4; a5]
        
        
        %Matrix Output
        A = (inv(QBaseM))*terms;
        
        else 
            A = zeros(6,1);
        end
        
        if DEBUG
            A
        end
        
        %creates a time matrix for plugging into the quintic funciton
        t = zeros(1,n);
        for j = 1:n
        
            if DEBUG
                t
                j
                n
                timeOffset
            end
            
            
            t(1,j) = time/n*j + timeOffset;
        
        end
        
        if DEBUG
            t
        end
         
        
        %builds the quintic trajectory with the quintic function
        qT(u,1) = A(1,1); 
        for k = 1:n
            if DEBUG
                qT
                disp(sprintf('u = %f, y = %f, k = %f, timeOffset = %f', u, y, k, timeOffset));
                
            end
             
            qT(u,((y-1)*n+k+1)) = A(1,1) + A(2,1)*(t(1,k))^1 + A(3,1)*(t(1,k))^2 + A(4,1)*(t(1,k))^3 + A(5,1)*(t(1,k))^4  +A(6,1)*(t(1,k))^5;

        end
        
        timeOffset = timeOffset + time;
        
    end
            
end

if DEBUG
    disp('final trajectory:');
    qT
end

end
