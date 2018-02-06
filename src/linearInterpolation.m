function Pos = linearInterpolation(p, r, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    p = [300,   0,   0, 300;
           0, 300,   0,   0;
           0,   0, 450,   0];
    r = 10;

    d = true;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DEBUG = d;
       
    %number of points to spread difference across
    holdSize = r;
    
    %makes a matrix to hold the new trajectory
    Pos = zeros(3, (size(p,2)-1)*r);
      
    %conter for incrementing
    counter = 0;
    
    %fills in the new trajectory
    for j = 1:(size(p,2)-1)
        
        for u = holdSize*(j-1)+1:holdSize*(j)
            Pos(1,u) = p(1,j)-(p(1,j)- p(1,j+1))/holdSize*counter;
            Pos(2,u) = p(2,j)-(p(2,j)- p(2,j+1))/holdSize*counter;
            Pos(3,u) = p(3,j)-(p(3,j)- p(3,j+1))/holdSize*counter;
            counter = counter + 1;
        end
        
        counter = 0;
    
    end
    
    if DEBUG
        disp('Linear Interpolation matrix:');
        Pos
    end
    
end