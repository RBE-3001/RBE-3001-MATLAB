
%function inputs:
    % q0 -> initial angles
    % pF -> final position
    % debug -> isDebug on
    
function qFinal = numInvKin(qInitial, pFinal, DEBUG)
    
    p0 = fwkin3001(qInitial, true, false);
    
    q0 = qInitial;
    qF = q0;
  
    try
        jacobFull = jacob0(qInitial, false);
        jacob = jacobFull(1:3, :);
        invJacob = inv(jacob);
    catch
        error('Jacobian is not inversible');
    end
    
    try
        qOut = ikin3001(pFinal, DEBUG);
    catch
        error('Input point is outside taskspace');
    end
       
    diff = qOut - qF;
    
    while abs(diff(3,1)) > 1 && abs(diff(2,1)) > 1 && abs(diff(1,1)) > 1
        
       p0 = fwkin3001(q0, true, false);
        
    %division in place for performance
        qF = (invJacob * (pFinal - p0)) - q0; 
        q0 = qF;
        diff = qOut- qF;
        
        if DEBUG
            diff(3,1)
        end
    end

    qFinal = qOut;
    
end