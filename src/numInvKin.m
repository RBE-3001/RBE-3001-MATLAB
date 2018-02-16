
%function inputs:
    % q0 -> initial angles
    % pF -> final position
    % debug -> isDebug on
    
function qF = numInvKin(q0, pF, DEBUG)
    
    p0 = fwkin3001(q0, true, false);
    
    jacobFull = jacob0(q0, false);
    
    jacob = jacobFull(1:3, :);
    
    if DEBUG
        jacob
        p0
    end
    
    try
        invJacob = inv(jacob);
        
        if DEBUG
            invJacob
        end
        
        try
            %qF = (invJacob * (pF - p0)) - q0;
            qF=ikin3001(pF, DEBUG);
        catch
            error('Input point is outside taskspace: %f', p0);
        end
    catch 
        error('Jacobian is not inversible: %f', jacob);
    end

end