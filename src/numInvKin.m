
%function inputs:
    % q0 -> initial angles
    % pF -> final position
    % debug -> isDebug on
    
function qF = numInvKin(q0, pF, DEBUG)
    
    p0 = fwkin3001(q0, true, DEBUG); 
    
    jacob = jacob0(q0, DEBUG);
    try
        invJacob = pinv(jacob);
        try
            qF = (invJacob * (pF - p0)) - q0;            
        catch
            error('Input point is outside taskspace: %f', p0);
        end
    catch 
        error('Jacobian is not inversible: %f', jacob);
    end

end