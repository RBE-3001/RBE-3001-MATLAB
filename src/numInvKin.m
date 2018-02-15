
%function inputs:
    % q0 -> initial angles
    % pF -> final position
    % debug -> isDebug on
    
function qD = numInvKin(q0, pF, DEBUG, PLOT)
    
    p0 = fwkin3001(q0, true, DEBUG); 
    
    jacob = jacob0(q0, DEBUG);
    try
        invJacob = pinv(jacob);
        try
            qD = (invJacob * (pF - p0)) - q0;
            
            if PLOT
                
            end
            
        catch
            error('Input point is outside taskspace: %f', p0);
        end
    catch 
        error('Jacobian is not inversible: %f', jacob);
    end

end