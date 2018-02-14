
%function inputs:
    % q0 -> initial angles
    % pF -> final position
    % debug -> isDebug on
    
function qD = numInvKin(q0, pF, debug)
    
    p0 = fwkin3001(q0, true, debug); 
    
    jacob = jacob0(q0, debug);
    try
        invJacob = pinv(jacob);
        try
            qD = (invJacob * (pF - p0)) - q0;
        catch
            % not in task space
        end
    catch 
        % throw error
    end

end