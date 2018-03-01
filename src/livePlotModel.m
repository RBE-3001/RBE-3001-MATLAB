function livePlotModel (m, k, dpt, l, a, fS, vS, f, p, d)

degreesPerTics = dpt;
lab = l;
axe = a;
forceScale = fS;
velocityScale = vS;
FORCE = f;
PLOT = p;
DEBUG = d;


    %plots a stick model with green spheres for joints, thick blue lines for links,
    %and a thin red line for path
    if PLOT
        %plots links and joints
        f1 = stickModel([m(k,1), m(k,4), m(k,7)]*degreesPerTics, degreesPerTics, lab, axe);
        %plots path
        if k > 1
            traceModel([m(k-1,1), m(k-1,4), m(k-1,7),m(k,1), m(k,4), m(k,7)]*degreesPerTics, lab, axe);
        end
        
        %instantaneous joint angles
        instJointAngles = [m(k,1); m(k,4); m(k,7)]*degreesPerTics;
        
        %instantaneous torques
        instJointTorque(1,1) = ADCToTorque(m(k,3), 1, DEBUG).';
        instJointTorque(2,1) = ADCToTorque(m(k,6), 2, DEBUG).';
        instJointTorque(3,1) = ADCToTorque(m(k,9), 3, DEBUG).';
        instJointTorques = statics3001(instJointAngles, instJointTorque, DEBUG);
        
        if FORCE
            %draws vector of force on end effector
            quiverModel(instJointAngles, instJointTorques, norm(instJointTorques)*forceScale, axe, FORCE, DEBUG);
        
        else
            %draws vector of velocity of end effector
            quiverModel([m(k,1); m(k,4); m(k,7)]*degreesPerTics, [m(k,2); m(k,5); m(k,8)]*degreesPerTics, velocityScale, FORCE, DEBUG);
        end
        
    end
    
end