function cleanCSV ()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% deletes old data logging data

delete armDataValuesCopy.csv; %original unchanged data from all nine channels
delete armDataValues.csv;     %has smoothed load cell values
delete X-Y-Z-Position.csv;
delete X-Y-Z-Velocity.csv;
delete X-Y-Z-Force.csv;
delete TCP.csv;
delete JointAngles.csv;
delete JointVelocities.csv;
delete JointAcceletations.csv;
delete .Jp.csv;
delete JointTorque.csv;
delete averageLoadCell.csv;
delete avgZForce.csv;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%