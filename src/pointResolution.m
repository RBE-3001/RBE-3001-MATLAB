function Pres = pointResolution(p, r, DPT, d)
degreesPerTics = DPT;
DEBUG = d;

%set data resultion (number of data points per set-point)
holdSize = r;

%builds trajectory using inverse kinimatics
Pres = zeros(3,holdSize*size(p,2));
for k = 1:size(p,2)
    sPres = zeros(3,1);
    sPres = real(ikin3001(p(:,k), DEBUG));
    sPres = sPres/degreesPerTics;
    for j = 1:holdSize
        Pres(:,(j+(k-1)*holdSize)) = sPres(:,:);
    end
end

%displays the set-points matrix
if DEBUG
    disp('Point Resolution matrix:');
    Pres
end

end