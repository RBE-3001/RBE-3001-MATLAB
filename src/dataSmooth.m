function R = dataSmooth (m, n, l, p, d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   test data   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%imput matrix of data to be smoothed
%{
%sample dynamic data
m = [-8.1967; -3.2787; -3.2733; 8.1967; 8.1967; 72.250; 72.013; 95.082;
     95.082;  129.72;  75.286; 75.286; 77.049; 78.818; 78.818; 63.934;
      47.541;  60.755;  60.755]


%sample static data
m = [0.47741; 0.47424; 0.48327; 0.47985; 0.48132; 0.48303; 0.48523;
     0.47985; 0.48278; 0.48230; 0.48181; 0.48059; 0.48132; 0.48132;
     0.48400; 0.48034; 0.48376; 0.48474; 0.48181; 0.48474; 0.48230;
     0.47985; 0.49060; 0.48864]

%number of past data points to average with a data point
n = 3;
 
%lab number
l = 5;

%plot
p = true;

%debug
d = false;
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lab = l;
PLOT  = p;
DEBUG = d;



%saves the original input matrix for plotting
originalData = m;

%loops through all of the values in the input matrix in order to average
%them with n number of previous entries
for i = n:size(m,1)
    
    %initializes the sum of n previous entires and current entry
    sum = 0;
    
    %creates the sum by adding up the current entry and n previous entries
    for j = i-n+1:i
        sum = sum + m(j,1);
        if DEBUG
            disp(sprintf('sum = %f', sum));
        end
    end
    
    %calculates the average for a particular entry using previous entries
    %to smooth
    m(i,1) =  sum/n;
end

if DEBUG
    disp(sprintf('smooth m = %f', m));
end

if PLOT
    %plots original data vs smoothed data
    figure('Position', [864, 50, 864, 864]);
    hold on
    plot(originalData, 'r-*', 'LineWidth', 2)
    plot(m, 'b--x', 'LineWidth', 2)
    title(sprintf('RBE 3001 Lab %d: Data Smoothing Visualization with %d Points Running Average', lab, n));
    xlabel('Data Points');
    ylabel('Data Value');
    legend('Original Data', 'Smoothed Data');
    grid on;
end

%sets output to modified matrix
R = m;

end
