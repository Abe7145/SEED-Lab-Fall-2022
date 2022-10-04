

A = readtable("motordata.csv");
A = table2array(A);

plot(A(:,1),A(:,2));
%fprintf(fid,'%d',1)
