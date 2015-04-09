function [ output_args ] = printMatlabVarAsEigen( file, typeEigen, nameEigen, varMatlab )
%PRINTMATLABVARASEIGEN Summary of this function goes here
%   Detailed explanation goes here

if ~isempty(typeEigen)
    fprintf( file, '%s %s(%d, %d);\n', typeEigen, nameEigen, size(varMatlab,1), size(varMatlab,2) );
end
fprintf( file, '%s <<\n', nameEigen );
fprintf( file, '%e', varMatlab(1) );
for i=2:length( varMatlab(:) )
    if mod(i-1,size(varMatlab,2)) ~= 0
        fprintf( file, ',\t');
    else
        fprintf( file, ',\n');
    end
    fprintf( file, '%e', varMatlab(i) );
end

fprintf( file, ';\n' );
