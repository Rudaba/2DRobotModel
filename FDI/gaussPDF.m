function lamda = gaussPDF(measurement, measurementPred, S)

z = measurement-measurementPred;

E = 1/sqrt(det(2*pi*S));

lamda = E * exp(-1/2 * z' * inv(S) * z);

end