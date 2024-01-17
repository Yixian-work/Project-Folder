function S = getReading(y, z, y_ans, z_ans, k)
% Obtain the radiation detector reading S from reading location (y, z),
% source location (y_ans, z_ans), and source strength k
    S = k./((y - y_ans).^2 + (z - z_ans).^2);
end