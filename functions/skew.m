function S = skew(x)
% INPUT: x: 3x1 vector
% OUTPUT: S: skew-symmetric matrix of x

S = ...
[
    0   ,  x(3), -x(2),  x(1);
   -x(3),  0   ,  x(1),  x(2);
    x(2), -x(1),  0   ,  x(3);
   -x(1), -x(2), -x(3),  0   ;
];

end

