function K = testf(T)
K = [ T(3, 2) - T(2, 3);
      T(1, 3) - T(3, 1);
      T(2, 1) - T(1, 2) ];
theta = acos((trace(T)-1) / 2);
t = sin(theta);
K = theta * K / (2*t);
end