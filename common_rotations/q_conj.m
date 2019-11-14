function r = q_conj(Q) % quatertnion conjugate r <- conj(Q)
  r = zeros(4,1);
  r(1) = Q(1);
  r(2:4) = -Q(2:4);
    