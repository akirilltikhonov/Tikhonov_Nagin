function r = q_prod(A, B) % quatertnion product r <- A*B
  r = zeros(4,1);
  r(1) = A(1)*B(1) - A(2)*B(2) - A(3)*B(3) - A(4)*B(4);
  r(2) = A(2)*B(1) + A(1)*B(2) - A(4)*B(3) + A(3)*B(4);
  r(3) = A(3)*B(1) + A(4)*B(2) + A(1)*B(3) - A(2)*B(4);
  r(4) = A(4)*B(1) - A(3)*B(2) + A(2)*B(3) + A(1)*B(4);
  
 