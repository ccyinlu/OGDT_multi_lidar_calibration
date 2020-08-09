function B = removeInf(A)
  B = A(all(~isinf(A),2), :);
end