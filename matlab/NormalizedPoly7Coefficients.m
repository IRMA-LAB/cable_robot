function coef = NormalizedPoly7Coefficients(T,coef)

for i = 1:4
  for j = 1:4
    if i == 1
      m(i,j) = 1;
    else
      m(i,j) = m(i-1,j)*(j+5-i);
    end
    t(i,j) = T^(j-i+4);
  end
  md(i,:) = m(i,:).*t(i,:);
end

for i = 1:4
  
end

b = zeros(4,1);
b(1) = 1;
c = linsolve(md,b);

coef.c = m(1,:)'.*c;
coef.cDerivative = m(2,:)'.*c;
coef.cDerivative2 = m(3,:)'.*c;