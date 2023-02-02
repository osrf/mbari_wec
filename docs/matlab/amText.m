function str = amText(MA)

if(size(MA) ~= [6 6])
  fprintf(' amText:Error - The added mass matrix must be 6x6.\n')
  str = "Error";
  return 
end

strMat(1,1) = ...
    "| \\(X_{\\dot{u}}\\)   | Surge Added Mass (\\(\\mu_{xx}\\))            | %12.2f  | kg        |";
strMat(1,2) = ...
    "| \\(X_{\\dot{v}}\\)   | Surge-Sway Added Mass (\\(\\mu_{xy}\\))       | %12.2f  | kg        |";
strMat(1,3) = ...
    "| \\(X_{\\dot{w}}\\)   | Surge-Heave Added Mass (\\(\\mu_{xz}\\))      | %12.2f  | kg        |";
strMat(1,4) = ...
    "| \\(X_{\\dot{p}}\\)   | Surge-Roll Added Mass MOI (\\(\\mu_{xp}\\))   | %12.2f  | kg m      |";
strMat(1,5) = ...
    "| \\(X_{\\dot{q}}\\)   | Surge-Pitch Added Mass MOI(\\(\\mu_{xq}\\))   | %12.2f  | kg m      |";
strMat(1,6) = ...
    "| \\(X_{\\dot{r}}\\)   | Surge-Yaw Added Mass MOI (\\(\\mu_{xr}\\))    | %12.2f  | kg m      |";

strMat(2,2) = ...
    "| \\(Y_{\\dot{v}}\\)   | Sway Added Mass (\\(\\mu_{yy}\\))             | %12.2f  | kg        |";
strMat(2,3) = ...
    "| \\(Y_{\\dot{w}}\\)   | Sway-Heave Added Mass (\\(\\mu_{yz}\\))       | %12.2f  | kg        |";
strMat(2,4) = ...
    "| \\(Y_{\\dot{p}}\\)   | Sway-Roll Added Mass MOI (\\(\\mu_{yp}\\))    | %12.2f  | kg m      |";
strMat(2,5) = ...
    "| \\(Y_{\\dot{q}}\\)   | Sway-Pitch Added Mass MOI(\\(\\mu_{yq}\\))    | %12.2f  | kg m      |";
strMat(2,6) = ...
    "| \\(Y_{\\dot{r}}\\)   | Sway-Yaw Added Mass MOI (\\(\\mu_{yr}\\))     | %12.2f  | kg m      |";


strMat(3,3) = ...
    "| \\(Z_{\\dot{w}}\\)   | Heave Added Mass (\\(\\mu_{yy}\\))            | %12.2f  | kg        |";
strMat(3,4) = ...
    "| \\(Z_{\\dot{p}}\\)   | Heave-Roll Added Mass MOI (\\(\\mu_{yp}\\))   | %12.2f  | kg m      |";
strMat(3,5) = ...
    "| \\(Z_{\\dot{q}}\\)   | Heave-Pitch Added Mass MOI(\\(\\mu_{yq}\\))   | %12.2f  | kg m      |";
strMat(3,6) = ...
    "| \\(Z_{\\dot{r}}\\)   | Heave-Yaw Added Mass MOI (\\(\\mu_{yr}\\))    | %12.2f  | kg m      |";


strMat(4,4) = ...
    "| \\(K_{\\dot{p}}\\)   | Roll Added Mass MOI (\\(\\mu_{pp}\\))         | %12.2f  | kg m\\(^2\\)|";
strMat(4,5) = ...
    "| \\(K_{\\dot{q}}\\)   | Roll-Pitch Added Mass MOI(\\(\\mu_{pq}\\))    | %12.2f  | kg m\\(^2\\)|";
strMat(4,6) = ...
    "| \\(K_{\\dot{r}}\\)   | Roll-Yaw Added Mass MOI (\\(\\mu_{pr}\\))     | %12.2f  | kg m\\(^2\\)|";


strMat(5,5) = ...
    "| \\(M_{\\dot{q}}\\)   | Pitch Added Mass MOI(\\(\\mu_{qq}\\))         | %12.2f  | kg m\\(^2\\)|";
strMat(5,6) = ...
    "| \\(M_{\\dot{r}}\\)   | Pitch-Yaw Added Mass MOI (\\(\\mu_{qr}\\))    | %12.2f  | kg m\\(^2\\)|";

strMat(6,6) = ...
    "| \\(N_{\\dot{r}}\\)   | Yaw Added Mass MOI (\\(\\mu_{rr}\\))          | %12.2f  | kg m\\(^2\\)|";

k=0;
for( i=1:6 )
  for( j=i:6 )
    if( MA(i,j) ~= 0 )
    k=k+1;
    str(k,1) = sprintf(strMat(i,j),10*round(MA(i,j)/10));
    end
  end
end


end
