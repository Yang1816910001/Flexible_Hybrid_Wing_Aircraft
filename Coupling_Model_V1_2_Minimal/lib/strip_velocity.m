function Vi = strip_velocity(V, omega, r) %#codegen
%STRIP_VELOCITY  片条当地速度 V_i = V + ω × r
%
%  V      3×1  质心平动速度
%  omega  3×1  角速度
%  r      3×n  质心 → 片条 AC（与 V、omega 同一坐标系）
%  Vi     3×n  第 j 列 = V + omega × r(:,j)
%
%  不用 ones/repmat/size 扩维，避免 MATLAB Function 把输出判成可变大小。

    wxr = omega_cross(omega, r);
    Vi = wxr;
    Vi(1, :) = V(1) + wxr(1, :);
    Vi(2, :) = V(2) + wxr(2, :);
    Vi(3, :) = V(3) + wxr(3, :);
end
