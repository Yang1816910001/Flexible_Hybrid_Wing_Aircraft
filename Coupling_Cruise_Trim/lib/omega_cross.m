function [wxr, What] = omega_cross(omega, r) %#codegen
%OMEGA_CROSS  ω × r，r 可以是 3×n
%
%  wxr = omega_cross(omega, r)
%  [wxr, What] = omega_cross(omega, r)
%
%  omega  3×1（或 1×3）
%  r      3×n  每列一个位置矢量，与 omega 同一坐标系
%  wxr    3×n  第 j 列 = omega × r(:,j)
%  What   3×3  [ω]×，wxr = What * r
%
%  MATLAB cross 只处理两个向量；这里用反对称阵一次乘完。
%  片条当地速度：V_i = V + omega_cross(omega, r_cg2ac_*)

    w1 = omega(1);
    w2 = omega(2);
    w3 = omega(3);
    What = [  0, -w3,  w2
             w3,   0, -w1
            -w2,  w1,   0 ];
    wxr = What * r;
end
