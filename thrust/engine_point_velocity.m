function V_pt = engine_point_velocity(V_b, omega_b, r_eng_b, r_stub_b, e_hinge_b, tilt_rate)
%ENGINE_POINT_VELOCITY  发动机点速度（机体系）
%
% V_pt = V_b + ω_b × r_eng + (θ̇ e_hinge) × r_stub
% 举升台 e_hinge=0。r_eng 相对 CG，r_stub 相对折点，均为当前构型。

    n = size(r_eng_b, 2);
    V_b = V_b(:);
    omega_b = omega_b(:);
    tilt_rate = tilt_rate(:);
    V_pt = zeros(3, n);
    for i = 1:n
        w_tilt = tilt_rate(i) * e_hinge_b(:, i);
        V_pt(:, i) = V_b + cross(omega_b, r_eng_b(:, i)) + cross(w_tilt, r_stub_b(:, i));
    end
end
