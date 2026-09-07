function [res, Q, pack] = cruise_static_residual(x, ctx, mode)
%CRUISE_STATIC_RESIDUAL  Residuals for cruise static trim (tilt frozen at 0).
%
%  mode = 'tilt_only' (default):
%    x = [alpha_rad; T_tilt_N], lift T=0
%    res = [Q(1); Q(3)]
%
%  mode = 'shared':
%    x = [alpha_rad; T_all_N]
%    res = [Q(1); Q(3)]
%
%  mode = 'pitch3':
%    x = [alpha_rad; T_tilt_N; T_front_lift_N]
%    aft lift T=0; front lift engines share T_front_lift
%    res = [Q(1); Q(3); Q(5)]   % surge, heave, pitch

    if nargin < 3 || isempty(mode)
        mode = 'tilt_only';
    end

    alpha = x(1);
    T = zeros(8, 1);
    switch lower(mode)
        case 'tilt_only'
            T(ctx.is_tilt) = max(x(2), 0);
            res_idx = [1; 3];
        case 'shared'
            T(:) = max(x(2), 0);
            res_idx = [1; 3];
        case 'pitch3'
            T(ctx.is_tilt) = max(x(2), 0);
            T(ctx.idx_lift_front) = max(x(3), 0);
            % aft lift remain 0
            res_idx = [1; 3; 5];
        otherwise
            error('cruise_static_residual:Mode', 'unknown mode %s', mode);
    end

    [Q, pack] = cruise_static_Q(alpha, T, ctx);
    res = Q(res_idx);
end
