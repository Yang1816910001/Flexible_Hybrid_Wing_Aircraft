function [Tmax, tnd] = turbofan_tmax(Fmax, Nt, mach)
%TURBOFAN_TMAX  MathWorks 涡扇满油门无量纲推力 × Fmax × Nt
%
%  表是块里 Non-dimensional Thrust(Throttle=1, Mach)，M=0:0.1:1。
%  图上 Thr_map=1，所以涡扇出口就是这个 Tmax，再乘油门得到 T_i。

    if nargin < 3 || isempty(mach)
        mach = 0;
    end
    Mk = 0:0.1:1;
    Trow = [1.00, 0.92, 0.84, 0.76, 0.68, 0.60, 0.58, 0.56, 0.54, 0.52, 0.50];
    tnd = interp1(Mk, Trow, min(max(double(mach), 0), 1), 'linear');
    Tmax = Fmax * Nt * tnd;
end
