function [n_pass, n_fail, logs] = logic_check(cond, msg, n_pass, n_fail, logs)
%LOGIC_CHECK  测试用 PASS/FAIL 记录
    if cond
        n_pass = n_pass + 1;
        logs{end+1} = ['PASS  ' msg]; %#ok<AGROW>
    else
        n_fail = n_fail + 1;
        logs{end+1} = ['FAIL  ' msg]; %#ok<AGROW>
    end
end
