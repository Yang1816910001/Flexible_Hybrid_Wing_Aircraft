function fix_propulsion_interfaces()
%FIX_PROPULSION_INTERFACES  修 aerodynamic_model_V1 推进口维数（不重铺 LUT）

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'thrust'));
    addpath(fullfile(root, 'lib'));
    cd(root);

    mdl = 'aerodynamic_model_V1';
    slx = fullfile(root, [mdl '.slx']);
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    load_system(slx);
    set_param(mdl, 'Lock', 'off');
    p = [mdl '/Propulsion Dynamics'];
    pw = [p '/Pack_Wrench'];

    set_param([p '/Constant4'], 'Value', '(pi/2)*ones(8,1)');
    set_param([p '/RPM'], 'PortDimensions', '8');
    set_param([p '/Tilt_cmd'], 'PortDimensions', '8');
    set_param([p '/dot_Tilt_cmd'], 'PortDimensions', '8');

    add_pitch_goto_(p);
    retarget_tables_(p);
    add_right_flips_(p);
    fix_splitter_(p);
    wire_fm_and_pack_(p, pw);

    save_system(mdl);
    fprintf('saved %s\n', slx);
end

function add_pitch_goto_(p)
    if isempty(find_system(p, 'SearchDepth', 1, 'Name', 'BladePitch_deg'))
        add_block('simulink/Sources/Constant', [p '/BladePitch_deg'], ...
            'Value', '13', 'Position', [90 620 150 650]);
        add_block('simulink/Signal Routing/Goto', [p '/Goto_Pitch'], ...
            'GotoTag', 'BladePitchAngle_DEG', 'TagVisibility', 'local', ...
            'Position', [180 624 250 646]);
        add_line(p, 'BladePitch_deg/1', 'Goto_Pitch/1', 'autorouting', 'on');
    end
end

function retarget_tables_(p)
    rename_lut_prefix_([p '/Subsystem2'], 'cfd2d_P1_', 'cfd2d_P5_');
    rename_lut_prefix_([p '/Subsystem3'], 'cfd2d_P1_', 'cfd2d_P5_');
    rename_lut_prefix_([p '/Subsystem6'], 'cfd3d_P2_', 'cfd3d_P6_');
    rename_lut_prefix_([p '/Subsystem7'], 'cfd3d_P2_', 'cfd3d_P6_');
end

function rename_lut_prefix_(sys, oldp, newp)
    luts = find_system(sys, 'SearchDepth', 1, 'BlockType', 'Lookup_n-D');
    for i = 1:numel(luts)
        t = get_param(luts{i}, 'Table');
        if startsWith(t, oldp)
            set_param(luts{i}, 'Table', [newp t(numel(oldp)+1:end)]);
        end
    end
end

function add_right_flips_(p)
    hover = {'P_FY', 'P_MX', 'P_MZ'};
    tilt = {'P_FY', 'P_MX1', 'P_MZ'};
    insert_neg_( [p '/Subsystem1'], hover);
    insert_neg_( [p '/Subsystem3'], hover);
    insert_neg_( [p '/Subsystem5'], tilt);
    insert_neg_( [p '/Subsystem7'], tilt);
end

function insert_neg_(sys, names)
    for k = 1:numel(names)
        src = names{k};
        gname = [src '_R'];
        if ~isempty(find_system(sys, 'SearchDepth', 1, 'Name', gname))
            continue
        end
        lh = get_param([sys '/' src], 'LineHandles');
        ln = lh.Outport(1);
        if ln <= 0
            continue
        end
        dstH = get_param(ln, 'DstBlockHandle');
        dstP = get_param(ln, 'DstPortHandle');
        dstName = get_param(dstH(1), 'Name');
        portNum = get_param(dstP(1), 'PortNumber');
        pos = get_param([sys '/' src], 'Position');
        gpos = [pos(3)+20 pos(2) pos(3)+50 pos(4)];
        delete_line(ln);
        add_block('simulink/Math Operations/Gain', [sys '/' gname], ...
            'Gain', '-1', 'Position', gpos);
        add_line(sys, [src '/1'], [gname '/1'], 'autorouting', 'on');
        add_line(sys, [gname '/1'], sprintf('%s/%d', dstName, portNum), ...
            'autorouting', 'on');
    end
end

function fix_splitter_(p)
    blk = [p '/MATLAB Function'];
    script = sprintf([ ...
        'function [Hover1,Tilt2,Tilt3,Hover4,Hover5,Tilt6,Tilt7,Hover8] = ', ...
        'fcn(Lambda1, Lambda2, F_mach, M_mach, Fq, Mq)\n', ...
        'L1 = Lambda1(:); L2 = Lambda2(:); Fm = F_mach(:); Mm = M_mach(:);\n', ...
        'Fq = Fq(:); Mq = Mq(:);\n', ...
        'Hover1 = [L1(1); L2(1); Fm(1); Mm(1); Fq(1); Mq(1)];\n', ...
        'Tilt2  = [L1(2); L2(2); Fm(2); Mm(2); Fq(2); Mq(2)];\n', ...
        'Tilt3  = [L1(3); L2(3); Fm(3); Mm(3); Fq(3); Mq(3)];\n', ...
        'Hover4 = [L1(4); L2(4); Fm(4); Mm(4); Fq(4); Mq(4)];\n', ...
        'Hover5 = [L1(5); L2(5); Fm(5); Mm(5); Fq(5); Mq(5)];\n', ...
        'Tilt6  = [L1(6); L2(6); Fm(6); Mm(6); Fq(6); Mq(6)];\n', ...
        'Tilt7  = [L1(7); L2(7); Fm(7); Mm(7); Fq(7); Mq(7)];\n', ...
        'Hover8 = [L1(8); L2(8); Fm(8); Mm(8); Fq(8); Mq(8)];\n']);
    cfg = get_param(blk, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = script;
    try
        cfg.SimulateUsing = 'InterpretedExecution';
    catch
    end
end

function wire_fm_and_pack_(p, pw)
    safe_del_line_(p, 'Mux/1', 'Pack_Wrench/1');
    safe_del_line_(p, 'Mux/1', 'Out Bus Element/1');

    if isempty(find_system(p, 'SearchDepth', 1, 'Name', 'FM_6x8'))
        add_block('simulink/Math Operations/Reshape', [p '/FM_6x8'], ...
            'OutputDimensionality', 'Customize', ...
            'OutputDimensions', '[6 8]', ...
            'Position', [980 80 1060 120]);
    end
    if isempty(find_system(p, 'SearchDepth', 1, 'Name', 'T_from_Fx'))
        add_block('simulink/Signal Routing/Selector', [p '/T_from_Fx'], ...
            'Position', [1100 30 1180 70]);
        sel = [p '/T_from_Fx'];
        set_param(sel, 'NumberOfDimensions', '1');
        set_param(sel, 'InputPortWidth', '48');
        set_param(sel, 'IndexOptions', 'Index vector (dialog)');
        set_param(sel, 'Indices', '[1 7 13 19 25 31 37 43]');
    end

    try add_line(p, 'Mux/1', 'FM_6x8/1', 'autorouting', 'on'); catch, end
    try add_line(p, 'Mux/1', 'T_from_Fx/1', 'autorouting', 'on'); catch, end
    try add_line(p, 'T_from_Fx/1', 'Out Bus Element/1', 'autorouting', 'on'); catch, end

    set_param([pw '/T'], 'PortDimensions', '[6 8]');
    try set_param([pw '/T'], 'Name', 'FM'); catch, end
    fmName = 'FM';
    if isempty(find_system(pw, 'SearchDepth', 1, 'Name', 'FM'))
        fmName = 'T';
    end

    fcn = [pw '/engine_pack_wrenches'];
    script = sprintf([ ...
        'function W_prop = fcn(FM, e_s, p_eng, p_eng0, C9, R_sb)\n', ...
        'W_prop = engine_pack_wrenches(FM, e_s, p_eng, p_eng0, C9, R_sb);\n']);
    cfg = get_param(fcn, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = script;
    try
        cfg.SimulateUsing = 'InterpretedExecution';
    catch
    end
    drawnow;

    if isempty(find_system(pw, 'SearchDepth', 1, 'Name', 'C9'))
        add_block('simulink/Sources/In1', [pw '/C9'], ...
            'PortDimensions', '[9 8]', 'Position', [30 200 60 214]);
        add_block('simulink/Sources/In1', [pw '/R_sb'], ...
            'PortDimensions', '[3 3]', 'Position', [30 240 60 254]);
    end
    try add_line(pw, 'C9/1', 'engine_pack_wrenches/5', 'autorouting', 'on'); catch, end
    try add_line(pw, 'R_sb/1', 'engine_pack_wrenches/6', 'autorouting', 'on'); catch, end
    try add_line(p, 'FM_6x8/1', 'Pack_Wrench/1', 'autorouting', 'on'); catch, end
    try add_line(p, 'engine_dcm_body_to_eng/1', 'Pack_Wrench/5', 'autorouting', 'on'); catch, end
    try add_line(p, 'R_sb/1', 'Pack_Wrench/6', 'autorouting', 'on'); catch, end
end

function safe_del_line_(sys, src, dst)
    try
        delete_line(sys, src, dst);
    catch
    end
end
