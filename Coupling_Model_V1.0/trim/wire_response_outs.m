function wire_response_outs
%WIRE_RESPONSE_OUTS  Root outs for response plots: Vb,Wb, att, eta_e, etadot_e.
%
%  Adds/updates RootBusSel signals and Outports:
%    Q, elastic_displacement, elastic_m_velocity,
%    Velocity_b, Omega_b, rigid_m_displacement, Alpha

    mdl = 'aerodynamic_model';
    load_system(mdl);

    sigs = [ ...
        'Q,elastic_displacement,elastic_m_velocity,' ...
        'Velocity_b,Omega_b,rigid_m_displacement,Alpha'];
    bs = [mdl '/RootBusSel'];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'RootBusSel'))
        error('wire_response_outs:NoBusSel', 'RootBusSel missing');
    end
    set_param(bs, 'OutputSignals', sigs);

    % Desired outports (name -> bus sel port index 1-based)
    want = { ...
        'Out_Q', 1, 'Q'; ...
        'Out_eta_e', 2, 'elastic_displacement'; ...
        'Out_etad_e', 3, 'elastic_m_velocity'; ...
        'Out_Vb', 4, 'Velocity_b'; ...
        'Out_Wb', 5, 'Omega_b'; ...
        'Out_eta_r', 6, 'rigid_m_displacement'; ...
        'Out_Alpha', 7, 'Alpha'};

    ph_bs = get_param(bs, 'PortHandles');
    n_bs = numel(ph_bs.Outport);
    if n_bs < 7
        error('wire_response_outs:Ports', 'RootBusSel has %d outs, need 7', n_bs);
    end

    y0 = 40;
    for i = 1:size(want, 1)
        oname = want{i, 1};
        oport = want{i, 2};
        path = [mdl '/' oname];
        if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', oname))
            add_block('simulink/Sinks/Out1', path, ...
                'Position', [1200, y0 + (i-1)*40, 1230, y0 + (i-1)*40 + 14]);
        end
        % reconnect
        ph = get_param(path, 'PortHandles');
        ln = get_param(ph.Inport, 'Line');
        if ln > 0
            try, delete_line(ln); catch, end
        end
        add_line(mdl, ph_bs.Outport(oport), ph.Inport, 'autorouting', 'on');
    end

    wire_t_engine_out_(mdl);

    try
        set_param(mdl, 'SimulationCommand', 'update');
    catch
    end
    try
        save_system(mdl);
    catch
    end
    fprintf('wire_response_outs: RootBusSel -> %s  + Out_T_engine\n', sigs);
end

function wire_t_engine_out_(mdl)
% T_engine lives on Propulsion's output bus; branch it to a root outport.
    p = [mdl '/Propulsion Dynamics'];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Propulsion Dynamics'))
        error('wire_response_outs:NoProp', 'Propulsion Dynamics missing');
    end
    sel = [mdl '/PropTSel'];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'PropTSel'))
        add_block('simulink/Signal Routing/Bus Selector', sel, ...
            'Position', [980 520 1100 560]);
    end
    set_param(sel, 'OutputSignals', 'T_engine');

    ph_p = get_param(p, 'PortHandles');
    ph_s = get_param(sel, 'PortHandles');
    ln_in = get_param(ph_s.Inport, 'Line');
    if ln_in <= 0
        add_line(mdl, ph_p.Outport(1), ph_s.Inport, 'autorouting', 'on');
    end

    oname = 'Out_T_engine';
    path = [mdl '/' oname];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', oname))
        add_block('simulink/Sinks/Out1', path, ...
            'Position', [1200 530 1230 544]);
    end
    ph_o = get_param(path, 'PortHandles');
    ln_o = get_param(ph_o.Inport, 'Line');
    if ln_o > 0
        try, delete_line(ln_o); catch, end
    end
    add_line(mdl, ph_s.Outport(1), ph_o.Inport, 'autorouting', 'on');
end
