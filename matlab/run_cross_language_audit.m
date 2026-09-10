function run_cross_language_audit
% Execute UNCHANGED MATLAB tests/solvers and archive failures as well as passes.
% Fang (2024), predictor (3), original plant (35)-(36), feedback (37)-(40).
    root=fileparts(mfilename('fullpath'));
    repo=fileparts(root); out=fullfile(repo,'results','matlab-audit');
    if ~exist(out,'dir'), mkdir(out); end
    addpath(genpath(fullfile(root,'src')));
    diary(fullfile(out,'matlab.log'));
    cleanup=onCleanup(@() diary('off'));
    fprintf('MATLAB %s\n',version);
    testsOK=false; testError=''; testRows=table();
    try
        suite=testsuite(fullfile(root,'tests'),'IncludeSubfolders',true);
        if isempty(suite), error('Audit:EmptySuite','No tests were discovered.'); end
        runner=matlab.unittest.TestRunner.withTextOutput;
        runner.addPlugin(matlab.unittest.plugins.XMLPlugin.producingJUnitFormat(fullfile(out,'tests.xml')));
        res=runner.run(suite);
        testRows=table({res.Name}',[res.Passed]',[res.Failed]',[res.Incomplete]',[res.Duration]',...
            'VariableNames',{'Name','Passed','Failed','Incomplete','Duration'});
        writetable(testRows,fullfile(out,'tests.csv')); disp(testRows);
        testsOK=all([res.Passed]) && ~any([res.Incomplete]);
    catch err
        testError=getReport(err,'extended','hyperlinks','off'); fprintf('%s\n',testError);
    end
    kinds={'predictor','uncompensated','delay-free','predictor','predictor'};
    names={'predictor-default','uncompensated-default','delay-free-default','horizon-05','half-grid'};
    pars={struct(),struct(),struct(),struct('D0',.5,'dt',.001,'t_end',10),...
          struct('D0',.45,'dt',.02,'t_end',.1)};
    refs={'(3),(35)-(36),(39)-(40)','(35)-(38)','(35)-(38), zero delays',...
          '(3),(35)-(36),(39)-(40); audit-only D0','(3),(35)-(36),(39)-(40); audit-only dt'};
    rows=cell(numel(names),1); casesOK=true;
    for k=1:numel(names)
        row=struct('name',names{k},'kind',kinds{k},'parameters',pars{k},'source_equations',refs{k},...
                   'status','failed','error','');
        try
            switch kinds{k}
                case 'predictor', r=run_inexact_predictor(pars{k});
                case 'uncompensated', r=run_uncompensated(pars{k});
                case 'delay-free', r=run_delay_free(pars{k});
            end
            t=r.t; x=r.x_hist; u=[r.u1_hist,r.u2_hist]; extra=[];
            if isfield(r,'P_hist'), extra=r.P_hist; end
            data=[t,x,u,extra];
            if ~all(isfinite(data(:))), error('Audit:Nonfinite','Nonfinite trajectory.'); end
            row.status='completed'; row.final_time=t(end); row.samples=numel(t);
            row.final_state=x(end,:); row.final_norm=norm(x(end,:)); row.initial_control=u(1,:);
            row.max_abs_control=max(abs(u),[],1); row.csv=[names{k},'.csv'];
            row.columns=struct('time',1,'physical_state',2:3,'control',4:5);
            writematrix(data,fullfile(out,row.csv));
            fprintf('CASE %s norm=%.17g U0=%s\n',names{k},row.final_norm,mat2str(row.initial_control,17));
        catch err
            casesOK=false; row.error=getReport(err,'extended','hyperlinks','off'); fprintf('%s\n',row.error);
        end
        rows{k}=row;
    end
    record=struct('project','fang','matlab_version',version,'release',version('-release'),...
        'tested_checkout',getenv('GITHUB_SHA'),'run_id',getenv('GITHUB_RUN_ID'),...
        'tests_passed',testsOK,'test_error',testError,'test_count',height(testRows),...
        'all_cases_finite',casesOK,'round_half',round([-2.5,-.5,.5,2.5,22.5]),'cases',{rows},...
        'limits','Original solvers and assertions unmodified; finite-run and cross-language checks are not theorem proofs.');
    fid=fopen(fullfile(out,'audit.json'),'w');
    if fid<0, error('Audit:Write','Cannot write audit record.'); end
    fprintf(fid,'%s\n',jsonencode(record)); fclose(fid);
    if ~testsOK || ~casesOK
        error('Audit:Failure','Failures retained in tests.csv/audit.json; assertions were not relaxed.');
    end
end
