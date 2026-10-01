%% jonah_chase_cost_trade.m
% THW3: Ps requirement -> estimated steady-flight fuel cost per hour.
% Sizing method/baseline adapted from Adetomiwa Debo-Lawal's ps_trade.m.
% AI-assisted cost extension for Jonah Chase to review and reproduce.
% PRELIMINARY: baseline remains placeholder, not a frozen team design.
% No mission fuel-burn or maintenance model is available.
% This revision estimates fuel flow from steady-level-flight drag.
% Not total DOC or a sortie-average fuel prediction.
% See linked sources and limitations in jonah-chase.tex.
clear; clc; close all;
outDir = fileparts(mfilename('fullpath'));
if isempty(outDir), outDir = pwd; end

%% Inherited placeholder sizing assumptions
W0b=11000; WfFrac=2800/11000; WS0=60; CD0=0.022;
AR=5.0; e=0.80; Wcrew=2*(205+35); TWeng=5.5; kinst=1.3; C=-0.10;
PsThr=90; PsObj=125; PsRng=90:5:125;
h=15000; Mach=0.40:0.01:0.80;

tol=0.1; maxIter=500;

%% Constraint and threshold calibration (same as supplied ps_trade.m)
[~,d,rho,a]=isa_ft(h);
k=1/(pi*AR*e); beta=1-0.5*WfFrac; WS=beta*WS0;
TWb=min(arrayfun(@(M) tw_sls(PsThr,M,a,rho,d,beta,WS,CD0,k),Mach));
Wengb=kinst*TWb*W0b/TWeng;
A=(W0b-Wcrew-Wengb-WfFrac*W0b)/W0b/W0b^C;
n=numel(PsRng); W0=zeros(1,n); TW=W0; T=W0; Mbest=W0; iterations=W0;
for i=1:n
    [TW(i),j]=min(arrayfun(@(M) tw_sls(PsRng(i),M,a,rho,d,beta,WS,CD0,k),Mach));
    Mbest(i)=Mach(j); W=W0b; converged=false;
    for it=1:maxIter
        den=1-WfFrac-A*W^C;
        assert(isfinite(den) && den>0,'Nonphysical sizing denominator.');
        Wn=(Wcrew+kinst*TW(i)*W/TWeng)/den;
        assert(isfinite(Wn) && Wn>0,'Invalid sized gross weight.');
        if abs(Wn-W)<tol, converged=true; break; end
        W=0.5*W+0.5*Wn;
    end
    assert(converged,'Sizing failed to converge for Ps = %g.',PsRng(i));
    W0(i)=Wn; T(i)=TW(i)*Wn; iterations(i)=it;
end


%% Common-condition fuel expense, not maximum installed thrust fuel flow
% Analyst-selected comparison condition: clean, 1g, M0.60, 15000ft,
% weight MTOW minus half loaded fuel. No claim of mission-average cost.
% Honeywell F124 brochure p4: SLS max-power TSFC 0.78.
% Applying it at altitude/part power is an approximation; vary +/-20%.
% DLA FY2026 Oct1 2025 standard price JP5 $3.70/USgal, historical basis.
% Fuel density 6.8 lb/USgal is an explicit analyst assumption.
M_eval=0.60; tsfc=0.78; price_gal=3.70; density_lb_gal=6.8;
q_eval=0.5*rho*(M_eval*a)^2;
S=W0/WS0; W_eval=beta*W0;
CL_eval=W_eval./(q_eval*S);
CD_eval=CD0+k*CL_eval.^2;
Drag=q_eval*S.*CD_eval; % total airplane thrust required = drag
alpha_eval=0.6*d*(1+0.2*M_eval^2)^3.5;
thrust_fraction=Drag./(alpha_eval*T);
assert(all(thrust_fraction>0 & thrust_fraction<=1),'Insufficient available thrust.');
fuel_lb_hr=tsfc*Drag;
cost_hr=fuel_lb_hr/density_lb_gal*price_gal;
cost_low=0.8*cost_hr; cost_high=1.2*cost_hr;
penalty=100*(cost_hr(end)/cost_hr(1)-1);
sensitivity=(cost_hr(end)/cost_hr(1)-1)/((PsObj-PsThr)/PsThr);
slope=(cost_hr(end)-cost_hr(1))/(PsObj-PsThr);
assert(abs(cost_hr(end)/cost_hr(1)-W0(end)/W0(1))<1e-9);
results=table(PsRng',W0',T',S',Drag',thrust_fraction',fuel_lb_hr',cost_hr',cost_low',cost_high', ...
 'VariableNames',{'Ps_fps','MTOW_lb','InstalledThrust_lbf','WingArea_ft2','RequiredThrust_lbf', ...
 'AvailableThrustFraction','Fuel_lb_hr','FuelCost_USD_hr','TSFCminus20_USD_hr','TSFCplus20_USD_hr'});
disp(results);
writetable(results,fullfile(outDir,'jonah-chase-cost-results.csv'));
summary=table(cost_hr(1),cost_hr(end),cost_hr(end)-cost_hr(1),penalty,sensitivity,slope, ...
 'VariableNames',{'Baseline_USD_hr','Objective_USD_hr','Increase_USD_hr','Increase_pct','Sensitivity','USD_hr_per_fps'});
disp(summary); writetable(summary,fullfile(outDir,'jonah-chase-cost-summary.csv'));
fig=figure('Color','w','Units','inches','Position',[1 1 7.4 4.1]);
ax=axes(fig); hold(ax,'on');
hb=fill([PsRng fliplr(PsRng)],[cost_low fliplr(cost_high)],[0.9 0.8 0.83], ...
 'EdgeColor','none','FaceAlpha',0.6);
hc=plot(PsRng,cost_hr,'-o','Color',[0.53 0.12 0.25],'LineWidth',2,'MarkerFaceColor','w');
xline(PsThr,':k','Threshold 90','LabelVerticalAlignment','bottom','HandleVisibility','off');
xline(PsObj,':k','Objective 125','LabelVerticalAlignment','bottom','HandleVisibility','off');
xlabel('Required specific excess power, P_s (ft/s)');
ylabel('Estimated fuel cost (USD/flight hr)');
title({'Fuel expense of increased performance','Steady level flight: M0.60, 15,000 ft; historical FY2026 fuel price'});
legend([hc hb],{'Constant TSFC = 0.78 lb/(lbf hr)','Assumed TSFC +/-20% (not confidence bounds)'},'Location','northwest');
xlim([88 127]); ylim([0 max(cost_high)*1.35]); grid on; box on;
set(ax,'FontSize',10);
exportgraphics(fig,fullfile(outDir,'jonah-chase-trade.png'),'Resolution',300);

function tw=tw_sls(Ps,M,a,rho,d,beta,WS,CD0,k)
V=M*a; q=0.5*rho*V^2; alpha=0.6*d*(1+0.2*M^2)^3.5;
tw=beta/alpha*(Ps/V+q*CD0/WS+k*WS/q);
end
function [th,d,rho,a]=isa_ft(h)
T=518.67-0.00356616*h; th=T/518.67; d=th^5.2559;
rho=0.0023769*d/th; a=sqrt(1.4*1716.49*T);
end

