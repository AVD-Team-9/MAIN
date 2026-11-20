function results = processVSPAero(inputFile,cfg)
% Arbitrary-length rectangular VSPAERO CSV/text/table processor.
% One row = one solved case, not an implied Cartesian product.
% Required cfg: conceptName, configuration, dragMode ('total'/'induced_only').
% Missing Mach/Beta/Re may be supplied as cfg.mach/cfg.beta/cfg.reynolds:
% scalar broadcasts, N-vector matches rows. Unknown values remain NaN.
% Optional exact header overrides: cfg.columns.alpha/CL/drag/mach/beta/reynolds/Cm.
% Fits need >=3 distinct alpha and CL values; sparse cases still export/plot.
if nargin~=2, error('Supply inputFile and cfg.'); end
for key={'conceptName','configuration','dragMode'}
    assert(isfield(cfg,key{1}),'Missing cfg.%s',key{1});
end
d=struct('outputDir','aero_results','columns',struct(),'CD0',NaN, ...
 'Cfe',NaN,'Swet_m2',NaN,'Sref_m2',NaN,'alphaFit_deg',[-4 8], ...
 'CLmax',NaN,'CLmin',NaN,'betaTolerance_deg',0.01, ...
 'machDecimals',8,'betaDecimals',8,'reDecimals',0, ...
 'extraCD',0,'enableWave',false,'notes','No provenance supplied.');
f=fieldnames(d);
for j=1:numel(f), if ~isfield(cfg,f{j}),cfg.(f{j})=d.(f{j});end,end
if istable(inputFile), T=inputFile; source='MATLAB table';
else
 source=char(inputFile);
 opts=detectImportOptions(inputFile,'FileType','text'); T=readtable(inputFile,opts);
end
N=height(T); assert(N>=1,'Input has no data rows.');
a=column(T,cfg,'alpha',{'Alpha','AoA','alpha_deg'},true);
cl=column(T,cfg,'CL',{'CL','CLtot'},true);
m=column(T,cfg,'mach',{'Mach','M'},false);
beta=column(T,cfg,'beta',{'Beta','beta_deg'},false);
re=column(T,cfg,'reynolds',{'ReCref','Re','Reynolds'},false);
cm=column(T,cfg,'Cm',{'CMy','CMytot','Cm'},false);
assert(all(isfinite([a;cl])),'Alpha and CL must be finite.');
assert(all(m(isfinite(m))>=0),'Mach must be nonnegative.');
assert(all(re(isfinite(re))>0),'Reynolds must be positive.');
assert(numel(cfg.alphaFit_deg)==2 && diff(cfg.alphaFit_deg)>0,'Invalid alphaFit_deg.');
mode=lower(string(cfg.dragMode));
assert(isscalar(mode) && any(mode==["total","induced_only"]),'Invalid dragMode.');
if mode=="total"
 imported=column(T,cfg,'drag',{'CDtot','CDtotal','CD'},true);
 assert(~cfg.enableWave && cfg.extraCD==0,'Total mode forbids extra/wave drag additions.');
 cd0=nan(N,1); extra=zeros(N,1);
else
 imported=column(T,cfg,'drag',{'CDi','CDind','CDinduced'},true);
 if ~isfinite(cfg.CD0)
  z=[cfg.Cfe cfg.Swet_m2 cfg.Sref_m2];
  assert(all(isfinite(z)) && all(z>0),'Supply positive CD0 or Cfe, Swet_m2, Sref_m2.');
  cfg.CD0=cfg.Cfe*cfg.Swet_m2/cfg.Sref_m2;
 end
 assert(isscalar(cfg.CD0) && cfg.CD0>0,'CD0 must be positive scalar.');
 cd0=repmat(cfg.CD0,N,1); extra=expand(cfg.extraCD,N,'extraCD');
 assert(all(isfinite(extra) & extra>=0),'extraCD must be nonnegative finite.');
end
assert(all(isfinite(imported)),'Drag input must be finite.');
wave=zeros(N,1);
if cfg.enableWave
 assert(strcmpi(cfg.configuration,'clean'),'Wave model only for clean configuration.');
 for k={'sweepQC_deg','tc','kappa'}
  assert(isfield(cfg,k{1}) && isscalar(cfg.(k{1})) && isfinite(cfg.(k{1})), ...
   'Wave model needs scalar cfg.%s',k{1});
 end
 assert(all(isfinite(m)),'Wave model requires known Mach for every row.');
 assert(abs(cfg.sweepQC_deg)<80 && cfg.tc>0 && cfg.kappa>0,'Invalid wave model inputs.');
 c=cosd(cfg.sweepQC_deg);
 mdd=cfg.kappa/c-cfg.tc/c^2-cl/(10*c^3);
 wave=20*max(m-(mdd-(0.1/80)^(1/3)),0).^4;
end
if mode=="total",cd=imported;else,cd=imported+cd0+extra+wave;end
ld=cl./cd;ld(cd<=0)=NaN;
bounds=true(N,1);
if isfinite(cfg.CLmax),bounds=bounds & cl<=cfg.CLmax;end
if isfinite(cfg.CLmin),bounds=bounds & cl>=cfg.CLmin;end
% Unknown conditions form explicit unknown groups, never zero assumptions.
keys=[round(m,cfg.machDecimals) round(beta,cfg.betaDecimals) round(re,cfg.reDecimals)];
keys(isnan(keys))=-realmax;
[uniqueKeys,~,group]=unique(keys,'rows');
uniqueKeys(uniqueKeys==-realmax)=NaN;
ng=size(uniqueKeys,1);
Data=table((1:N)',group,m,beta,re,a,cl,cm,imported,cd0,extra,wave,cd,ld,bounds, ...
 'VariableNames',{'SourceRow','Group','Mach','Beta_deg','Reynolds','Alpha_deg','CL','Cm', ...
 'ImportedDrag','CD0_added','CDextra_added','CDwave_added','CDtotal','LiftToDrag','WithinCLBounds'});
rows=cell(ng,18);
name=regexprep(char(cfg.conceptName),'[^A-Za-z0-9_-]','_');
folder=tempname(cfg.outputDir);
if ~exist(cfg.outputDir,'dir'),mkdir(cfg.outputDir);end
mkdir(folder);
for g=1:ng
 ix=find(group==g); x=a(ix); y=cl(ix); z=cd(ix);
 mask=x>=cfg.alphaFit_deg(1) & x<=cfg.alphaFit_deg(2) & bounds(ix);
 % A group is fixed-beta: nonzero/unknown beta still exports but no symmetric fit.
 symmetric=isfinite(uniqueKeys(g,2)) && abs(uniqueKeys(g,2))<=cfg.betaTolerance_deg;
 slope=NaN;az=NaN;rLift=NaN;coef=nan(3,1);rDrag=NaN;
 liftStatus="Insufficient distinct alpha in fit window";
 dragStatus="Insufficient distinct CL in fit window";
 if ~symmetric
  liftStatus="Fit skipped: beta unknown or nonzero";dragStatus=liftStatus;
 else
  if sum(mask)>=3 && numel(unique(x(mask)))>=3
   X=[ones(sum(mask),1) x(mask)]; fit=X\y(mask);
   slope=fit(2);if abs(slope)>eps,az=-fit(1)/slope;end
   rLift=rSquared(y(mask),X*fit);liftStatus="Fit available; verify attached flow";
  end
  X=[ones(sum(mask),1) y(mask) y(mask).^2];
  if sum(mask)>=3 && rank(X)==3
   coef=X\z(mask);rDrag=rSquared(z(mask),X*coef);
   dragStatus="Fit available; verify validity";
  end
 end
 eligible=bounds(ix) & y>0 & z>0 & isfinite(ld(ix));
 best=NaN;bc=NaN;ba=NaN;
 if any(eligible)
  ii=find(eligible);[best,j]=max(ld(ix(ii)));j=ii(j);bc=y(j);ba=x(j);
 end
 rows(g,:)={g,uniqueKeys(g,1),uniqueKeys(g,2),uniqueKeys(g,3),numel(ix), ...
 numel(unique(x)),sum(mask),slope,slope*180/pi,az,rLift, ...
 coef(1),coef(2),coef(3),rDrag,best,bc,ba};
 if g==1,liftStatuses=strings(ng,1);dragStatuses=strings(ng,1);end
 liftStatuses(g)=liftStatus;dragStatuses(g)=dragStatus;
 fig=figure('Visible','off','Color','w','Position',[80 80 1100 760]);
 cleanup=onCleanup(@()close(fig));tiledlayout(2,2);
 [~,order]=sort(x);
 nexttile;plot(x(order),y(order),'o-');xlabel('Alpha (deg)');ylabel('C_L');grid on;title('Imported lift');
 nexttile;plot(z(order),y(order),'o-');xlabel('C_D');ylabel('C_L');grid on;title('Assembled polar');
 nexttile;plot(y(order),ld(ix(order)),'o-');xlabel('C_L');ylabel('L/D');grid on;title('Sampled efficiency');
 nexttile;
 if any(isfinite(cm(ix)))
  plot(x(order),cm(ix(order)),'o-');xlabel('Alpha (deg)');ylabel('C_m');grid on;
 else,axis off;text(0,0.5,'No pitching-moment data');end
 sgtitle(sprintf('%s | %s | M=%g, beta=%g deg, Re=%g | %d rows', ...
  char(cfg.conceptName),char(cfg.configuration),uniqueKeys(g,:),numel(ix)),'Interpreter','none');
 exportgraphics(fig,fullfile(folder,sprintf('%s_Group_%03d.png',name,g)),'Resolution',180);
 savefig(fig,fullfile(folder,sprintf('Group_%03d.fig',g)));clear cleanup
end
Summary=cell2table(rows,'VariableNames',{'Group','Mach','Beta_deg','Reynolds', ...
 'SampleCount','DistinctAlphaCount','FitWindowCount','CLalpha_perDeg','CLalpha_perRad', ...
 'AlphaZeroLift_deg','LiftFit_R2','Polar_b0','Polar_b1','Polar_b2','PolarFit_R2', ...
 'BestSampled_LD','CL_at_BestLD','Alpha_at_BestLD_deg'});
Summary.LiftFitStatus=liftStatuses;Summary.DragFitStatus=dragStatuses;
assumptions={ 'Concept',char(cfg.conceptName);'Configuration',char(cfg.configuration); ...
 'Source',source;'Drag mode',char(mode); ...
 'Sampling','One row is one actual solved case. No missing Cartesian combinations are invented.'; ...
 'Missing conditions','Unknown Mach, beta, Re remain NaN and group separately. Set cfg scalar/vector fallbacks if known.'; ...
 'Fits','At least 3 distinct alpha for lift, and full-rank 3-term drag fit; near-zero known beta only.'; ...
 'Fit window',mat2str(cfg.alphaFit_deg); ...
 'Grouping precision',sprintf('Mach %d decimals; beta %d; Re %d',cfg.machDecimals,cfg.betaDecimals,cfg.reDecimals); ...
 'Polar equation','CD = b0 + b1*CL + b2*CL^2. Coefficients are regression outputs, not guaranteed physical drag components.'; ...
 'Best L/D','Best positive-lift eligible sampled point within each condition group, including nonzero beta; not a global optimum.'; ...
 'Stall','CL limits are externally supplied; no automatic stall prediction.'; ...
 'References','Verify source reference area/chord/span and moment reference point.'; ...
 'Forces','This revision exports coefficients, not dimensional forces. Use consistent atmosphere and geometry downstream.'; ...
 'Provenance',char(cfg.notes)};
Assumptions=cell2table(assumptions,'VariableNames',{'Item','Description'});
book=fullfile(folder,'Aerodynamics_Report.xlsx');
% Excel row limit includes header. Split long data tables across sheets.
writeSheets(Summary,book,'Summary');writeSheets(Data,book,'Evaluated');
writeSheets(T,book,'Imported');writetable(Assumptions,book,'Sheet','Assumptions');
save(fullfile(folder,'Results.mat'),'Summary','Data','cfg','Assumptions');
results=struct('summary',Summary,'data',Data,'settings',cfg,'outputFolder',folder);
fprintf('Processed %d rows in %d groups. Output: %s\n',N,ng,folder);
end
function x=column(T,cfg,key,aliases,required)
names=T.Properties.VariableNames;idx=[];
if isfield(cfg.columns,key)
 idx=find(strcmp(names,char(cfg.columns.(key))));
 assert(numel(idx)==1,'Mapped %s column was not found.',key);
else
 for j=1:numel(aliases),idx=find(strcmp(names,aliases{j}));if ~isempty(idx),break;end,end
end
if isempty(idx)
 if isfield(cfg,key),x=expand(cfg.(key),height(T),key);
 else,x=nan(height(T),1);end
else
 x=T{:,idx};if ~isnumeric(x),x=str2double(string(x));end
 x=double(x(:));
end
assert(~any(isinf(x)),'Infinite %s values are not supported.',key);
if required,assert(all(isfinite(x)),'Missing/invalid %s. Set cfg.columns.%s.',key,key);end
end
function x=expand(x,n,key)
assert(isnumeric(x) && isvector(x),'cfg.%s must be numeric scalar/vector.',key);
if isscalar(x),x=repmat(double(x),n,1);
else,assert(numel(x)==n,'cfg.%s must have 1 or %d elements.',key,n);x=double(x(:));end
end
function r=rSquared(y,p)
d=sum((y-mean(y)).^2);r=NaN;if d>eps,r=1-sum((y-p).^2)/d;end
end
function writeSheets(T,book,prefix)
limit=1048575;
for k=1:ceil(height(T)/limit)
 ix=(k-1)*limit+1:min(k*limit,height(T));
 writetable(T(ix,:),book,'Sheet',sprintf('%s_%d',prefix,k));
end
end
