function [T,info] = readVSPResultsCSV(filename)
% Read OpenVSP Results Manager row-oriented multi-block CSV.
% Extract VSPAERO_Polar only. Iterations/load stations are NOT flight cases.
% All numeric polar fields are retained with case-sensitive names.
fid=fopen(filename,'r');assert(fid>=0,'Cannot open input file.');
cleanup=onCleanup(@()fclose(fid));
blocks={}; current=struct('type','','names',{{}},'values',{{}});
while true
 line=fgetl(fid);if ~ischar(line),break;end
 tokens=strsplit(line,',','CollapseDelimiters',false);
 tokens=cellfun(@strtrim,tokens,'UniformOutput',false);
 if isempty(tokens),continue;end
 key=erase(tokens{1},char(65279));
 if strcmp(key,'Results_Name')
  if ~isempty(current.type),blocks{end+1}=current;end %#ok<AGROW>
  assert(numel(tokens)>=2,'Malformed Results_Name row.');
  current=struct('type',tokens{2},'names',{{}},'values',{{}});
 elseif ~isempty(current.type) && ~isempty(key)
  current.names{end+1}=key;current.values{end+1}=tokens(2:end);
 end
end
if ~isempty(current.type),blocks{end+1}=current;end
T=table;count=0;counts=zeros(numel(blocks),1);types=strings(numel(blocks),1);
for j=1:numel(blocks)
 b=blocks{j};types(j)=string(b.type);
 if ~strcmp(b.type,'VSPAERO_Polar'),continue;end
 ia=find(strcmp(b.names,'Alpha'),1);
 assert(~isempty(ia),'Polar missing Alpha.');
 alpha=str2double(b.values{ia});N=numel(alpha);
 assert(N>=1 && all(isfinite(alpha)),'Invalid polar Alpha.');
 P=table;
 for k=1:numel(b.names)
  name=b.names{k};
  if startsWith(name,'Results_'),continue;end
  v=str2double(b.values{k});
  if isempty(v) || all(isnan(v)),continue;end
  assert(numel(v)==1 || numel(v)==N,'Field %s length does not match polar cases.',name);
  if isscalar(v),v=repmat(v,N,1);end
  name=matlab.lang.makeValidName(name);
  assert(~ismember(name,P.Properties.VariableNames),'Duplicate normalized field: %s',name);
  P.(name)=v(:);
 end
 if ismember('Re_1e6',P.Properties.VariableNames)
  P.ReCref=P.Re_1e6*1e6;
 end
 count=count+1;P.PolarBlock=repmat(count,N,1);counts(j)=N;
 if isempty(T),T=P;
 else
  assert(isequal(T.Properties.VariableNames,P.Properties.VariableNames), ...
   'Polar blocks have different fields. Process each export separately.');
  T=[T;P]; %#ok<AGROW>
 end
end
assert(count>0,'No VSPAERO_Polar block found. Export polar results from OpenVSP.');
info=table(types,counts,'VariableNames',{'BlockType','ExtractedCaseCount'});
end
