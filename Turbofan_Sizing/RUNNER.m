clear; clc; close all;

% PRM2 trainer, threshold requirements
%R = run_tf_constraints('json', 'prm2_requirements.json');

% PRM2 trainer, objective requirements
%R = run_tf_constraints('json', 'prm2_objective_requirements.json');

% T-45C validation case
%R = run_tf_constraints('json', 't45c_requirements.json');

% original business-jet example
%R = run_tf_constraints();
R = run_tf_constraints('json', 'prm2_requirements.json')

compare_concepts({'prm2_concept2_rfp_requirements.json', 'prm2_concept3_rfp_requirements.json', ...
                  'prm2_concept5_rfp_requirements.json', 'prm2_concept6_rfp_requirements.json'});
compare_concepts({'prm2_concept2_requirements.json', 'prm2_concept3_requirements.json', ...
                  'prm2_concept5_requirements.json', 'prm2_concept6_requirements.json'});