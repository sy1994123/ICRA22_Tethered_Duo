function [tot_cost] = total_cost_scalar(opt_traj,R)
[S,P] = total_cost_constraint(opt_traj); % Function defined by me  
tot_cost = sum(S);
N=size(opt_traj,1);
k1=5; k2=2;
    for i=1:2
        tot_cost=tot_cost+ k1*(opt_traj(1:N,i)'*R*opt_traj(1:N,i)); 
    end
    for i=4:5
        tot_cost=tot_cost+ k1*(opt_traj(1:N,i)'*R*opt_traj(1:N,i)); 
    end
    tot_cost=tot_cost+ k2*(opt_traj(1:N,3)'*R*opt_traj(1:N,3));
    tot_cost=tot_cost+ k2*(opt_traj(1:N,6)'*R*opt_traj(1:N,6));
end