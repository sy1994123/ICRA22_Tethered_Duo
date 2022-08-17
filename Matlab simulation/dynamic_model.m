 clc;clear;  
syms t l mc vd wd v w t_dot J mw
  m=mc+2*mw;
  S=[cos(t) -l*sin(t); sin(t) l*cos(t); 0 1];
  pinv(S)
  S_dot=[-sin(t)*t_dot -l*cos(t)*t_dot; cos(t)*t_dot -l*sin(t)*t_dot; 0 0];
  M=[m 0 mc*l*sin(t);0 m -mc*l*cos(t);mc*l*sin(t) -mc*l*cos(t) J];
  Vm=[0 0 mc*l*t_dot*cos(t); 0 0 mc*l*t_dot*sin(t); 0 0 0];
  M_bar=transpose(S)*M*S;  
  V_bar=transpose(S)*M*S_dot+transpose(S)*Vm*S;

