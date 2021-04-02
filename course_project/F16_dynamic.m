function [sys,x0,str,ts] = F16_dynamic(t,x,u,flag,para)
%%
switch flag

  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;

  case 1
    sys=mdlDerivatives(t,x,u,para);

  case 2
    sys=mdlUpdate(t,x,u);


  case 3
    sys=mdlOutputs(t,x,u,para);


  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);


  case 9
    sys=mdlTerminate(t,x,u);


  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
%%
%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 15;
sizes.NumInputs      = 12;   %待定
sizes.DirFeedthrough = 1;   %模块否存在直接贯连(分子阶次大于等于分母阶次时需设为1)
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [0 0 0 0 0 0 0 0 0 0 0 0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,para)
%%
% u(1) = Fxa; u(2) = Fya; u(3) = Fza; 
% u(4) = Ftx; u(5) = Fty; u(6) = Ftz;
% u(7) = La;  u(8) = Lt; 
% u(9) = Ma;  u(10)= Mt;
% u(11)= Na;  u(12)= Nt;

Ixx = para.Ixx;
Iyy = para.Iyy;
Izz = para.Izz;
Ixz = para.Ixz;

L=u(7)+u(8);
M=u(9)+u(10);
N=u(11)+u(12);

%pos_x = x(1) pos_y = x(2) pos_z = x(3)
vx=x(4);        vy=x(5);            vz=x(6);
phi = x(7);     theta = x(8);       psi = x(9);
% Φ             θ                   ψ
p=x(10);        q=x(11);            r=x(12);

%position in the inertial frame 
dx = vx;
dy = vy;
dz = vz;

%velocity of inertial frame
du = (u(1)+u(4)-para.m*para.g0*sin(theta))/para.m-vz*q+r*vy;
dv = (u(2)+u(5)+para.m*para.g0*sin(phi)*cos(theta))/para.m-r*vx+p*vz;
dw = (u(3)+u(6)+para.m*para.g0*cos(phi)*cos(theta))/para.m-p*vy+q*vx;
%Eulerian angles
dphi    = p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
dtheta  = q*cos(phi)-r*sin(phi);
dpsi    = q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta));
%angle rate of body frame
dp = (Ixz*(Ixx-Iyy+Izz)*p*q+(Izz*(Iyy-Izz)-Ixz*Ixz)*q*r+Izz*(L)+Ixz*(N))/(Ixx*Izz - Ixz*Ixz)  ;
dq = ((Izz-Ixx)*p*r+(r*r-p*p)*Ixz+(M))/Iyy   ;
dr = ((Ixx*Ixx-Ixx*Iyy+Ixz*Ixz)*p*q+Ixz*(Iyy-Izz-Ixx)*q*r+Ixz*(L)+Ixx*N)/(Ixx*Izz-Ixz^Ixz)  ;
%
sys = [dx,dy,dz , du,dv,dw , dphi,dtheta,dpsi , dp,dq,dr];
sys = real(sys);

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)
%%

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,para)
%%
%pos_x = x(1) pos_y = x(2) pos_z = x(3)
%vx=x(4); vy=x(5); vz=x(6);

phi = x(7);     theta = x(8);       psi = x(9);
% Φ             θ                   ψ
p=x(10);        q=x(11);            r=x(12);

dphi    = p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
dtheta  = q*cos(phi)-r*sin(phi);
dpsi    = q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta));

sys=[x(1),x(2),x(3),   x(4),x(5),x(6),    phi,theta,psi,  p,q,r  dphi,dtheta,dpsi];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)
%%

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)
%%

sys = [];

% end mdlTerminate
