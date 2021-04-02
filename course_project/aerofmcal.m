function [sys,x0,str,ts] = aerofmcal(t,x,u,flag,para)

switch flag

  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;

  case 1
    sys=mdlDerivatives(t,x,u);

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
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes


sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;   % Fxa Fya Fza La Ma Na Fxt
sizes.NumInputs      = 8;   % alpha delta_e beta Ma h 
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

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
end
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];
end
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];
end
% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,para)
%Output: Fxa Fya Fza La Ma Na Fxt
%Input:  Ma h   u v w p q r de dr da dt
alpha   = u(1);
beta    = u(2);
Ma      = u(3);
h       = u(4);
da      = u(5);
de      = u(6);
dr      = u(7);
dt      = u(8);

V=sqrt(u^2+v^2+w^2);
c=para.mac; %c=c_bar
b=para.span;
S=para.aera;
xcg=para.xcg;

% [Tem, vs, pas, rou] = atmoscoesa(h);% Tem 当地温度 vs 当地音速 pas 当地压强 rou 当地密度(kg/m^3)
% rou = rou*0.0624;                   % 换算为英制单位 lb/ft^3
% q=0.5*rou*V;                        % 动压计算


%yi = interp1(x,Y,xi) 
%使用线性插值返回一维函数 Y 在列向量 xi 各点的值。
%向量 x 指定基础区间的坐标。输出 yi 的长度等于 xi 的长度。

%推力T与功率P

tidle    = interp2(para.Tidle(1,2:7),para.Tidle(2:7,1),para.Tidle(2:7,2:7),h,Ma);
tmil     = interp2(para.Tmil(1,2:7),para.Tmil(2:7,1),para.Tmil(2:7,2:7),h,Ma);
tmax     = interp2(para.Tmax(1,2:7),para.Tmax(2:7,1),para.Tmax(2:7,2:7),h,Ma);

    if dt<0.77
        P = 64.94*dt;
    else
        P = 217.38*dt - 117.38;
    end
    
    if P<50
        T = tidle+(tmil-tidle)*(P/50);
    else
        T=tmil+(tmax-tmil)*(P-50)/50;
    end



%气动系数====单位全都是度
cxq = interp1(para.CXQ(:,1),para.CXQ(:,2),alpha);
cyr = interp1(para.CYR(:,1),para.CYR(:,2),alpha);
cyp = interp1(para.CYP(:,1),para.CYP(:,2),alpha);
czq = interp1(para.CZQ(:,1),para.CZQ(:,2),alpha);
clr = interp1(para.CLR(:,1),para.CLR(:,2),alpha);
clp = interp1(para.CLP(:,1),para.CLP(:,2),alpha);
cmq = interp1(para.CMQ(:,1),para.CMQ(:,2),alpha);
cnr = interp1(para.CNR(:,1),para.CNR(:,2),alpha);
cnp = interp1(para.CNP(:,1),para.CNP(:,2),alpha);
cz  = interp1(para.CZ(:,1),para.CZ(:,2),alpha);

cxad        = interp2(para.CXad(1,2:6),para.CXad(2:13,1),para.CXad(2:13,2:6),de,alpha);
cmad        = interp2(para.CMad(1,2:6),para.CMad(2:13,1),para.CMad(2:13,2:6),de,alpha);
clab        = interp2(para.CLab(1,2:8),para.CLab(2:13,1),para.CLab(2:13,2:8),beta,alpha);
cnab        = interp2(para.CNab(1,2:8),para.CNab(2:13,1),para.CNab(2:13,2:8),beta,alpha);

dcl_da20    = interp2(para.DCL_da20(1,2:8),para.DCL_da20(2:13,1),para.DCL_da20(2:13,2:8),beta,alpha);
dcl_dr30    = interp2(para.DCL_dr30(1,2:8),para.DCL_dr30(2:13,1),para.DCL_dr30(2:13,2:8),beta,alpha);
dcn_da20    = interp2(para.DCN_da20(1,2:8),para.DCN_da20(2:13,1),para.DCN_da20(2:13,2:8),beta,alpha);
dcn_dr30    = interp2(para.DCN_dr30(1,2:8),para.DCN_dr30(2:13,1),para.DCN_dr30(2:13,2:8),beta,alpha);


% CXA=cxad+(c*q*cxq)/(2*V);
% CZA=cz*(1-(beta/57.3)^2)-0.19*de/25+c*q/(2*V)*czq;
% CYA=(-0.02*beta+0.21*da/20+0.86*dr/30)+b/(2*V)*(cyr*r+cyp*p);
% CMA=cmad+c*q/(2*V)*cmq ;
% CLA=clab+dcl_da20*da/20+dcl_dr30*dr/30+b/(2*V)*(clr*r+clp*p);
% CNA=cnab+dcn_da20*da/20+dcn_dr30*dr/30+b/(2*V)*(cnr*r+cnp*p);

% FXA=CXA*q*S;
% FYA=CYA*q*S;
% FZA=CZA*q*S;
% LA=CLA*q*S*b;
% MA=CMA*q*S*c;
% NA=CNA*q*S*b;
% 
% FXT=T;
% FYT=0;
% FZT=0;
% LT=0;
% MT=0;
% NT=0;
%sys = [FXA,FYA,FZA, LA,MA,NA, FXT];
sys = 0;
end
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

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
end
% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

end% end mdlTerminate
