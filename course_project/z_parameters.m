%% 仿真参数
clear all
clc
%%
para.T                          = 1e-3;                     %仿真步长
%%
%mass properties
para.m                          = 20500;                    % 单位: lbs
para.Ixx                        = 9496;                     % 单位：slug-ft^2
para.Iyy                        = 55814;                    % 单位：slug-ft^2
para.Izz                        = 63100;                    % 单位：slug-ft^2
para.Ixz                        = 982;                      % 单位：slug-ft^2
para.g0                         = 32.15;                    % 单位: ft/s^2
% wing dimensions
para.span                       = 30;                       % 翼展                                    单位: ft
para.area                       = 300;                      % 机翼面积                                单位: ft^2
para.mac                        = 11.32;                    % 平均气动弦长(mean aerodynamic chord)    单位：ft

%reference CG location
para.xcg                        = 0.35*para.mac;

%operator surface
para.dt                         = 0.5;                      % 油门开合
para.da                         = 10;                       % 副翼
para.de                         = 10;                       % 升降舵
para.dr                         = 10;                       % 方向舵



% 气动系数
load    CLab
load    CLP
load    CLR
load    CMad
load    CMQ
load    CNab
load    CNP
load    CNR
load    CXad
load    CXQ
load    CYP
load    CYR
load    CZ
load    CZQ
load    DCL_da20
load    DCL_dr30
load    DCN_da20
load    DCN_dr30
load    Tidle
load    Tmax
load    Tmil

para.CLab       = CLab;
para.CLP        =CLP;
para.CLR        =CLR;
para.CMad       =CMad;
para.CMQ        =CMQ;
para.CNab       =CNab;
para.CNP        =CNP;
para.CNR        =CNR;
para.CXad       =CXad;
para.CXQ        =CXQ;
para.CYP        =CYP;
para.CYR        =CYR;
para.CZ         =CZ;
para.CZQ        =CZQ;
para.DCL_da20   =DCL_da20;
para.DCL_dr30   =DCL_dr30;
para.DCN_da20   =DCN_da20;
para.DCN_dr30   =DCN_dr30;
para.Tidle      =Tidle;
para.Tmax       =Tmax;
para.Tmil       =Tmil;