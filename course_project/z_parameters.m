%% �������
clear all
clc
%%
para.T                          = 1e-3;                     %���沽��
%%
%mass properties
para.m                          = 20500;                    % ��λ: lbs
para.Ixx                        = 9496;                     % ��λ��slug-ft^2
para.Iyy                        = 55814;                    % ��λ��slug-ft^2
para.Izz                        = 63100;                    % ��λ��slug-ft^2
para.Ixz                        = 982;                      % ��λ��slug-ft^2
para.g0                         = 32.15;                    % ��λ: ft/s^2
% wing dimensions
para.span                       = 30;                       % ��չ                                    ��λ: ft
para.area                       = 300;                      % �������                                ��λ: ft^2
para.mac                        = 11.32;                    % ƽ�������ҳ�(mean aerodynamic chord)    ��λ��ft

%reference CG location
para.xcg                        = 0.35*para.mac;

%operator surface
para.dt                         = 0.5;                      % ���ſ���
para.da                         = 10;                       % ����
para.de                         = 10;                       % ������
para.dr                         = 10;                       % �����



% ����ϵ��
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