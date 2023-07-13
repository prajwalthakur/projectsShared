global P;
%global x;
% P.xCL=[-3,0,3,5,6,8,10,11];
% P.yCL=[-0.08501,0.12496,0.34844,0.5,0.61,0.8,0.9,1];
% P.xCD=[0,3,5,7,9,10,11,12,13];
% P.yCD=[0.03489,0.03745,0.042,0.06,0.08,0.09,0.105,0.12,0.13];
% P.xCM=[0,5,10,13];
% P.yCM=[0,-0.15,-0.25,-0.3];



P.gravity = 9.81; % m/s^2
%%%%VTOL SPECIAL%%%%
%VTOL
P.l= 0.6; %given in paper=0.6m
P.l2= 0.6;
P.zeta=pi/3; %assumed 60 degree
%motor configuration random
P.km = 0.15;            %torque constant
P.kf= 0.2;            %cofficient of friction


%physical parameters of airframe
P.mass = 13.5; % kg
%P.mass=47/9.8;
P.Jx   = 0.8244; % kg m^2
%P.Jx=   0.26198;
P.Jy   = 1.135; % kg m^2
%P.Jy =  0.05;
P.Jz   = 1.759; % kg m^2
%P.Jz=0.5525;
%P.Jxz  = 0.1204; % kg m^2
P.Jxz=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aerodynamic coefficients
P.S_wing        = 0.55; %total area off the wing
P.b             = 2.8956;
P.c             = 0.18994; %aerodynamic cord
P.S_prop        = 0.2027; %area swept by propellor
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prop parameters
P.D_prop = 20*(0.0254);     % prop diameter in m

% Motor parameters
P.K_V = 145.;                   % from datasheet RPM/V
P.KQ = (1. / P.K_V) * 60. / (2. * pi);  % KQ in N-m/A, V-s/rad
P.R_motor = 0.042;              % ohms
P.i0 = 1.5;                     % no-load (zero-torque) current (A)


% Inputs
P.ncells = 12.;
P.V_max = 3.7 * P.ncells;  % max voltage for specified number of battery cells

% Coeffiecients from prop_data fit
P.C_Q2 = -0.01664;
P.C_Q1 = 0.004970;
P.C_Q0 = 0.005230;
P.C_T2 = -0.1079;
P.C_T1 = -0.06044;
P.C_T0 = 0.09357;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sample time
%P.Ts = 10;
% autopilot sample rate
P.Ts = 0.008;

%for Va=20:1:16   
[A,B,x_trim,u_trim] = param_chap5(26); 
   % eig_A_lon = [2.011;3.012;4.013];
    %eig_A_lat = [2.001;3.002;4.003];
    %eig_A_lon
%     x=horzcat(eig_A_lon,eig_A_lat);
%     %if all(real(eig_A_lon)<=0)
%        % ~all(real(eig_A_lon)<0)
%         v=["Va=",num2str(Va)];
%         disp(v);
%         header = {'eig_A_lon','eig_A_lat'};
%         xForDisplay = [header; num2cell(x)];
%         disp(xForDisplay);
   % end
    
%     sprintf(' %8.2f %8.3f\n', );
%     
    
    
%end
