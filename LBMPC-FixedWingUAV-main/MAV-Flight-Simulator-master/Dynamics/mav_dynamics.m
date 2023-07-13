function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

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
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

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
function sys=mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pe    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    % precalculating angles values
    Ctheta = cos(theta);
    Stheta = sin(theta);
    Ttheta = tan(theta);
    Cpsi   = cos(psi);
    Spsi   = sin(psi);
    Cphi   = cos(phi);
    Sphi   = sin(phi);
    
    % stroing J Matrix values in this file
    JJ = [
        P.Jx;...
        P.Jy;...
        P.Jz;...
        P.Jxz;...
        ];
      
    % Gamma values for Inertia Matrix
    G = JJ(1)*JJ(3) - JJ(4)^2;
    G1 = (JJ(4)*(JJ(1) -JJ(2)+JJ(3))) / G;
    G2 = (JJ(3)*(JJ(3)-JJ(2))+(JJ(4)^2)) / G;
    G3 = JJ(3) / G;
    G4 = JJ(4) / G;
    G5 = (JJ(3)-JJ(1)) / JJ(2);
    G6 = JJ(4) / JJ(2);
    G7 = ((JJ(1)-JJ(2))*JJ(1) + (JJ(4)^2)) / G;
    G8 = JJ(1) / G;
    
  
    pndot = u*(Ctheta*Cpsi) + v*(Sphi*Stheta*Cpsi - Cphi*Spsi) + w*(Cphi*Stheta*Cpsi + Sphi*Spsi);
    pedot = u*(Ctheta*Spsi) + v*(Sphi*Stheta*Spsi + Cphi*Cpsi) + w*(Cphi*Stheta*Spsi - Sphi*Cpsi);
    pddot = u*(-Stheta) + v*(Sphi*Ctheta) + w*(Cphi*Ctheta);
    udot = r*v - q*w + (P.mass)*fx;
    vdot = p*w - r*u + (P.mass)*fy;
    wdot = q*u - p*v + (P.mass)*fz;
    phidot = p + q*(Sphi*Ttheta) + r*(Cphi*Ttheta);
    thetadot = q*(Cphi) + r*(-Sphi);
    psidot = q*(Sphi/Ctheta) + r*(Cphi/Ctheta);
    pdot = G1*p*q - G2*q*r + G3*ell + G4*n;
    qdot = G5*p*r - G6*(p^2 - r^2) + m/(JJ(2));
    rdot = G7*p*r - G1*q*r + G4*ell + G8*n;
    

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

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

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

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

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
