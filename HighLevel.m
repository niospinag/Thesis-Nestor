clc
clear all 
%close all

%--------------------------------------------------------------------------
% SYSTEM
%--------------------------------------------------------------------------
%..........................................................................
% System constrains
%..........................................................................

% States
Ae = [1 0; -1 0; 0 1; 0 -1];
be = [100 100 100 100]';

% Energy

umax = 1;
umin = -1;

Au = [1 -1]';
bu = [umax -umin]';

% Disturbances
md = 0.03;
dmax = 3*md;
dmin = -3*md;

Ad = [1 -1]';
bd = [dmax -dmin]';


% Adjacency Matrix

M_A = [0 0; 0 1];

g1 = 2;
cx1 = 0;
cx2 = 0.2;
cx3 = 1;
cx21 = (1 - cx2);
cx4 = 1;
%..........................................................................
%--------------------------------------------------------------------------

Ts = 0.05;

A = [0 1; -1 0];
C = [0 1];

Al = [0.9988 0.05; -0.05 0.9988];
B_l = [0.16 0.08]';

A1 = Al;
B1o = [0.1 0.05]';

B1 = (g1+cx1)*B1o;
C1 = C;

A2 = Al;
B2o = [-0.08 -0.05]';

B2 = (cx2 + cx21)*B2o;
C2 = C;

A3 = Al;
B3o = [-0.19 -0.1]';
B3 = cx3*B3o;
C3 = C;

A4 = Al;
B4o = [0.13 0.08]';
B4 = cx4*B4o;
C4 = C;

%--------------------------------------------------------------------------
% MPC tunning
%--------------------------------------------------------------------------
%..........................................................................
% Backward finite horizon
%..........................................................................

L = 40;

%..........................................................................
% MHE weights
%..........................................................................

W = 1000;
Wd = 100;

%..........................................................................
% Forward finite horizon
%..........................................................................

T = 40;

%..........................................................................
% Weights
%..........................................................................

Q = [1e1 0; 0 1e1];
R = 0.01;

%..........................................................................
%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
% QUADRATIC OPTIMIZATION PROBLEM
%--------------------------------------------------------------------------
%..........................................................................
% Formulation
%..........................................................................

% States number
nx = size(A1,2);

% Control signals number
nu = size(B1,2);

% Disturbance number
nd = size(B1,2);

% Output number
nc = size(eye(2),1);

% Output number
nc1 = size( C1, 1 );
nc3 = size( C3, 1 );
nc2 = size( C2, 1 );
nc4 = size( C4, 1 );

%..........................................................................
% Construción Gxc, Guc
%..........................................................................

Gxc1 = zeros( nc1*(L+1), nx);
Guc1 = zeros( nc1*(L+1), nu*L);
Ggc1 = zeros( nc1*(L+1), nx);

Gxc2 = zeros( nc2*(L+1), nx);
Guc2 = zeros( nc2*(L+1), nu*L);
Ggc2 = zeros( nc2*(L+1), nx);

Gxc3 = zeros( nc3*(L+1), nx);
Guc3 = zeros( nc3*(L+1), nu*L);
Ggc3 = zeros( nc3*(L+1), nx);

Gxc4 = zeros( nc4*(L+1), nx);
Guc4 = zeros( nc4*(L+1), nu*L);
Ggc4 = zeros( nc4*(L+1), nx);

Gxc1( 1:nc1, :) = C1;
Gxc2( 1:nc2, :) = C2;
Gxc3( 1:nc3, :) = C3;
Gxc4( 1:nc4, :) = C4;


var11 = C1;
var21 = C2;
var31 = C3;
var41 = C4;

for i = 2:L+1
    
    var12 = var11;
    var22 = var21;
    var32 = var31;
    var42 = var41;
    
    var11 = var11*A1;
    var21 = var21*A2;
    var31 = var31*A3;
    var41 = var41*A4;
   
    
    Gxc1( nc1*(i-1)+1:nc1*i, : ) = var11;
    Gxc2( nc2*(i-1)+1:nc2*i, : ) = var21;
    Gxc3( nc3*(i-1)+1:nc3*i, : ) = var31;
    Gxc4( nc4*(i-1)+1:nc4*i, : ) = var41;
    
    for j = 1:L-i+2
        
        Guc1( nc1*(j+i-2)+1:nc1*(j+i-1) , nu*(j-1)+1:nu*j ) = (var11/A1)*B1;
        Guc2( nc2*(j+i-2)+1:nc2*(j+i-1) , nu*(j-1)+1:nu*j ) = (var21/A2)*B2;
        Guc3( nc3*(j+i-2)+1:nc3*(j+i-1) , nu*(j-1)+1:nu*j ) = (var31/A3)*B3;
        Guc4( nc4*(j+i-2)+1:nc4*(j+i-1) , nu*(j-1)+1:nu*j ) = (var41/A4)*B4;
%        

        Ggc1( nc1*(j+i-2)+1:nc1*(j+i-1) , nx*(j-1)+1:nx*j ) = var12;
        Ggc2( nc2*(j+i-2)+1:nc2*(j+i-1) , nx*(j-1)+1:nx*j ) = var22;
        Ggc3( nc3*(j+i-2)+1:nc3*(j+i-1) , nx*(j-1)+1:nx*j ) = var32;
        Ggc4( nc4*(j+i-2)+1:nc4*(j+i-1) , nx*(j-1)+1:nx*j ) = var42;        
    end
    
end

Guc1 = -Guc1;
Guc2 = -Guc2;
Guc3 = -Guc3;
Guc4 = -Guc4;

clear var11 var12 var21 var22 var31 var32 var41 var42
%
%..........................................................................
% Gx, Gu, Gw construction
%..........................................................................

Gx = zeros( nx*T, nx);
Gu = zeros( nx*T, nu*T);
Gg = zeros( nx*T, nx*T);

Gu2 = zeros( nx*T, nu*T);
Gu3 = zeros( nx*T, nu*T);
Gu4 = zeros( nx*T, nu*T);

var = eye(nx);

var31 = eye(nx);
var21 = eye(nx);
var3 = eye(nx);
var41 = eye(nx);

for i = 1:T
    
    var2 = var*B1;
    var32 = var31*B3;
    var22 = var21*B2;
    var42 = var41*B4;
    
    
    for j = 1:T-i+1
        Gu( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = var2;
        Gu3( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = var32;
        Gu2( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = var22;
        Gu4( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = var42;
%         
        Gg( nx*(j+i-2)+1:nx*(j+i-1) , nx*(j-1)+1:nx*j ) = var3;
    end
    
    var = var*A1;
    var3 = var3*A1;
    var31 = var31*A3;
    var21 = var21*A2;
    var41 = var41*A4;
    
    Gx( nx*(i-1)+1:nx*i  ,:) = var;
end

Gu = -Gu;
Gw = Gu;
Gu3 = -Gu3;
Gu2 = -Gu2;
Gu4 = -Gu4;
clear var var2


%..........................................................................
% W_hat construction
%..........................................................................

W_hat = kron(eye(L+1),W);

Wd_hat = kron( eye(L) , Wd );

%..........................................................................
% Q_hat, R_hat construction
%..........................................................................

Q_hat = kron( eye(T), Q );

R_hat = kron( eye(T), R );

%..........................................................................
% Hm construction
%..........................................................................

% MHE
Hm1 = [Gxc1'*W_hat*Gxc1  Gxc1'*W_hat*Guc1;
      Guc1'*W_hat*Gxc1  Guc1'*W_hat*Guc1+Wd_hat];
  
Hm1 =( Hm1+Hm1')/2;


Hm2 = [Gxc2'*W_hat*Gxc2 Gxc2'*W_hat*Guc2;
      Guc2'*W_hat*Gxc2 Guc2'*W_hat*Guc2+Wd_hat];
  
Hm2 =( Hm2+Hm2')/2;


Hm3 = [Gxc3'*W_hat*Gxc3 Gxc3'*W_hat*Guc3;
      Guc3'*W_hat*Gxc3 Guc3'*W_hat*Guc3+Wd_hat];
  
Hm3 =( Hm3+Hm3')/2;

Hm4 = [Gxc4'*W_hat*Gxc4 Gxc4'*W_hat*Guc4;
      Guc4'*W_hat*Gxc4 Guc4'*W_hat*Guc4+Wd_hat];
  
Hm4 =( Hm4+Hm4')/2;

%..........................................................................
% H construction
%..........................................................................

H = Gu'*Q_hat*Gu + R_hat;
H = ( H+H' )/2;

H2 = Gu2'*Q_hat*Gu2 + R_hat;
H2 = ( H2+H2' )/2;

H3 = Gu3'*Q_hat*Gu3 + R_hat;
H3 = ( H3+H3' )/2;

H4 = Gu4'*Q_hat*Gu4 + R_hat;
H4 = ( H4+H4' )/2;

%..........................................................................
% Constrains Augmented System
%..........................................................................

% MPC

Au_hat = kron( eye(T), Au );
bu_hat = kron( ones(T,1), bu );

AU = Au_hat;

AA = AU;

% MHE

Ae_hat_m = kron( eye(1), Ae );
be_hat_m = kron( ones(1,1), be );

Ad_hat_m = kron( eye(L), Ad );
bd_hat_m = kron( ones(L,1), bd );

AA_m = [Ae_hat_m zeros( size( Ae_hat_m , 1 ) , size( Ad_hat_m , 2 ) );
      zeros( size( Ad_hat_m , 1) , size( Ae_hat_m , 2 ) ) Ad_hat_m];
  
bU_m = [be_hat_m; bd_hat_m];
%

%..........................................................................
%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
% SIMULATION
%--------------------------------------------------------------------------
%..........................................................................
% Parameters
%..........................................................................

% Inicial conditions
x = [1 1]';

x1 = [3*randn(1,1) 0.5*randn(1,1)]';

x3 = [3*randn(1,1) 0.5*randn(1,1)]';

x2 = [3*randn(1,1) 0.5*randn(1,1)]';

x4 = [3*randn(1,1) 0.5*randn(1,1)]';

e = g1*( x - x1 );

e3 = cx3*(x1 - x3);

e2 = cx2*(x3 - x2) + cx21*(x1 - x2);

e4 = cx4*(x2 - x4);

% Final time
tf = 25;

% Time vector
t = 0:Ts:tf;
t = t';

%..........................................................................
% History vectors
%..........................................................................

% Real States
X = zeros( length(t)+1 , nx );
X( 1 , : ) = x;

X1 = zeros( length(t)+1 , nx );
X1( 1 , : ) = x1;

X3 = zeros( length(t)+1 , nx );
X3( 1 , : ) = x3;

X2 = zeros( length(t)+1 , nx );
X2( 1 , : ) = x2;

X4 = zeros( length(t)+1 , nx );
X4( 1 , : ) = x4;

E = zeros( length(t)+1 , nx );
Ee1 = E;
E( 1 , : ) = e;

E3 = zeros( length(t)+1 , nx );
Ee3 = E3;
E3( 1 , : ) = e3;
% 
E2 = zeros( length(t)+1 , nx );
Ee2 = E2;
E2( 1 , : ) = e2;

E4 = zeros( length(t)+1 , nx );
Ee4 = E4;
E4( 1 , : ) = e4;

% Output
Y1 = zeros( length(t), nc1);
Y1( 1, :) = C1*e;

Y2 = zeros( length(t), nc2);
Y2( 1, :) = C2*e2;

Y3 = zeros( length(t), nc3);
Y3( 1, :) = C3*e3;

Y4 = zeros( length(t), nc4);
Y4( 1, :) = C4*e4;

% Control Signal
U = zeros( length(t) , nu );

U3 = zeros( length(t) , nu );

U2 = zeros( length(t) , nu );

U4 = zeros( length(t) , nu );

U_leader = zeros( length(t) , nu );

% Disturbances

D1 = zeros( length(t) , nd );
D1e = D1;

D2 = zeros( length(t) , nd );
D2e = D2;

D3 = zeros( length(t) , nd );
D3e = D3;

D4 = zeros( length(t) , nd );
D4e = D4;
%..........................................................................
% System start
%..........................................................................

% Magnitud del ruido
%  Revisar la función con la que se esta generando el ruido
% Control Signal Leader
u_leader = @(k) 0.2 * sin((2 * pi/ 200)*k);

for i = 1:L
    
    
    x = Al*x + B_l*u_leader(i);    
    X ( i+1, : ) = x;
    U_leader(i, :) = u_leader(i);    
    
    di = md*randn(1,1);
    x1 = A1*x1 + B1o*di;
    X1 ( i+1, : ) = x1;
    D1( i, : ) = di;
    
    di = md*randn(1,1);
    x2 = A2*x2 + B2o*di;
    X2 ( i+1, : ) = x2;
    D2( i, : ) = di;
    
    di = md*randn(1,1);
    x3 = A3*x3 + B3o*di;
    X3 ( i+1, : ) = x3;
    D3( i, : ) = di;
    
    di = md*randn(1,1);
    x4 = A4*x4 + B4o*di;
    X4 ( i+1, : ) = x4;
    D4( i, : ) = di;
%     
    e = g1*( x - x1 );
    e2 = cx2*(x3 - x2) + cx21*(x1 - x2);
    e3 = cx3*(x1 - x3);
    e4 = cx4*(x2 - x4);
    
    E( i+1, :) = e;
    E2( i+1, :) = e2;
    E3( i+1, :) = e3;
    E4( i+1, :) = e4;
%     
    Y1( i+1, : ) = C1*e;
    Y2( i+1, : ) = C2*e2;
    Y3( i+1, : ) = C3*e3;
    Y4( i+1, : ) = C4*e4;
end



%--------------------------------------------------------------------------
% OPTIMIZATION
%--------------------------------------------------------------------------
%..........................................................................
% Parameters
%..........................................................................

theta = 0.9;

%..........................................................................
% Inicial values for solution for dual-primal problem
%..........................................................................
% MPC
u = 0*abs(rand( size(AA,2) , 1 ));

u3 = 0*abs(rand( size(AA,2) , 1 ));

u2 = 0*abs(rand( size(AA,2) , 1 ));

u4 = 0*abs(rand( size(AA,2) , 1 ));

% u = umax*ones( size(AA,2) , 1 );
Lambda = 1*ones( size(AA,1) , 1 );
Lambda3 = 1*ones( size(AA,1) , 1 );
Lambda2 = 1*ones( size(AA,1) , 1 );
Lambda4 = 1*ones( size(AA,1) , 1 );

s =  1./Lambda;
s3 =  1./Lambda;
s2 =  1./Lambda;
s4 =  1./Lambda;


% MHE

um = 0*abs(rand( size(AA_m,2) , 1 ));

u3m = 0*abs(rand( size(AA_m,2) , 1 ));
% 
u2m = 0*abs(rand( size(AA_m,2) , 1 ));

u4m = 0*abs(rand( size(AA_m,2) , 1 ));

% u = umax*ones( size(AA,2) , 1 );
Lambdam = 1*ones( size(AA_m,1) , 1 );
Lambda3m = 1*ones( size(AA_m,1) , 1 );
Lambda2m = 1*ones( size(AA_m,1) , 1 );
Lambda4m = 1*ones( size(AA_m,1) , 1 );

sm =  1./Lambdam;
s3m =  1./Lambda3m;
s2m =  1./Lambda2m;
s4m =  1./Lambda4m;

nloop = 0;

%..........................................................................
%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
% SIMULATION LOOP
%--------------------------------------------------------------------------

tic
for i = L+1:length(t)
    
    up1  = U( i-L:i-1, : );
    y1 = Y1( i-L:i, :);
    
    up2  = U2( i-L:i-1, : );
    y2 = Y2( i-L:i, :);
    
    
    up3  = U3( i-L:i-1, : );
    y3 = Y3( i-L:i, :);
    
    up4  = U4( i-L:i-1, : );
    y4 = Y4( i-L:i, :);
    
    up0 = U_leader( i-L:i-1, : );
    
    % control signal k-1
    uam = -1e6*ones( size(um,1) + size(u3m,1) + size(u2m,1) + size(u4m,1), 1);
    % Iteration number
    iter2 = 1;
    
    while norm([um;u2m;u3m;u4m] - uam) > 1e-1 && iter2 <= 10000
        
        uam = [um;u2m;u3m;u4m];
        iter2 = iter2 +1;
        
        gammam = g1*kron( eye(L), B_l)*(up0);
        gamma2m = cx2*kron( eye(L), B3o )*(up3 + u3m(nx+1:end , :)) + ...
                  cx21*kron( eye(L), B1o )*(up1 + um(nx+1:end , :));
        gamma3m = cx3*kron( eye(L), B1o )*(up1 + um(nx+1:end , :));
        gamma4m = cx4*kron( eye(L), B2o )*(up2 + u2m(nx+1:end , :));
        
        
        Fm1 = [up1'*Guc1'*W_hat*Gxc1+gammam'*Ggc1'*W_hat*Gxc1-y1'*W_hat*Gxc1  ...
            up1'*Guc1'*W_hat*Guc1+gammam'*Ggc1'*W_hat*Guc1-y1'*W_hat*Guc1];
        
        Fm2 = [up2'*Guc2'*W_hat*Gxc2+gamma2m'*Ggc2'*W_hat*Gxc2-y2'*W_hat*Gxc2  ...
            up2'*Guc2'*W_hat*Guc2+gamma2m'*Ggc2'*W_hat*Guc2-y2'*W_hat*Guc2];
        
        Fm3 = [up3'*Guc3'*W_hat*Gxc3+gamma3m'*Ggc3'*W_hat*Gxc3-y3'*W_hat*Gxc3  ...
            up3'*Guc3'*W_hat*Guc3+gamma3m'*Ggc3'*W_hat*Guc3-y3'*W_hat*Guc3];   
        
        Fm4 = [up4'*Guc4'*W_hat*Gxc4+gamma4m'*Ggc4'*W_hat*Gxc4-y4'*W_hat*Gxc4  ...
            up4'*Guc4'*W_hat*Guc4+gamma4m'*Ggc4'*W_hat*Guc4-y4'*W_hat*Guc4];   
        
        
        
        um = ( Hm1 + theta*( AA_m'*AA_m ) )  \ ( AA_m'*( theta*sm + Lambdam ) - Fm1' );
        
        sm = min( AA_m*um - ( 1/theta )*Lambdam , bU_m );
        
        Lambdam = Lambdam + theta*(-AA_m*um + sm);
        
        
        
        
        u2m = ( Hm2 + theta*( AA_m'*AA_m ) )  \ ( AA_m'*( theta*s2m + Lambda2m ) - Fm2' );
        
        s2m = min( AA_m*u2m - ( 1/theta )*Lambda2m , bU_m );
        
        Lambda2m = Lambda2m + theta*(-AA_m*u2m + s2m);
        
        
        
        
        u3m = ( Hm3 + theta*( AA_m'*AA_m ) )  \ ( AA_m'*( theta*s3m + Lambda3m ) - Fm3' );
        
        s3m = min( AA_m*u3m - ( 1/theta )*Lambda3m , bU_m );
        
        Lambda3m = Lambda3m + theta*(-AA_m*u3m + s3m);
        
        
        
        u4m = ( Hm4 + theta*( AA_m'*AA_m ) )  \ ( AA_m'*( theta*s4m + Lambda4m ) - Fm4' );
        
        s4m = min( AA_m*u4m - ( 1/theta )*Lambda4m , bU_m );
        
        Lambda4m = Lambda4m + theta*(-AA_m*u4m + s4m);
        
    end
   
    
    out = um( 1:nx, :);
    out3 = u3m( 1:nx, :);
    out2 = u2m( 1:nx, :);
    out4 = u4m( 1:nx, :);
    
    for j=1:L
        out = A1*out - B1*(up1(j) + um(nx+j,:)) + g1*B_l * up0(j);
        out3 = A3*out3 - B3*(up3(j) + u3m(nx+j,:)) + cx3*B1o*(up1(j) + um(nx+j,:));
        out2 = A2*out2 - B2*(up2(j) + u2m(nx+j,:)) + cx2*B3o*(up3(j) + u3m(nx+j,:)) ...
                                                   + cx21*B1o*(up1(j) + um(nx+j,:));
                                               
        out4 = A4*out4 - B4*(up4(j) + u4m(nx+j,:)) + cx4*B2o*(up2(j) + u2m(nx+j,:));
    end
    
    Ee1(i,:) = out;
    Ee3(i,:) = out3;
    Ee2(i,:) = out2;
    Ee4(i,:) = out4;
    
    e = out;
    e3 = out3;
    e2 = out2;
    e4 = out4;
    
    dd1 = md*randn(size(u));
    dd3 = md*randn(size(u));
    dd2 = md*randn(size(u));
    dd4 = md*randn(size(u));
    
%     
    %......................................................................
    % Constrains Augmented System
    %......................................................................
    
    bU = bu_hat;
    
    %......................................................................
    %----------------------------------------------------------------------
    
    %----------------------------------------------------------------------
    % SOLUTION FOR QUADRATIC OPTIMIZATION PROBLEM
    %----------------------------------------------------------------------
    
    % control signal k-1
    ua = -1e6*ones( size(u,1) + size(u3, 1) + size(u2, 1) + size(u4, 1) , 1);
    % Iteration number
    iter = 1;
    
    while norm([u;u2;u3;u4] - ua) > 1e-3 && iter <= 10000
        
        ua = [u;u2;u3;u4];
        
        gamma = g1*kron( eye(T), B_l)*(u_leader(i+1:i+T)');
        gamma3 = cx3*kron( eye(T), B1o )*(u+dd1);
        gamma2 = cx2*kron( eye(T), B3o )*(u3+dd3) + cx21*kron( eye(T), B1o )*(u+dd1);
        gamma4 = cx4*kron( eye(T), B2o )*(u2+dd2);
        
        
        F = e'*Gx'*Q_hat*Gu + gamma'*Gg'*Q_hat*Gu + dd1'*Gu'*Q_hat*Gu;
        F3 = e3'*Gx'*Q_hat*Gu3 + gamma3'*Gg'*Q_hat*Gu3 + dd3'*Gu3'*Q_hat*Gu3;
        F2 = e2'*Gx'*Q_hat*Gu2 + gamma2'*Gg'*Q_hat*Gu2 + dd2'*Gu2'*Q_hat*Gu2;
        F4 = e4'*Gx'*Q_hat*Gu4 + gamma4'*Gg'*Q_hat*Gu4 + dd4'*Gu4'*Q_hat*Gu4;
        
        
        u = ( H + theta*( AA'*AA ) )  \ ( AA'*( theta*s + Lambda ) - F' );
        
        s = min( AA*u - ( 1/theta )*Lambda , bU );
        
        Lambda = Lambda + theta*(-AA*u + s);%
        
        
        
        u2 = ( H2 + theta*( AA'*AA ) )  \ ( AA'*( theta*s2 + Lambda2 ) - F2' );
        
        s2 = min( AA*u2 - ( 1/theta )*Lambda2 , bU );
        
        Lambda2 = Lambda2 + theta*(-AA*u2 + s2);  
        
        
        
        u3 = ( H3 + theta*( AA'*AA ) )  \ ( AA'*( theta*s3 + Lambda3 ) - F3' );
        
        s3 = min( AA*u3 - ( 1/theta )*Lambda3 , bU );
        
        Lambda3 = Lambda3 + theta*(-AA*u3 + s3);
        
        
        
        u4 = ( H4 + theta*( AA'*AA ) )  \ ( AA'*( theta*s4 + Lambda4 ) - F4' );
        
        s4 = min( AA*u4 - ( 1/theta )*Lambda4 , bU );
        
        Lambda4 = Lambda4 + theta*(-AA*u4 + s4);
        
        
        
        iter = iter + 1;
        
    end
    
    %----------------------------------------------------------------------
    % System, state, and control update 
    %----------------------------------------------------------------------
    
    % System Leader
    x = Al*x +  B_l*u_leader(i);
    
    % State Leader
    X( i+1 , : ) = x;
    U_leader( i, :) = u_leader(i);
    
    % System agent 1
    di = md*randn(1,1);
    D1( i, : ) = di;
    x1 = A1*x1 + B1o*(u(1,1) + di);
    
    % State agent 1
    X1( i+1 , : ) = x1;
    
    
    
    
    % System agent 2
    di = md*randn(1,1);
    D2( i, : ) = di;
    x2 = A2*x2 + B2o*(u2(1,1) + di);
    
    % State agent 2
    X2( i+1 , : ) = x2;
    
    
  
    
    % System agent 3
    di = md*randn(1,1);
    D3( i, : ) = di;
    x3 = A3*x3 + B3o*(u3(1,1) + di);
    
    % State agent 3
    X3( i+1 , : ) = x3;
    
    
    
    
     % System agent 3
    di = md*randn(1,1);
    D4( i, : ) = di;
    x4 = A4*x4 + B4o*(u4(1,1) + di);
    
    % State agent 3
    X4( i+1 , : ) = x4;
    
    
    

    e = g1*(x - x1);
    Y1(i+1) = C1*e;
    
    E(i+1, :) = e;
    
    
    
    
    e2 = cx2*(x3 - x2) + cx21*(x1 - x2);
    Y2( i+1 , : ) = C2*e2;
    
    E2(i+1, :) = e2;
    
    
    
    
    e3 = cx3*(x1 - x3);
    
    E3(i+1, :) = e3;
    Y3(i+1, :) = C3*e3;
    
    
    e4 = cx4*(x2 - x4);
    
    E4(i+1, :) = e4;
    Y4(i+1, :) = C4*e4;
    
    
    % Control
    U( i , :) = u(1,1);
    
    U2( i , :) = u2(1,1);
    
    U3( i , :) = u3(1,1);
    
    U4( i , :) = u4(1,1);
    
    nloop = nloop + 1;
end
toc

%--------------------------------------------------------------------------
% k = 0:length(t);
% k = k';

k = 0:300;
k = k';

%--------------------------------------------------------------------------

figure(2)

subplot(2,1,1)

hold on
plot( k, X(k+1,1), 'LineWidth', 1.5)    
plot( k, X1(k+1,1), 'LineWidth', 1.5)
plot( k, X2(k+1,1), 'LineWidth', 1.5)
plot( k, X3(k+1,1), 'LineWidth', 1.5)
plot( k, X4(k+1,1), 'LineWidth', 1.5)
hold off
xlim([0 k(end)])
grid on
grid minor
set(gca,'FontSize', 15)
ylabel('$x_1$', 'Interpreter', 'latex', 'FontSize', 20)


subplot(2,1,2)

hold on
plot( k, X(k+1,2), 'LineWidth', 1.5)
plot( k, X1(k+1,2), 'LineWidth', 1.5)
plot( k, X2(k+1,2), 'LineWidth', 1.5)
plot( k, X3(k+1,2), 'LineWidth', 1.5)
plot( k, X4(k+1,2), 'LineWidth', 1.5)
hold off
xlim([0 k(end)])
legend({'$A_0$', '$A_1$', '$A_2$', '$A_3$', '$A_4$'}, 'Interpreter',...
        'latex', 'Orientation', 'horizontal','FontSize', 15)   
grid on
grid minor
set(gca,'FontSize', 15)
ylabel('$x_2$', 'Interpreter', 'latex', 'FontSize', 20)
xlabel('$k$', 'Interpreter', 'latex', 'FontSize', 20)

%--------------------------------------------------------------------------

figure(3)

subplot(2,1,1)

hold on
plot( k, E(k+1,1), 'LineWidth', 1.5)
plot( k, E2(k+1,1), 'LineWidth', 1.5)
plot( k, E3(k+1,1), 'LineWidth', 1.5)
plot( k, E4(k+1,1), 'LineWidth', 1.5)
hold off
xlim([0 k(end)])
grid on
grid minor
set(gca,'FontSize', 15)
ylabel('$\varepsilon_1$', 'Interpreter', 'latex', 'FontSize', 20)


subplot(2,1,2)

hold on
plot( k, E(k+1,2), 'LineWidth', 1.5)
plot( k, E2(k+1,2), 'LineWidth', 1.5)
plot( k, E3(k+1,2), 'LineWidth', 1.5)
plot( k, E4(k+1,2), 'LineWidth', 1.5)
hold off
xlim([0 k(end)])
legend({'$A_1$', '$A_2$', '$A_3$', '$A_4$'}, 'Interpreter',...
        'latex', 'Orientation', 'horizontal','FontSize', 15)   
grid on
grid minor
set(gca,'FontSize', 15)
ylabel('$\varepsilon_2$', 'Interpreter', 'latex', 'FontSize', 20)
xlabel('$k$', 'Interpreter', 'latex', 'FontSize', 20)

%--------------------------------------------------------------------------
% k = 0:length(t)-1;

%--------------------------------------------------------------------------

figure(4)

hold on

stairs(k, U_leader(k+1), 'LineWidth', 1.5)
stairs(k, U(k+1), 'LineWidth', 1.5)
stairs(k, U2(k+1), 'LineWidth', 1.5)
stairs(k, U3(k+1), 'LineWidth', 1.5)
stairs(k, U4(k+1), 'LineWidth', 1.5)
hold off
xlim([0 k(end)])
ylim([-1.1 1.1])
grid on
grid minor
set(gca,'FontSize', 15)
ylabel('$u$', 'Interpreter', 'latex', 'FontSize', 20)
xlabel('$k$', 'Interpreter', 'latex', 'FontSize', 20)
legend({'$u_0$','$u_1$', '$u_2$', '$u_3$', '$u_4$'}, 'Interpreter',...
        'latex', 'Orientation', 'horizontal','FontSize', 15)   


