clc
clear 

%% Dinamyc System
%--------------------------------------------------------------------------
% SYSTEM
%--------------------------------------------------------------------------
%..........................................................................
% System constrains
%..........................................................................
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
% creando la ecuacion (5)
Ts = 0.05; %delta time

A = [0 1; -1 0];

C = [1 0; 0 1];

Al = [0.9988 0.05; -0.05 0.9988];
% Al = [1 0; 0 1];
A1 = Al;
A2 = Al;
A3 = Al;
A4 = Al;

B_l = [1 0; 0 1];

B1o = [1 0; 0 1];
B1 = (g1+cx1)*B1o;

B2o = [1 0; 0 1];
B2 = (cx2 + cx21)*B2o;

B3o = [1 0; 0 1];
B3 = cx3*B3o;

B4o = [1 0; 0 1];
B4 = cx4*B4o;

C1 = C;
C2 = C;
C3 = C;
C4 = C;

% Disturbances
md = 0.03;
dmax = 3*md;
dmin = -3*md;

Ad = [1 0; -1 0; 0 1; 0 -1];
bd = [dmax -dmin dmax -dmin]';

% Energy
umax = 5;
umin = -5;

% Constraints
A_err = [1 0; -1 0; 0 1; 0 -1];
b_err = [100 100 100 100]';

Au = [1 0; -1 0; 0 1; 0 -1];
bu = [umax -umin umax -umin]';




%% MPC Tuning

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

%--------------------------------------------------------------------------
% MPC tunning
%--------------------------------------------------------------------------
%..........................................................................
% Backward finite horizon
%..........................................................................

L = 40;%Horizonte del MHE
T = 40;%horizonte del mpc
%..........................................................................
% MHE weights
%..........................................................................

W = 1000*eye(nu);
Wd = 100;

%..........................................................................
% MPC Weights
%..........................................................................

Q = 10*eye(nx);
R = 0.1*eye(nu);

%--------------------------------------------------------------------------
% QUADRATIC OPTIMIZATION PROBLEM
%--------------------------------------------------------------------------
%..........................................................................
% Formulation
%..........................................................................





%%  Construction A_, B_, P_
%  Equation (8)
A_ = zeros( nx*T, nx); %Gx

B_1 = zeros( nx*T, nu*T); %Gu
B_2 = zeros( nx*T, nu*T); %Gu
B_3 = zeros( nx*T, nu*T); %Gu
B_4 = zeros( nx*T, nu*T); %Gu

P_ = zeros( nx*T, nx*T); %Gg
for i = 1:T
    A_( nx*(i-1)+1:nx*i  ,:) = A1^i;
    for j = 1:T-i+1
        B_1( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = -(A1^(i-1))*B1;
        B_2( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = -(A2^(i-1))*B2;
        B_3( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = -(A3^(i-1))*B3;
        B_4( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = -(A4^(i-1))*B4;
        
        P_( nx*(j+i-2)+1:nx*(j+i-1) , nx*(j-1)+1:nx*j ) = (A1^(i-1));
    end
        
end

%% MPC
% equation 11

%..........................................................................
% Q_hat, R_hat construction
%..........................................................................

Q_hat = kron( eye(T), Q );

R_hat = kron( eye(T), R );


H = B_1'*Q_hat*B_1 + R_hat;
H = ( H+H' )/2;

H2 = B_2'*Q_hat*B_2 + R_hat;
H2 = ( H2+H2' )/2;

H3 = B_3'*Q_hat*B_3 + R_hat;
H3 = ( H3+H3' )/2;

H4 = B_4'*Q_hat*B_4 + R_hat;
H4 = ( H4+H4' )/2;


%% Costruction A_e, B_e, P_e
% Equation (15)
Ae_1 = zeros( nc1*(L+1), nx); %Gxc1
Ae_2 = zeros( nc1*(L+1), nx); %Gxc2
Ae_3 = zeros( nc1*(L+1), nx); %Gxc3
Ae_4 = zeros( nc1*(L+1), nx); %Gxc4

Be_1 = zeros( nc1*(L+1), nu*L); %Guc1
Be_2 = zeros( nc1*(L+1), nu*L); %Guc2
Be_3 = zeros( nc1*(L+1), nu*L); %Guc3
Be_4 = zeros( nc1*(L+1), nu*L); %Guc4

Pe_1 = zeros( nc1*(L+1), nx); %Ggc1
Pe_2 = zeros( nc1*(L+1), nx); %Ggc2
Pe_3 = zeros( nc1*(L+1), nx); %Ggc3
Pe_4 = zeros( nc1*(L+1), nx); %Ggc4

Ae_1( 1:nc1, :) = C1;
for i = 2:L+1
    Ae_1( nc1*(i-1)+1:nc1*i , : ) = C1*(A1^(i-1));
    Ae_2( nc1*(i-1)+1:nc1*i , : ) = C2*(A2^(i-1));
    Ae_3( nc1*(i-1)+1:nc1*i , : ) = C3*(A3^(i-1));
    Ae_4( nc1*(i-1)+1:nc1*i , : ) = C4*(A4^(i-1));
    for j = 1:L-i+2
        Be_1( nc1*(j+i-2)+1:nc1*(j+i-1) , nu*(j-1)+1:nu*j ) = -(C1*(A1^(i-2)))*B1;
        Be_2( nc1*(j+i-2)+1:nc1*(j+i-1) , nu*(j-1)+1:nu*j ) = -(C2*(A2^(i-2)))*B2;
        Be_3( nc1*(j+i-2)+1:nc1*(j+i-1) , nu*(j-1)+1:nu*j ) = -(C3*(A3^(i-2)))*B3;
        Be_4( nc1*(j+i-2)+1:nc1*(j+i-1) , nu*(j-1)+1:nu*j ) = -(C4*(A4^(i-2)))*B4;
        
        Pe_1( nc1*(j+i-2)+1:nc1*(j+i-1) , nx*(j-1)+1:nx*j ) = (C1*(A1^(i-2))); 
        Pe_2( nc1*(j+i-2)+1:nc1*(j+i-1) , nx*(j-1)+1:nx*j ) = (C2*(A2^(i-2))); 
        Pe_3( nc1*(j+i-2)+1:nc1*(j+i-1) , nx*(j-1)+1:nx*j ) = (C3*(A3^(i-2))); 
        Pe_4( nc1*(j+i-2)+1:nc1*(j+i-1) , nx*(j-1)+1:nx*j ) = (C4*(A4^(i-2))); 
    end
end



%%  MHE
% equation 18

%..........................................................................
% W_hat construction
%..........................................................................

W_hat = kron(eye((L+1)),W);

Wd_hat = kron( eye(size(Be_2,2)) , Wd );


Hm1 = [Ae_1'*W_hat*Ae_1  Ae_1'*W_hat*Be_1;
      Be_1'*W_hat*Ae_1  Be_1'*W_hat*Be_1+Wd_hat];
  
Hm1 =( Hm1+Hm1')/2;


Hm2 = [Ae_2'*W_hat*Ae_2 Ae_2'*W_hat*Be_2;
      Be_2'*W_hat*Ae_2 Be_2'*W_hat*Be_2+Wd_hat];
  
Hm2 =( Hm2+Hm2')/2;


Hm3 = [Ae_3'*W_hat*Ae_3 Ae_3'*W_hat*Be_3;
      Be_3'*W_hat*Ae_3 Be_3'*W_hat*Be_3+Wd_hat];
  
Hm3 =( Hm3+Hm3')/2;

Hm4 = [Ae_4'*W_hat*Ae_4 Ae_4'*W_hat*Be_4;
      Be_4'*W_hat*Ae_4 Be_4'*W_hat*Be_4+Wd_hat];
  
Hm4 =( Hm4+Hm4')/2;

%..........................................................................
%% Constrains Augmented System
%..........................................................................

% MPC

Au_hat = kron( eye(T), Au );
bu_hat = kron( ones(T,1), bu );

AU = Au_hat;

AA = AU;

% MHE

Ae_hat_m = kron( eye(1), A_err );
be_hat_m = kron( ones(1,1), b_err );

Ad_hat_m = kron( eye(L), Ad );
bd_hat_m = kron( ones(L,1), bd );

AA_m = [Ae_hat_m zeros( size( Ae_hat_m , 1 ) , size( Ad_hat_m , 2 ) );
      zeros( size( Ad_hat_m , 1) , size( Ae_hat_m , 2 ) ) Ad_hat_m];
  
bU_m = [be_hat_m; bd_hat_m];
%

%..........................................................................
%--------------------------------------------------------------------------



%--------------------------------------------------------------------------
%% Initial Conditions
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
%% History vectors
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
% Y1 = zeros( nc1*length(t), 1);
Y1( 1, :) = C1*e;

Y2 = zeros( length(t), nc1);
Y2( 1, :) = C2*e2;

Y3 = zeros( length(t), nc1);
Y3( 1, :) = C3*e3;

Y4 = zeros( length(t), nc1);
Y4( 1, :) = C4*e4;

% Control Signal
U = zeros( length(t) , nu );

U3 = zeros( length(t) , nu );

U2 = zeros( length(t) , nu );

U4 = zeros( length(t) , nu );

U_leader = zeros( length(t) , nu );

% Disturbances

D1 = zeros( length(t) , nd );
% D1e = D1;

D2 = zeros( length(t) , nd );
% D2e = D2;

D3 = zeros( length(t) , nd );
% D3e = D3;

D4 = zeros( length(t) , nd );
% D4e = D4;
%..........................................................................
%% System start
%..........................................................................

% Magnitud del ruido
%  Revisar la función con la que se esta generando el ruido
% Control Signal Leader
% u_leader = @(k) 0.2 * sin((2 * pi/ 200)*k)*ones(nu,1);

for i = 1:L
    
    
    x = Al*x + B_l*u_leader(i);    
    X ( i+1, : ) = x;
    U_leader(i, :) = u_leader(i);    
    
    di = md*randn(nu,1);
    x1 = A1*x1 + B1o*di;
    X1 ( i+1, : ) = x1;
    D1( i, : ) = di;
    
    di = md*randn(nu,1);
    x2 = A2*x2 + B2o*di;
    X2 ( i+1, : ) = x2;
    D2( i, : ) = di;
    
    di = md*randn(nu,1);
    x3 = A3*x3 + B3o*di;
    X3 ( i+1, : ) = x3;
    D3( i, : ) = di;
    
    di = md*randn(nu,1);
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
% plot(X1(1:L),1:L)

%%
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
%% SIMULATION LOOP
%--------------------------------------------------------------------------

tic
for i = L+1:length(t)
 
    up1  = reshape(U( i-L:i-1, : )', [L*nu, 1]);
    y1 = reshape(Y1(i-L:i, :)', [(L+1)*nu, 1]);
    
    up2  = reshape(U2( i-L:i-1, : )', [L*nu, 1]);
    y2 = reshape(Y2(i-L:i, :)', [(L+1)*nu, 1]);
    
    
    up3  = reshape(U3( i-L:i-1, : )', [L*nu, 1]);
    y3 = reshape(Y3(i-L:i, :)', [(L+1)*nu, 1]);
    
    up4  = reshape(U4( i-L:i-1, : )', [L*nu, 1]);
    y4 = reshape(Y4(i-L:i, :)', [(L+1)*nu, 1]);
    
    up0 = reshape(U_leader( i-L:i-1, : )', [L*nu, 1]);
    
    % control signal k-1
    uam = -1e6*ones( size(um,1) + size(u3m,1) + size(u2m,1) + size(u4m,1), 1);
    % Iteration number
    iter2 = 1;
    
    while norm([um;u2m;u3m;u4m] - uam) > 1e-1 && iter2 <= 10000
        
        uam = [um;u2m;u3m;u4m];
        iter2 = iter2 +1;
        
        gammam = g1*kron( eye(L), B_l)*(up0);
        gamma2m = cx2*kron( eye(L), B3o )*(up3 + u3m(end-nu*L+1:end , :)) + ...
                  cx21*kron( eye(L), B1o )*(up1 + um(end-nu*L+1:end , :));
        gamma3m = cx3*kron( eye(L), B1o )*(up1 + um(end-nu*L+1:end , :));
        gamma4m = cx4*kron( eye(L), B2o )*(up2 + u2m(end-nu*L+1:end , :));
        
        
        Fm1 = [up1'*Be_1'*W_hat*Ae_1 + gammam'*Pe_1'*W_hat*Ae_1 - y1'*W_hat*Ae_1  ...
            up1'*Be_1'*W_hat*Be_1 + gammam'*Pe_1'*W_hat*Be_1 - y1'*W_hat*Be_1];
        
        Fm2 = [up2'*Be_2'*W_hat*Ae_2+gamma2m'*Pe_2'*W_hat*Ae_2-y2'*W_hat*Ae_2  ...
            up2'*Be_2'*W_hat*Be_2+gamma2m'*Pe_2'*W_hat*Be_2-y2'*W_hat*Be_2];
        
        Fm3 = [up3'*Be_3'*W_hat*Ae_3+gamma3m'*Pe_3'*W_hat*Ae_3-y3'*W_hat*Ae_3  ...
            up3'*Be_3'*W_hat*Be_3+gamma3m'*Pe_3'*W_hat*Be_3-y3'*W_hat*Be_3];   
        
        Fm4 = [up4'*Be_4'*W_hat*Ae_4+gamma4m'*Pe_4'*W_hat*Ae_4-y4'*W_hat*Ae_4  ...
            up4'*Be_4'*W_hat*Be_4+gamma4m'*Pe_4'*W_hat*Be_4-y4'*W_hat*Be_4];   
        
        
        
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
        ind = nu*(j-1)+1:nu*j;%index of up_
        ind2 = nx+nu*(j-1)+1:nx+nu*j; %index of um_
        out = A1*out - B1*(up1(ind,:) + um(ind2,:)) + g1*B_l * up0(ind,:);
        out3 = A3*out3 - B3*(up3(ind,:) + u3m(ind2,:)) + cx3*B1o*(up1(ind,:) + um(ind2,:));
        out2 = A2*out2 - B2*(up2(ind,:) + u2m(ind2,:)) + cx2*B3o*(up3(ind,:) + u3m(ind2,:)) ...
                                                   + cx21*B1o*(up1(ind,:) + um(ind2,:));
        out4 = A4*out4 - B4*(up4(ind) + u4m(ind2,:)) + cx4*B2o*(up2(ind,:) + u2m(ind2,:));
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
%       gamma = pi
%       Gg = P
        gamma = g1*kron( eye(T), B_l)*(u_leader(i+1:i+T));
        gamma3 = cx3*kron( eye(T), B1o )*(u+dd1);
        gamma2 = cx2*kron( eye(T), B3o )*(u3+dd3) + cx21*kron( eye(T), B1o )*(u+dd1);
        gamma4 = cx4*kron( eye(T), B2o )*(u2+dd2);
        
%       Equation 11
        F1 = e'*A_'*Q_hat*B_1 + gamma'*P_'*Q_hat*B_1 + dd1'*B_1'*Q_hat*B_1;
        F3 = e3'*A_'*Q_hat*B_3 + gamma3'*P_'*Q_hat*B_3 + dd3'*B_3'*Q_hat*B_3;
        F2 = e2'*A_'*Q_hat*B_2 + gamma2'*P_'*Q_hat*B_2 + dd2'*B_2'*Q_hat*B_2;
        F4 = e4'*A_'*Q_hat*B_4 + gamma4'*P_'*Q_hat*B_4 + dd4'*B_4'*Q_hat*B_4;
        
%       PROCESO DE OPTIMIZACION ADMM
        u = ( H + theta*( AA'*AA ) )  \ ( AA'*( theta*s + Lambda ) - F1' );
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
    di = md*randn(nu,1);
    D1( i, : ) = di;
    x1 = A1*x1 + B1o*(u(1:nu,1) + di);
    
    % State agent 1
    X1( i+1 , : ) = x1;
    
    
    % System agent 2
    di = md*randn(nu,1);
    D2( i, : ) = di;
    x2 = A2*x2 + B2o*(u2(1:nu,1) + di);
    % State agent 2
    X2( i+1 , : ) = x2;
    
    
    % System agent 3
    di = md*randn(nu,1);
    D3( i, : ) = di;
    x3 = A3*x3 + B3o*(u3(1,1) + di);
    % State agent 3
    X3( i+1 , : ) = x3;
    
    
    
    
     % System agent 4
    di = md*randn(nu,1);
    D4( i, : ) = di;
    x4 = A4*x4 + B4o*(u4(1,1) + di);
    
    % State agent 4
    X4( i+1 , : ) = x4;
    
    
    
    
    

    e = g1*(x - x1);
    Y1(i+1,:) = C1*e;
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
    
    nloop = nloop + 1
end
toc

%--------------------------------------------------------------------------
%% Ploting
k = 0:500;
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
title('X- position')


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
title('Y- Position')
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

 save('myFile2.mat')
