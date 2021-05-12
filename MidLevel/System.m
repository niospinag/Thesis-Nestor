classdef    System
% control problem properties      
    properties
        name = nan;
        
        Ts = nan;% delta time
        A = nan;
        B = nan
        Bo = nan;
        C = nan;
        
%         A, b disturbances
        Ad = nan;
        bd = nan;
        


        % Constraints
        A_err = nan;
        b_err = nan;

        Au = nan;
        bu = nan;
        
        % States number
        nx = nan;
        % Control signals number
        nu = nan;
        % Disturbance number
        nd = nan;
        % Output number
        nc = nan;
        
%         Backward finite horizon
        L = nan;%Horizonte del MHE
        T = nan;%horizonte del mpc
        
        % MHE weights
        W = nan;
        Wd = nan;
        
        W_hat = nan;
        Wd_hat = nan;
        
        
        % MHE Matrices
        Ae_ = nan; %Gxc
        Be_ = nan; %Guc
        Pe_ = nan; %Ggc
        
%         MPC MATRICES
        A_ = nan; %Gx
        B_ = nan; %Gu
        P_ = nan; %Gg
        
        
        % MPC Weights
        Q = nan;
        R = nan;
        
        Q_hat = nan;
        R_hat = nan;
        
%         MPC matrizes
        H = nan;
        F = nan;
        s = nan
%       
% constraints augmented
        Au_hat = nan;
        bu_hat = nan;
        
        Ae_hat_m = nan;
        be_hat_m = nan;
        
        Ad_hat_m = nan;
        bd_hat_m = nan;
        
        AA_m = nan;
        bU_m = nan;



% initial conditions
%         estados
        x = nan;
        
        %History vectors
        X = nan;
        Ee = nan;
        E = nan;
        Y = nan;
        U = nan;
        D = nan;
        
        
        
        
        
        
        
        
        
        
%         ######################################################################
                     
    end
    
    
    methods
        function obj = System(name);
            obj.name= name;
            fprintf('Object created :\n', name);
        end
               
%%  Construction A_, B_, P_
        function [obj,B_] = MPC_Arrays(obj)
        %  Equation (8)
            T = obj.T;
            L = obj.L;
            nx = obj.nx;
            nu = obj.nu;
            nc = obj.nc;
            A_ = zeros( nx*T, nx); %Gx
            B_ = zeros( nx*T, nu*T); %Gu
            P_ = zeros( nx*T, nx*T); %Gg
            for i = 1:T
                A_( nx*(i-1)+1:nx*i  ,:) = obj.A^i;
                for j = 1:T-i+1
                    B_( nx*(j+i-2)+1:nx*(j+i-1) , nu*(j-1)+1:nu*j ) = -(obj.A^(i-1))*obj.B;
                    P_( nx*(j+i-2)+1:nx*(j+i-1) , nx*(j-1)+1:nx*j ) = (obj.A^(i-1));
                end
            end
            Q_hat = kron( eye(T), obj.Q );
            R_hat = kron( eye(T), obj.R );

            H = B_'*Q_hat*B_ + R_hat;
            H = ( H+H' )/2;

            obj.A_ = A_; %Gx
            obj.B_ = B_; %Gu
            obj.P_ = P_; %Gg
            obj.H = H;
            obj.Q_hat = Q_hat;
            obj.R_hat = R_hat;
    %         ______________________________________________________
            % Costruction A_e, B_e, P_e
            % Equation (15)
        end
        function [obj,Ae_, Be_, Pe_, Hm] = MHE_Arrays(obj)
            T = obj.T;
            L = obj.L;
            nx = obj.nx;
            nu = obj.nu;
            nc = obj.nc;
            Ae_ = zeros( nc*(L+1), nx); %Gxc1
            Ae_( 1:nc, :) = obj.C;
            Be_ = zeros( nc*(L+1), nu*L); %Guc1
            Pe_ = zeros( nc*(L+1), nx); %Ggc1

            for i = 2:L+1
                Ae_( nc*(i-1)+1:nc*i , : ) = obj.C*(obj.A^(i-1));

                for j = 1:L-i+2
                    Be_( nc*(j+i-2)+1:nc*(j+i-1) , nu*(j-1)+1:nu*j ) = -(obj.C*(obj.A^(i-2)))*obj.B;
                    Pe_( nc*(j+i-2)+1:nc*(j+i-1) , nx*(j-1)+1:nx*j ) = (obj.C*(obj.A^(i-2))); 
                end
            end

            obj.Ae_ = Ae_;
            obj.Be_ = Be_;
            obj.Pe_ = Pe_;

            W_hat = kron(eye((L+1)),obj.W);
            Wd_hat = kron( eye(size(Be_,2)) , obj.Wd );

            Hm = [Ae_'*W_hat*Ae_  Ae_'*W_hat*Be_;
                  Be_'*W_hat*Ae_  Be_'*W_hat*Be_+Wd_hat];
            Hm =( Hm+Hm')/2;

            obj.W_hat = W_hat;
            obj.Wd_hat = Wd_hat ;

            fprintf('Arrays Created!!! \n', obj.T);
        end


        function obj = Const_Augment(obj)
            T = obj.T;
            L = obj.L;
            obj.Au_hat = kron( eye(T), obj.Au );
            obj.bu_hat = kron( ones(T,1), obj.bu );


            % MHE

            Ae_hat_m = kron( eye(1), obj.A_err );
            be_hat_m = kron( ones(1,1), obj.b_err );

            Ad_hat_m = kron( eye(L), obj.Ad );
            bd_hat_m = kron( ones(L,1), obj.bd );

            AA_m = [Ae_hat_m zeros( size( Ae_hat_m , 1 ) , size( Ad_hat_m , 2 ) );
                  zeros( size( Ad_hat_m , 1) , size( Ae_hat_m , 2 ) ) Ad_hat_m];
            
            obj.AA_m = AA_m;
              
            bU_m = [be_hat_m; bd_hat_m];
            obj.bU_m = bU_m;
      
        end
        
        function obj = History_Vectors(obj,x, e, t)
            obj.X = zeros( length(t)+1 , obj.nx );
            obj.X( 1 , : ) = x;
            obj.x = x;
            
            obj.E = zeros( length(t)+1 , obj.nx );
            obj.Ee = zeros( length(t)+1 , obj.nx );
            obj.E( 1 , : ) = e;
            obj.Y = zeros( length(t), obj.nc);
            obj.Y( 1, :) = obj.C*e;
            obj.U = zeros( length(t) , obj.nu );
            obj.D = zeros( length(t) , obj.nd );
        end
        
        
        
        function [obj, x]=make_step(obj, x, u, i)
            x  = obj.A*x + obj.Bo*u;
            obj.X ( i+1, : ) = x;
            obj.x = x;
            
        end
        
        
        
        
        
        
        
        
        
        
        
        
        
        
end     
        
    
    
    
    
    
    
    
end