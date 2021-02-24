classdef System
   
% control problem properties      
    properties
             
        n_x = 3;
        n_u = 2;
        n_y = 3;
        
        x0 = zeros(3,1);
        
        x_ = [];
        u_ = [];
        y_ = [];

        dt = 1;
        t_now = 0;
        time_ = [];
       
    end
    
        
    
    methods
        
               
%% add observations
% update the previous history 
% for now, we will pass in the entire previous history
% then we can easily figure out what time we're at by checking the length 
        function  [obj, y] = make_step(obj, u)
            obj.x_ = [obj.x_ obj.x0];
            obj.u_ = [obj.u_ u];
            x_v = obj.x0(1);
            y_v = obj.x0(2);
            t_v = obj.x0(3);
            psi_v = obj.x0(4);
            v_v = obj.x0(5);
            
            
            y = zeros(5,1);
            
        
            
            
            y(1) = v_v*sin(t_v)*t_v - v_v*sin() 
%             obj.dt*cos(obj.x0(3))*u(1);
            y(2) = obj.x0(2) + obj.dt*sin(obj.x0(3))*u(1);
            y(3) = obj.x0(3) + obj.dt*u(2);
            

            obj.y_ = [obj.y_ y];
            obj.time_ = [obj.time_ obj.t_now];
            obj.x0 = y ;
            obj.t_now = obj.t_now + obj.dt;

        end


        
%% 
        function obj = reset(obj, x0)
            obj.x0 = x0;
            obj.x_ = [];
            obj.u_ = [];
            obj.y_ = [];
            obj.time_ = [];
            obj.t_now = 0;
           
    
        end
    end
    
    
end