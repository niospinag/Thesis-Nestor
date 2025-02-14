function [Vref, Zref] = Gen_Refer(Vdes, vel, pos, Zdes,  zel, vh)
d_sec = 25;
nv = size(Vdes,1);
vehicles = 1:nv;
d_pos = pos-pos(vh);
d_z = zel - zel(vh);
L_max = 5;
Vref = Vdes;
Zref = Zdes;
%% sensing the environment
Up_oc = 0;
Down_oc = 0;
Front_oc = 0;
Back_oc = 0;
Vfront = nan;
Vback = nan;
neighbor = vehicles(vehicles~=vh);
for i = neighbor
    if(d_z(i)==1 && abs(d_pos(i)) < d_sec) %%if vh is over and close
        Up_oc = 1;
    elseif (d_z(i)==-1 && abs(d_pos(i)) < d_sec)
        Down_oc = 1;
    end

    if (d_z(i)==0 && abs(d_pos(i)) < d_sec)
        if( d_pos(i) > 0 )
            Front_oc = 1;
            Vfront = vel(i);
        else
            Back_oc = 1;
            Vback = vel(i);
        end
    end
end


%% control action

if (Front_oc)
    if(Vfront<vel(vh))
        Vref(vh) = Vfront;
        %Try to evade the obstacle
        if(Up_oc == 0 && zel(vh) < L_max)
            Zref(vh)= zel(vh)+1;
        elseif Down_oc == 0 && zel(vh) > 1
            Zref(vh)= zel(vh)-1;
        else
            Zref(vh)= zel(vh);
        end
    else
        Vref(vh) = Vdes(vh)
        Zref(vh)=ch_lane(Zdes(vh),zel(vh),Up_oc,Down_oc) 
    end
    
% elseif (Back_oc)
%     if(Vback>vel(vh))
%         Vref(vh) = Vback;
%         %Try to evade the obstacle
%         if Down_oc == 0 && zel(vh) > 1
%             Zref(vh)= zel(vh)-1;
%         elseif(Up_oc == 0 && zel(vh) < L_max)
%             Zref(vh)= zel(vh)+1;
%         else
%             Zref(vh)= zel(vh);
%         end
%         
%     else
%         Vref(vh) = Vdes(vh);
%         Zref(vh)=ch_lane(Zdes(vh),zel(vh),Up_oc,Down_oc) ;
%     end
else
    Vref(vh) = Vdes(vh);
    Zref(vh)=ch_lane(Zdes(vh),zel(vh),Up_oc,Down_oc);   
end
    
end


