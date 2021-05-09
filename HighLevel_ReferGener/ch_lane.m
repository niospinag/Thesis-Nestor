% check if vh can change of line
function [Zr]=ch_lane(Zd,z,Up_oc,Down_oc) 
% intention of go up
if Zd > z
    if(Up_oc)
        Zr= z;
    else
        Zr = Zd;
    end
%  Intention of go down
elseif Zd < z
    if(Down_oc)
        Zr = z;
    else
        Zr = Zd;
    end
else
    Zr = Zd;
end

end