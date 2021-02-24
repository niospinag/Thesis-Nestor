function u0 =random_u(u0, u_max)
    switch_prob = 0.5;
    u_next = (0.5-rand(2, 1))*u_max;  % New candidate value.
    switch1 = rand() >= (1-switch_prob);  % switching? 0 or 1.
    u0 = (1-switch1)*u0 + switch1*u_next;  % Old or new value.
end