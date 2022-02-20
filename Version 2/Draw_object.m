function Draw_object(vhist,zhist,vphist,zphist,dhist,T,delay_time)


N = size(vphist,2)-1;%Horizonte de prediccion
V = size(vphist,3);%numero de vehiculos


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;
% V
%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
an = 10; alt=0.5; % parametros carroceria

rob_diam=1.5;
r = rob_diam/2;  % robot radius
ang=0:0.005:2*pi;
x_circle=an*cos(ang);
y_circle=alt*sin(ang);


figure(500)%create a window named "500"
% Animate the robot motion
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0.2 1 0.5]);%tamaño del grafico en pantalla


xp=zeros(1,size(vphist,3),V);

% %  initial conditions
% for n=2:V
%     xp(1,1,n)=dhist(n-1);
% end



% ---------vehiculo n--------
for n = 1:size(vphist,3) %numer of vehicles
    for j = 1:size(vphist,1)
        xp(j,1,n)=dhist(n,j);
        for i = 1:(size(vphist,2)-1)
            xp(j,i+1,n)=xp(j,i,n)+T*vphist(j,i,n);
        end
    end
end

x=zeros(V,size(vhist,2));

% for n=2:V
%     x(1,1,n)=dhist(n-1);
% end


x = dhist;
y=zhist;
%  dibujo dinamico
for k = 1:size(vhist,2)

%     for n=1:V
%         x(1,k+1,n)=x(1,k,n)+T*vhist(n,k);
%     end
    
    %-------------Plot any car----------------

    plot_car(an,alt,x(1,k),y(1,k),0,'b')%Plot the car
    plot_car(an,alt,x(2,k),y(2,k),0,'r')%Plot the car
    plot_car(an,alt,x(3,k),y(3,k),0,'g')%Plot the car
    plot_car(an,alt,x(4,k),y(4,k),0,'c')%Plot the car
    plot_car(an,alt,x(5,k),y(5,k),0,'m')%Plot the car
    plot_car(an,alt,x(6,k),y(6,k),0,'y')%Plot the car
    %

    hold on;




    %-----------Plot trajectories------------
    %------------agente 1------------
    plot(x(1,k),y(2,k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,1),zphist(k,:,1),'b--*')
    end
    plot(x(1,k)+x_circle,y(1,k)+y_circle,'--b')% plot robot circle
    %------------agente 2------------
    plot(x(2,k),y(2,k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,2),zphist(k,:,2),'r--*')
    end
    plot(x(2,k)+x_circle,y(2,k)+y_circle,'--r')% plot robot circle
    %------------agente 3------------
    plot(x(3,k),y(3,k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,3),zphist(k,:,3),'g--*')
    end
    plot(x(3,k)+x_circle,y(3,k)+y_circle,'--g')% plot robot circle

    %------------agente 4------------
    plot(x(4,k),y(4,k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,4),zphist(k,:,4),'c--*')
    end
    plot(x(4,k)+x_circle,y(4,k)+y_circle,'--c')% plot robot circle

    %------------agente 5------------
    plot(x(5,k),y(5,k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,5),zphist(k,:,5),'m--*')
    end
    plot(x(5,k)+x_circle,y(5,k)+y_circle,'--m')% plot robot circle

    %------------agente 6------------
    plot(x(6,k),y(6,k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,6),zphist(k,:,6),'y--*')
    end
    plot(x(6,k)+x_circle,y(6,k)+y_circle,'--y')% plot robot circle
    %

    filename = 'MPC.gif';% <---------------------------------------------------------------------------------------------------------------


    hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-150 300 -0.2 7])%descripcion de los ejes
    %   axis([x_min x_max y_min y_max])

    pause(delay_time)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame


    %---------------------make gift--------------------------
    frame = getframe(figure(500));
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
    if k == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
  else
      imwrite(imind,cm,filename,'gif','WriteMode','append');
  end

  %

end
close(gcf)