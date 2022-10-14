function plotCoord_syst(scale,x_tcp,y_tcp,z_tcp,eulerZYX,parent_Ax)
        disp('  function plotCoord_syst')

       %Target Axis
       parentAx = parent_Ax;

       %Origin of Coord System
       x_coord = x_tcp;
       y_coord = y_tcp;                                     
       z_coord = z_tcp;

       %Length of Vectors (when diffrent scales are needed)
       X_lim = xlim(parentAx);
       Y_lim = ylim(parentAx);
       Z_lim = zlim(parentAx);

       xlength =(X_lim(1,2)-X_lim(1,1))*scale;
       ylength = (Y_lim(1,2) -Y_lim(1,1))*scale;
       zlength = (Z_lim(1,2)-Z_lim(1,1))*scale;

        hold (parentAx,"on");

        %x-axis                                    
        text(parentAx,x_coord+xlength,y_coord,z_coord,'    X','Color','red','FontSize',16);
        plot_Vector(xlength,x_coord,y_coord,z_coord,0,90+eulerZYX(2),0,'r',parentAx);

        %y-axis                                  
        text(parentAx,x_coord,y_coord+ylength,z_coord,'    Y','Color','green','FontSize',16);
        plot_Vector(ylength,x_coord,y_coord,z_coord,-90+eulerZYX(1),0,0,'g',parentAx);

        %z-axis                                   
        text(parentAx,x_coord,y_coord,z_coord+zlength,'    Z','Color','blue','FontSize',16);
        plot_Vector(zlength,x_coord,y_coord,z_coord,0,0,0,'blue',parentAx);   


        function plot_Vector(~,length,xt,yt,zt,alpha,beta,gamma,color,parent)
            disp('  function: plot_Vector') 
            %number of triangels to aproximate cone shape
            n = 4;
            %create values for calculating shape of circle
            a = linspace(0,2*pi,n+1);          
            %adds an additional element to close the circle
            a(n+2) = 0;
    
            %Variables that define the width of the circles
            width_Cone = 0.12*length;
            width_Line = width_Cone*0.35;
    
            %calculate x & y coord. of the circle
            x = cos(a)*width_Cone;  
            y = sin(a)*width_Cone;
            z = ones(1,n+2)*length;
    
            %calculate x & y coord. of the line
            x_line = cos(a)*width_Line;
            y_line = sin(a)*width_Line;
            z_line_end = ones(1,n+2)*length;
            z_line_start = zeros(1,n+2); 
    
            %tip of cone
            x3 = 0;
            y3 = 0;
            z3 = length*(1+0.25);
                   
            %create rotation matrix
            R_x = [1 0 0;...
                0 cosd(alpha) -sind(alpha);...
                0 sind(alpha) cosd(alpha)];
    
            R_y = [cosd(beta) 0 sind(beta);...
                0 1 0;...
                -sind(beta) 0 cosd(beta)];
    
            R_z = [cosd(gamma) -sind(gamma) 0;...
                sind(gamma) cosd(gamma) 0;...
                0 0 1];
    
            %calculate rot Matrix
            Rot = R_x*R_y*R_z;
    
            %transpose row vectors for rotation
            x = transpose(x);
            y = transpose(y);
            z = transpose(z);
            x_line = transpose(x_line);
            y_line = transpose(y_line);
            z_line_end = transpose(z_line_end);
            z_line_start = transpose(z_line_start);
            
            %Creates vektors to be rotated in a loop
            xyz =[x,y,z];
            xyz_line_end = [x_line,y_line,z_line_end];
            xyz_line_start = [x_line,y_line,z_line_start];
    
            %rotates xyz & xyz_line_end & xyz_line_start
            xyz_line_end = transpose(Rot*transpose(xyz_line_end));
            xyz_line_start = transpose(Rot*transpose(xyz_line_start));
            xyz = transpose(Rot*transpose(xyz));
                     
            %rotates tip of cone
            xyz3 = [x3;y3;z3];
            xyz3 = transpose(Rot*xyz3);
    
            %move circle & lower & upper circle  of the line/cylinder & tip of cone in x,y,z
            x = xyz(:,1)+xt;
            y = xyz(:,2)+yt;
            z = xyz(:,3)+zt;           
            x_line_start = xyz_line_start(:,1)+xt;
            y_line_start = xyz_line_start(:,2)+yt;
            z_line_start = xyz_line_start(:,3)+zt;           
            x_line_end = xyz_line_end(:,1)+xt;
            y_line_end = xyz_line_end(:,2)+yt;
            z_line_end = xyz_line_end(:,3)+zt;                                                      
            x3 = xyz3(1,1)+xt;
            y3 = xyz3(1,2)+yt;
            z3 = xyz3(1,3)+zt;
    
            % plots circels        
            fill3(parent,x,y,z,color);
            fill3(parent,x_line,y_line,z_line_start,color);
            fill3(parent,x_line,y_line,z_line_end,color);
           
            %plots cone/cylinder
            for m = 2:(n+1)                                                            
                fill3(parent,[x(m-1,1),x(m,1),x3],[y(m-1,1),y(m,1),y3],[z(m-1,1),z(m,1),z3],color)%,'LineStyle','none' 
                fill3(parent,[x_line_start(m-1,1),x_line_start(m,1),x_line_end(m,1),x_line_end(m-1,1)], ...%4 elements of x form an square
                    [y_line_start(m-1,1),y_line_start(m,1),y_line_end(m,1),y_line_end(m-1,1)], ...
                    [z_line_start(m-1,1),z_line_start(m,1),z_line_end(m,1),z_line_end(m-1,1)],color)%,'LineStyle','none'              
            end            
    end
end
      
      