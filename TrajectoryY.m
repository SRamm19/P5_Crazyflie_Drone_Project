function [y,dy,ddy] =TrajectoryX(start_pos,via_point,end_point,Hz)
[Cv,Rv]=size(via_point);
Movemementvector=[start_pos;via_point;end_point];
[col,row]=size(Movemementvector);

for i=0:(Cv+1)
    if i==0
        a01=start_pos(1,2);
        a11=0;
        dx1=(via_point(1,2)-start_pos(1,2))/((via_point(1,4)-start_pos(1,4)));
        tf1=(via_point(1,4)-start_pos(1,4));
        % 
        a21=(3/(tf1^2))*(via_point(1,2)-start_pos(1,2))-(1/(tf1))*dx1;
        a31=-(2/(tf1^3))*(via_point(1,2)-start_pos(1,2))+(1/tf1^2)*(dx1);

        t=0:1/Hz:via_point(1,4);

        y=a01+a11*t+a21*t.^2+a31*t.^3;
        dy=a11+2*a21*t+3*a31*t.^2;
        ddy=2*a21+6*a31*t;

        
        
        
    end
    if 0<i

        if i<Cv  
        a02=via_point(i,2);
        a12=dx1;
        dx2=(via_point(i+1,2)-via_point(i,2))/(via_point(i+1,4)-via_point(i,4));
        tf2=(via_point(i+1,4)-via_point(i,4));

        a22=(3/(tf2^2))*(via_point(i+1,2)-via_point(i,2)) -2/((tf2))*dx1-(1/(tf2))*dx2;
        a32=-(2/(tf2^3))*(via_point(i+1,2)-via_point(i,2))+(1/tf2^2)*(dx1+dx2);
        
        
        t=1/Hz:1/Hz:(via_point(i+1,4)-via_point(i,4));

        y=[y(1,1:length(y)),a02+a12*t+a22*t.^2+a32*t.^3];
       dy=[dy(1,1:length(ddy)),a11+2*a22*t+3*a32*t.^2];
       ddy=[ddy(1,1:length(ddy)),2*a22+6*a32*t];

        dx1=(via_point(i+1,2)-via_point(1,2))/(via_point(i+1,4)-via_point(i,4));
                            
            
        end
    end
    
    if i==Cv
        %% afslutning
a03=via_point(i,2);
a13=dx1;
tf3=(end_point(1,4)-via_point(i,4));

a23=(3/(tf3^2))*(end_point(1,2)-via_point(i,2))-2/((tf3))*dx1;
a33=-(2/(tf3^3))*(end_point(1,2)-via_point(i,2))+(1/tf3^2)*(dx1);
t=1/Hz:1/Hz:end_point(1,4)-via_point(i,4);
y=[y(1,1:length(y)),a03+a13*t+a23*t.^2+a33*t.^3];
dy=[dy(1,1:length(dy)),a11+2*a23*t+3*a33*t.^2];
ddy=[ddy(1,1:length(ddy)),2*a23+6*a33*t];

       %       dx=[dx(1,1:length(dx)),t*0];

      
    end  
end
end
