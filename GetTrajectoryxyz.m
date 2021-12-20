function Path = GetTrajectoryxyz(start_pos,via_point,end_point,hz)



 [X,DX,DDX]=TrajectoryX(start_pos,via_point,end_point,hz);
 [Y,DY,DDY]=TrajectoryY(start_pos,via_point,end_point,hz);
 [Z,DZ,DDZ]=TrajectoryZ(start_pos,via_point,end_point,hz);
 
 Thrust=((DDZ*0.041-0.041*-9.82)*149776)-4263.8;
 ROLL=rad2deg(DDY/9.82);
 PITCH=rad2deg(DDX/-9.82);
 
Path=[X;DX;DDZ;Y;DY;DDY;Z;DZ;DDZ;ROLL;PITCH;Thrust];


end

