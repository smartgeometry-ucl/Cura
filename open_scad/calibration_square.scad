

width=100;
border_width=0.4;
border_height=0.4;


module calibration_square()  {
translate([0,0,border_height/2])
difference() {
		cube(size=[width+border_width*2,width+border_width*2,border_height], center=true);	
		cube(size=[width-border_width*2,width-border_width*2,border_height*2], center=true);
}
}

calibration_square();