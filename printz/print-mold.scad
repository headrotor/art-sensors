//intersection() {
   //surface(file = "example010.dat", center = true, convexity = 5);
   //rotate(45, [0, 0, 1]) surface(file = "example010.dat", center = true, convexity = 5); 
 //}

thick = 30;
imsize = 195;
pad=400;



//print();
//sheet(0);
//combine();
union() {
   difference() {
     translate([0, 0, thick/2]) cube(size=[imsize+pad,imsize+pad,thick],center=true);
    translate([0, 0, thick/2]) cube(size=[imsize,imsize,1.1*thick],center=true);
   } 
  translate([0, 0, 17]) scale([1, 1, 0.2]) surface(file = "200crop.png", center=true, invert=true);
}

module sheet(off = 0) {
   cube(size=[imsize+50,imsize+50,thick],center=true);
         
}


module print() {
    scale([1, 1, 0.2]) surface(file = "200crop.png",center=true);
    
}


