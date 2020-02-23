//intersection() {
   //surface(file = "example010.dat", center = true, convexity = 5);
   //rotate(45, [0, 0, 1]) surface(file = "example010.dat", center = true, convexity = 5); 
 //}

thick = 1;
imsize = 200;



//print();
//sheet(0);
//combine();
union() {
  cube(size=[imsize+50,imsize+50,thick],center=true); 
  translate([0, 0, 5]) scale([1, 1, 0.2]) surface(file = "200crop.png", center=true);
}

module sheet(off = 0) {
   cube(size=[imsize+50,imsize+50,thick],center=true);
         
}


module print() {
    scale([1, 1, 0.2]) surface(file = "200crop.png",center=true);
    
}


