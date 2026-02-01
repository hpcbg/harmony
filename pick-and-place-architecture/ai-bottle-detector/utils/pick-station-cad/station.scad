module bolt_holes() {
    // top front cover
    translate([0,53.5,156.5]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);

    // top back cover
    translate([0,147.5,156.5]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);

    // front cover
    translate([0,44.5,38.5]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);
    translate([0,44.5,156.5]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);

    // back cover
    translate([0,156.5,3.5]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);
    translate([0,156.5,156.5]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);

    // bottom rail
    translate([0,153.5,12.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);
    translate([0,88.5,7.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);

    // middle bottom rail
    translate([0,46.5,49.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);
    translate([0,111.5,44.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);

    // middle top rail
    translate([0,153.5,93.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);
    translate([0,88.5,88.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);

    // top rail
    translate([0,46.5,136.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);
    translate([0,111.5,131.7]) rotate([0,90,0]) cylinder(100, 1.2, 1.2, true);


    // camera holder
    translate([-13,0,147]) rotate([0,90,90]) cylinder(100, 1.2, 1.2, true);
    translate([13,0,147]) rotate([0,90,90]) cylinder(100, 1.2, 1.2, true);
    translate([-15,0,210]) rotate([0,90,90]) cylinder(100, 1.2, 1.2, true);
    translate([15,0,210]) rotate([0,90,90]) cylinder(100, 1.2, 1.2, true);
}

module left_cover() {
    difference() {
        union() {
            translate([76/2,100,80]) rotate([0,90,0]) cube([160,120,1], true);
            translate([76/2-0.5,-20,0]) cube([1,65,40]);
        }
        bolt_holes();
    }
}

module right_cover() {
    difference() {
        union() {
            translate([-76/2,100,80]) rotate([0,90,0]) cube([160,120,1], true);
            translate([-76/2-0.5,-20,0]) cube([1,65,40]);
        }
        bolt_holes();
    }
}

module top_front_cover() {
    difference() {
        union() {
            translate([0,60,160.5]) cube([77,40,1], true);
            translate([-75/2,50,153]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module top_back_cover() {
    difference() {
        union() {
            translate([0,140,160.5]) cube([77,40,1], true);
            translate([-75/2,144,153]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module front_cover() {
    difference() {
        union() {
            translate([0,41,(160 - 35)/2 + 35]) rotate([90,0,0]) cube([75,160-35,2], true);
            translate([-75/2,41,35]) cube([75,7,7]);
            translate([-75/2,41,153]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module back_cover() {
    difference() {
        union() {
            translate([0,160-1,160/2]) rotate([90,0,0]) cube([75,160,2], true);
            translate([-75/2,153,0]) cube([75,7,7]);
            translate([-75/2,153,153]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module bottom_rail() {
    difference() {
        union() {
            translate([-75/2,-20,1]) rotate([5,0,0]) cube([75,179,1]);
            translate([-75/2,150,9.2]) rotate([5,0,0]) cube([75,7,7]);
            translate([-75/2,85,4.2]) rotate([5,0,0]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module middle_bottom_rail() {
    difference() {
        union() {
            translate([-75/2,42,53]) rotate([-5,0,0]) cube([75,85,1]);
            translate([-75/2,43,46.2]) rotate([-5,0,0]) cube([75,7,7]);
            translate([-75/2,108,41.2]) rotate([-5,0,0]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module middle_top_rail() {
    difference() {
        union() {
            translate([-75/2,73,90]) rotate([5,0,0]) cube([75,85,1]);
            translate([-75/2,150,90.2]) rotate([5,0,0]) cube([75,7,7]);
            translate([-75/2,85,85.2]) rotate([5,0,0]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module top_rail() {
    difference() {
        union() {
            translate([-75/2,42,140]) rotate([-5,0,0]) cube([75,85,1]);
            translate([-75/2,43,133.2]) rotate([-5,0,0]) cube([75,7,7]);
            translate([-75/2,108,128.2]) rotate([-5,0,0]) cube([75,7,7]);
        }
        bolt_holes();
    }
}

module camera_holder() {
    difference() {
        translate([-25,40,140]) rotate([90,0,0]) cube([50,80,3]);
        bolt_holes();
    }
}

module working_area() {
    translate([-90,-180,0]) cube([180, 20, 15]);
    translate([-90,-180,0]) cube([20, 180, 15]);
    translate([90-20,-180,0]) cube([20, 180, 15]);
    translate([-90,-20,0]) cube([51.5, 20, 15]);
    translate([90-51.5,-20,0]) cube([51.5, 20, 15]);
    translate([-90,-26,0]) cube([26,26,15]);
    translate([90-26,-180,0]) cube([26,26,15]);
    translate([90-26,-26,0]) cube([26,26,15]);
    translate([-90,-180,0]) cube([26,26,15]);
}

module bottle_stand() {
    difference() {
        union() {
            cube([30,30,5], true);
            translate([0,0,10]) cube([10,10,20], true);
        }
        $fn=180; cylinder(100,2.5,2.5, true);
    }
}

module simple_rail() {
    translate([-76/2,140,35]) rotate([0,90,0]) cube([70,40,1], true);
    translate([-76/2-0.5,-20,0]) cube([1,140,40]);
    translate([76/2,140,35]) rotate([0,90,0]) cube([70,40,1], true);
    translate([76/2-0.5,-20,0]) cube([1,140,40]);
    difference() {
        translate([-75/2,-20,-20]) rotate([5,0,0]) cube([75,184,20]);
        union() {
            translate([-75/2,-30,-20])  cube([75,200,20]);
            translate([-75/2,160,0])  cube([75,200,20]);
        }
    }
    difference() {
        translate([-75/2,120,60]) cube([75,40,10]);
        union() {
            translate([25,140,0]) cylinder(100,1.5,1.5);
            translate([-25,140,0]) cylinder(100,1.5,1.5);
        }
    }
}
module camera_stand_base() {
    difference() {
        union() {
            translate([-75/2,120,70]) cube([75,40,5]);
            translate([-15,125,75]) cube([30,30,185]);
        }
        union() {
            translate([25,140,0]) cylinder(100,1.5,1.5);
            translate([-25,140,0]) cylinder(100,1.5,1.5);
            translate([0,140,250]) cylinder(100,1.5,1.5);
        }
    }
}
module camera_angle() {
    difference() {
        union() {
            translate([-15,125,260]) cube([30,30,20]);
            //translate([-15,155,260]) cube([30,5,2]);
        }
        union() {
            translate([-14,110,278]) rotate([-20,0,0]) cube([28,55,40]);
            translate([0,140,250]) cylinder(100,1.5,1.5);
            translate([0,140,263]) cylinder(100,5,5);
        }
    }
    translate([-15,154,261]) cube([5,1,8]);
    translate([10,154,261]) cube([5,1,8]);
}

module camera_tripod_stand() {
    difference() {
        translate([-15,125,190]) cube([30,30,70]);
        translate([0,140,190]) cylinder(10,2,2);
    }
    camera_angle();
}

//simple_rail();
//camera_stand_base();
//camera_angle();
camera_tripod_stand();
//translate([-14,100,271]) rotate([-20,0,0]) cube([28,55,40]);
// left_cover();
// right_cover();
// top_front_cover();
// top_back_cover();
// front_cover();
// back_cover();
// top_rail();
// middle_top_rail();
// middle_bottom_rail();
// bottom_rail();
// working_area();
// camera_holder();
// bottle_stand();