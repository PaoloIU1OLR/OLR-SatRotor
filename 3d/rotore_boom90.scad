// ============================================================================
// ROTORE PORTATILE AZ-EL PER ANTENNA SATELLITARE (TIPO ARROW - BOOM 20.5x20.5)
// Servomotori: 2x DSSERVO DS3230 PRO (30kg)
// Versione: Morsetto boom esteso a 90mm per massima presa e stabilità
// ============================================================================

/* [Selezione Vista / Esportazione STL] */
// 0 = Assieme completo, 1 = Base Azimuth (PVC), 2 = Forcella Elevation, 3 = Morsetto Boom, 4 = Cavallotto Boom
pezzo_da_visualizzare = 0; // [0:Assieme Completo, 1:Base Azimuth PVC, 2:Forcella Elevation, 3:Morsetto Boom, 4:Cavallotto Boom]

/* [Quote Servo DS3230 PRO con tolleranza +0.5mm] */
servo_L = 40.5;          // Lunghezza corpo
servo_W = 20.5;          // Larghezza corpo
servo_H_tot = 40.5;      // Altezza corpo (escluso pignone)
servo_H_flange = 27.8;   // Altezza dal fondo alla base delle alette
servo_flange_T = 3.1;    // Spessore alette
servo_flange_L = 54.5;   // Larghezza totale con alette
servo_hole_dx = 49.5;    // Interasse longitudinale fori viti
servo_hole_dy = 10.0;    // Interasse trasversale fori viti
servo_shaft_off = 10.0;  // Distanza asse pignone dal bordo corpo

/* [Quote Squadretta Circolare Alluminio 25T] */
horn_diam = 20.5;        // Diametro esterno squadretta
horn_thick = 2.5;        // Spessore incasso squadretta
horn_pcd = 14.0;         // Diametro interasse fori viti M2/M2.5 sulla squadretta

/* [Quote Antenna & Cuscinetto] */
boom_slot_size = 20.5;   // Canale boma con tolleranza (20.5 x 20.5mm)
bearing_od = 22.0;       // Cuscinetto 608ZZ diametro esterno
bearing_id = 8.0;        // Cuscinetto 608ZZ diametro interno
bearing_h = 7.0;         // Cuscinetto 608ZZ spessore

/* [Attacco Tubo PVC] */
pvc_hole_d = 25.5;       // Diametro foro bicchiere per tubo PVC (25.5mm)
pvc_socket_depth = 60.0; // Profondità di innesto a 60mm
pvc_wall = 5.5;          // Spessore parete del bicchiere di innesto
lock_bolt_d = 6.2;       // Foro passante per vite/coppiglia di blocco M6

/* [Parametri di Stampa e Tolleranze] */
tol = 0.25;              // Tolleranza accoppiamento cuscinetto
$fn = 64;

// Quote derivate
sL = servo_L;
sW = servo_W;
bS = boom_slot_size;

// ============================================================================
// MODULI DI UTILITA'
// ============================================================================

module ds3230_cutout(cable_slot_mode = "base") {
    // Corpo principale servo (passante)
    translate([-sL/2, -sW/2, -5])
        cube([sL, sW, servo_H_tot + 15]);
    
    // Scasso alette di fissaggio
    translate([-(servo_flange_L + 0.5)/2, -sW/2, servo_H_flange - 0.2])
        cube([servo_flange_L + 0.5, sW, servo_flange_T + 15]);
        
    // 4 Fori viti M3 (pre-foro autofilettante 2.6mm)
    for (x = [-servo_hole_dx/2, servo_hole_dx/2]) {
        for (y = [-servo_hole_dy/2, servo_hole_dy/2]) {
            translate([x, y, servo_H_flange - 12])
                cylinder(d = 2.6, h = 20);
        }
    }

    // Passacavi base (per il servo Azimuth)
    if (cable_slot_mode == "base") {
        for (s = [1, -1]) {
            scale([s, 1, 1]) {
                translate([sL/2 - 1, -5, 0])
                    cube([22, 10, servo_H_flange + 12]);
                translate([sL/2 - 4, -sW/2 - 4, -1])
                    cube([15, sW + 8, 12]);
            }
        }
    }
}

module horn_pocket() {
    cylinder(d = horn_diam, h = horn_thick + 0.5);
    cylinder(d = 7.0, h = 15, center = true);
    for (a = [0, 90, 180, 270]) {
        rotate([0, 0, a])
            translate([horn_pcd/2, 0, -5])
                cylinder(d = 2.4, h = 15);
    }
}

// ============================================================================
// PEZZO 1: BASE AZIMUTH
// ============================================================================
module base_azimuth() {
    base_d = 76;
    head_h = servo_H_flange + 6;
    socket_od = pvc_hole_d + pvc_wall*2; 
    servo_offset_x = (servo_L/2 - servo_shaft_off);

    difference() {
        union() {
            cylinder(d = socket_od, h = pvc_socket_depth);
            translate([0, 0, pvc_socket_depth])
                cylinder(d1 = socket_od, d2 = base_d, h = 14);
            translate([0, 0, pvc_socket_depth + 14])
                cylinder(d = base_d, h = head_h - 14);
        }
        
        translate([0, 0, -0.1])
            cylinder(d = pvc_hole_d, h = pvc_socket_depth + 0.1);
            
        translate([0, 0, -0.1])
            cylinder(d1 = pvc_hole_d + 2.0, d2 = pvc_hole_d, h = 2.5);
            
        translate([0, 0, pvc_socket_depth / 2])
            rotate([90, 0, 0])
                cylinder(d = lock_bolt_d, h = socket_od + 4, center = true);
                
        translate([0, -socket_od/2 + 2.2, pvc_socket_depth / 2])
            rotate([90, 30, 0])
                cylinder(d = 11.5, h = 4.0, $fn = 6);

        translate([servo_offset_x, 0, pvc_socket_depth + 6])
            ds3230_cutout(cable_slot_mode = "base");
            
        for (y = [-24, 24]) {
            translate([0, y, pvc_socket_depth + head_h/2 + 2])
                rotate([0, 90, 0])
                    cylinder(d = 13, h = base_d + 2, center = true);
        }
    }
}

// ============================================================================
// PEZZO 2: FORCELLA ELEVATION
// ============================================================================
module forcella_elevation() {
    inner_span = 58;          // Luce interna 58 mm
    arm_thick_left = 22;      // Spessore braccio sinistro
    arm_thick_right = 16;     // Spessore braccio destro
    plate_thick = 12;         // Spessore base forcella
    arm_h = 75;               
    arm_w = 46;               
    arm_top_h = arm_h + 20;   

    difference() {
        union() {
            translate([-(inner_span/2 + arm_thick_left), -arm_w/2, 0])
                cube([inner_span + arm_thick_left + arm_thick_right, arm_w, arm_top_h]);
        }
        
        translate([-inner_span/2, -arm_w/2 - 1, plate_thick])
            cube([inner_span, arm_w + 2, arm_top_h]);

        translate([0, 0, -0.1])
            horn_pocket();
            
        translate([-inner_span/2, 0, arm_h])
            rotate([0, 90, 0])
                rotate([0, 0, 180])
                    translate([-servo_shaft_off, 0, -arm_thick_left - 2])
                        ds3230_cutout(cable_slot_mode = "fork");
            
        translate([-inner_span/2 - arm_thick_left - 1, -4, arm_h + 10])
            cube([arm_thick_left + 2, 8, 24]);

        translate([inner_span/2 - 0.1, 0, arm_h])
            rotate([0, 90, 0])
                cylinder(d = bearing_od + tol, h = bearing_h + 0.5);
        translate([inner_span/2 - 1, 0, arm_h])
            rotate([0, 90, 0])
                cylinder(d = bearing_id + 2, h = arm_thick_right + 2);

        translate([-bS/2 - 1, -10, plate_thick - 2.0])
            cube([bS + 2, 20, 4.0]);
    }
}

// ============================================================================
// PEZZO 3: MORSETTO PORTA-BOOM ESTESO (90mm di lunghezza)
// ============================================================================
module morsetto_boom() {
    clamp_x = 42;          
    clamp_y = 90;          // Lunghezza estesa a 90 mm
    clamp_h = 30;          
    
    difference() {
        union() {
            translate([-clamp_x/2, -clamp_y/2, -clamp_h/2])
                cube([clamp_x, clamp_y, clamp_h]);
                
            translate([clamp_x/2, 0, 0])
                rotate([0, 90, 0])
                    cylinder(d = bearing_id - 0.15, h = 13);
            translate([clamp_x/2, 0, 0])
                rotate([0, 90, 0])
                    cylinder(d = 12, h = 4);
        }
        
        // Canale boma da 90mm
        translate([-bS/2, -clamp_y/2 - 1, clamp_h/2 - 14.0])
            cube([bS, clamp_y + 2, 16]);
            
        translate([-clamp_x/2 - 0.1, 0, 0])
            rotate([0, 90, 0])
                horn_pocket();
                
        // 6 Fori viti M3 distribuiti sulla lunghezza (3 file per lato a y = -32, 0, 32)
        for (x = [-16.5, 16.5]) {
            for (y = [-32, 0, 32]) {
                translate([x, y, 0])
                    cylinder(d = 3.5, h = clamp_h + 2, center = true);
            }
        }
    }
}

// ============================================================================
// PEZZO 4: CAVALLOTTO SUPERIORE CHIUSURA BOOM ESTESO (90mm)
// ============================================================================
module cavallotto_boom() {
    clamp_x = 42;
    clamp_y = 90;          // Lunghezza estesa a 90 mm
    cap_h = 12;
    
    difference() {
        translate([-clamp_x/2, -clamp_y/2, 0])
            cube([clamp_x, clamp_y, cap_h]);
            
        // Sede boma da 90mm
        translate([-bS/2, -clamp_y/2 - 1, -0.1])
            cube([bS, clamp_y + 2, 6.5]);
            
        // 6 Fori viti M3 con svasatura testa vite
        for (x = [-16.5, 16.5]) {
            for (y = [-32, 0, 32]) {
                translate([x, y, -1])
                    cylinder(d = 4.3, h = cap_h + 2);
                translate([x, y, cap_h - 3.5])
                    cylinder(d = 7.5, h = 4);
            }
        }
    }
}

// ============================================================================
// RENDERING
// ============================================================================
if (pezzo_da_visualizzare == 0) {
    color("DimGray") base_azimuth();
    color("SteelBlue") translate([0, 0, pvc_socket_depth + servo_H_flange + 12]) forcella_elevation();
    color("DarkOrange") translate([0, 0, pvc_socket_depth + servo_H_flange + 12 + 75]) morsetto_boom();
    color("Gold") translate([0, 0, pvc_socket_depth + servo_H_flange + 12 + 75 + 16]) cavallotto_boom();
    
    %translate([-10.25, -500, pvc_socket_depth + servo_H_flange + 12 + 75 + 0.9])
        cube([20.5, 1000, 20.5]);
} else if (pezzo_da_visualizzare == 1) {
    base_azimuth();
} else if (pezzo_da_visualizzare == 2) {
    forcella_elevation();
} else if (pezzo_da_visualizzare == 3) {
    rotate([0, -90, 0]) morsetto_boom();
} else if (pezzo_da_visualizzare == 4) {
    cavallotto_boom();
}