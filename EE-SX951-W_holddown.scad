// Omron EE-SX951-W peg hold-down, dimensions in mm.
// The sensor sits on the same flat surface as the two screw feet.
// In the installed top view, the origin is midway between the sensor holes.
// All four centers lie on X. +Y points toward the cable.
// F6 and export STL with print_orientation enabled for a flat print face.

/* [Mounting screws] */
// Slot-to-slot spacing, center to center.
screw_spacing = 25.4;
// Distance from the nearest peg to the short-reach position of the first screw.
screw_offset = 25.4;
// Additional reach from short to long position (3/8 inch).
screw_travel = 9.525;
// Which end of the sensor gets the screw pair.
mount_side = "right"; // [right,left]
// Clearance for nominal 1/8-inch screws.
screw_clearance_diameter = 3.6;
// Counterbore width and depth for a standard #6-32 socket-head cap screw.
screw_head_pocket_width = 6.2;
screw_head_pocket_depth = 3.55;
screw_foot_diameter = 10;

/* [Sensor fit] */
// Omron E435, page 6: EE-SX951 mounting hole pitch is 8 mm.
sensor_hole_spacing = 8;
peg_diameter = 2.8;
// Depth below the contact face. Keep shorter than the tab height.
peg_engagement = 2.6;
peg_tip_chamfer = 0.45;
contact_pad_diameter = 4.2;
// Measure from the mounting plate to the flats around the sensor holes.
sensor_tab_height = 4.1;
// Height of the optical slot floor above the mounting plate (12 - 6.5).
slot_floor_height = 5.5;
// Relief extends past both edges of the sensor's 5 mm optical slot.
disk_relief_width = 6.4;
// Vertical gap between the slot floor and the top of the recessed strap.
disk_relief_clearance = 0.6;
// Inside radius where the recessed strap meets each peg shoulder.
disk_relief_fillet = 0.55;

/* [Bracket] */
body_thickness = 3.2;
// Lift the arms above the contact pads to clear the molded edges.
tab_edge_clearance = 0.7;
// Blend the underside of the screw-side peg pad into the long arm.
pad_blend_length = 1.4;
// Flip the bracket so the screw end and peg shoulders sit on the print bed.
print_orientation = true;

/* [Hidden] */
$fn = 64;
overlap = 0.02;
arm_base = sensor_tab_height + tab_edge_clearance;
top_z = arm_base + body_thickness;
relief_top_z = slot_floor_height - disk_relief_clearance;
relief_cut_span = 2 * max(contact_pad_diameter, screw_foot_diameter);

peg_centers = [[-sensor_hole_spacing / 2, 0],
               [ sensor_hole_spacing / 2, 0]];
// Keep the strap narrow across the 13.4 mm sensor housing.
strap_end = [max(7.5, sensor_hole_spacing / 2), 0];
first_screw_x = sensor_hole_spacing / 2 + screw_offset;
screw_centers = [[first_screw_x, 0], [first_screw_x + screw_spacing, 0]];

assert(peg_engagement > peg_tip_chamfer && peg_engagement < sensor_tab_height,
       "Peg must have a straight section and stop above the mounting plate.");
assert(peg_diameter > 2 * peg_tip_chamfer && peg_diameter < contact_pad_diameter,
       "Peg must have a positive tip diameter and fit within the contact pad.");
assert(screw_travel > 0 && screw_spacing > screw_foot_diameter + screw_travel &&
       screw_foot_diameter >= screw_clearance_diameter + 3,
       "Screw feet must be distinct and have wall around the holes.");
assert(screw_head_pocket_width > screw_clearance_diameter &&
       screw_foot_diameter >= screw_head_pocket_width + 3,
       "Leave a screw head seat and at least 1.5 mm of wall around the pocket.");
assert(screw_head_pocket_depth > 0 && top_z - screw_head_pocket_depth >= 1.5,
       "Leave at least 1.5 mm of material under the screw head seat.");
assert(first_screw_x - screw_foot_diameter / 2 > strap_end[0],
       "Move the first screw farther from the sensor.");
assert(body_thickness > 0 && tab_edge_clearance >= 0);
assert(pad_blend_length >= 0 && peg_centers[1][0] + pad_blend_length <= strap_end[0],
       "Keep the underside blend within the narrow sensor strap.");
assert(disk_relief_width > 5 && disk_relief_width < sensor_hole_spacing,
       "Extend the relief past the slot edges and keep the peg shoulders.");
assert(disk_relief_clearance > 0 && relief_top_z >= sensor_tab_height + 0.6,
       "Keep at least 0.6 mm of strap below the disk relief.");
assert(disk_relief_fillet > 0 && disk_relief_fillet <= disk_relief_clearance &&
       disk_relief_width - 2 * disk_relief_fillet >= 5,
       "Keep the fillets below the slot floor and outside the optical slot width.");

module disk_at(point, z, diameter, height) {
    translate([point[0], point[1], z])
        cylinder(d = diameter, h = height);
}

module screw_sweep(point, z, diameter, height) {
    hull() {
        disk_at(point, z, diameter, height);
        disk_at(point + [screw_travel, 0], z, diameter, height);
    }
}

module locating_peg(point) {
    disk_at(point, sensor_tab_height - peg_engagement + peg_tip_chamfer,
            peg_diameter, peg_engagement - peg_tip_chamfer + overlap);
    translate([point[0], point[1], sensor_tab_height - peg_engagement])
        cylinder(d1 = peg_diameter - 2 * peg_tip_chamfer,
                 d2 = peg_diameter, h = peg_tip_chamfer + overlap);
}

module disk_relief_cut() {
    half_width = disk_relief_width / 2;
    depth = top_z - relief_top_z;
    radius = min(disk_relief_fillet, depth);
    shoulder = [for (angle = [0:7.5:90])
        [half_width - radius + radius * sin(angle), radius * (1 - cos(angle))]];

    // Draw the rounded recess in X/Z and cut across the strap in Y.
    translate([0, relief_cut_span / 2, relief_top_z])
        rotate([90, 0, 0])
            linear_extrude(height = relief_cut_span)
                polygon(concat(
                    shoulder,
                    [[half_width, depth + overlap], [-half_width, depth + overlap]],
                    [for (i = [len(shoulder) - 1:-1:0])
                        [-shoulder[i][0], shoulder[i][1]]]
                ));
}

module pad_blend_section(angle) {
    shift = pad_blend_length * (1 - cos(angle));
    z = sensor_tab_height + tab_edge_clearance * sin(angle);
    disk_at(peg_centers[1] + [shift, 0], z, contact_pad_diameter, overlap);
}

module hold_down() {
    difference() {
        union() {
            hull() {
                disk_at(peg_centers[0], arm_base, contact_pad_diameter, body_thickness);
                disk_at(strap_end, arm_base, contact_pad_diameter, body_thickness);
            }

            hull() {
                disk_at(strap_end, arm_base, contact_pad_diameter, body_thickness);
                disk_at(screw_centers[0], arm_base, screw_foot_diameter, body_thickness);
            }

            hull()
                for (point = screw_centers)
                    screw_sweep(point, arm_base, screw_foot_diameter, body_thickness);

            // Fill below the relief to tie the two peg pads together.
            hull()
                for (point = peg_centers)
                    disk_at(point, sensor_tab_height, contact_pad_diameter,
                            tab_edge_clearance + overlap);

            if (tab_edge_clearance > 0 && pad_blend_length > 0)
                for (angle = [0:15:75])
                    hull() {
                        pad_blend_section(angle);
                        pad_blend_section(angle + 15);
                    }

            for (point = peg_centers) {
                locating_peg(point);
            }

            for (point = screw_centers)
                screw_sweep(point, 0, screw_foot_diameter, arm_base + overlap);
        }

        // Open a path across the full strap width in the plane of the disk.
        if (relief_top_z < top_z)
            disk_relief_cut();

        for (point = screw_centers) {
            screw_sweep(point, -overlap, screw_clearance_diameter, top_z + 2 * overlap);
            screw_sweep(point, top_z - screw_head_pocket_depth,
                        screw_head_pocket_width, screw_head_pocket_depth + overlap);
        }
    }
}

module oriented_hold_down() {
    if (mount_side == "left")
        mirror([1, 0, 0]) hold_down();
    else
        hold_down();
}

if (print_orientation)
    translate([0, 0, top_z]) mirror([0, 0, 1]) oriented_hold_down();
else
    oriented_hold_down();
