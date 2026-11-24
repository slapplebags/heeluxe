
 // Flat collar-mounted photointerrupter timing disk. Units: millimeters.
 // Two radial notches 180 degrees apart produce two optical pulses per revolution.
 // One radial slot clears a #10-24 screw threaded into the metal collar.
 // Shaft diameter supplied by user; screw offset is approximate. Verify fit before printing.
 // Disk sits against the collar face; the center hole clears the shaft, not the collar OD.
 // Changing outer diameter does NOT move the mounting hole or resize the center opening.

 /* [Disk] */
 outer_diameter_in = 8.125;
 thickness_mm = 2.0;

 /* [Spokes and rim] */
 spoke_count = 4;
 spoke_width_mm = 12.0;
 spoke_angle_deg = 45;
 // Keep wider than notch depth so the rim stays connected beneath the notches.
 rim_width_mm = 12.0;
 // Solid hub supports the offset screw and its washer.
 hub_diameter_mm = 48.0;

 /* [Mounting - verify against hardware] */
 // 1-inch shaft, supplied by user.
 shaft_diameter_mm = 25.4;
 shaft_clearance_mm = 0.4;
 // Distance from shaft center to threaded hole center; approximately 17 mm.
 screw_offset_mm = 17.0;
 // 0 = right, 90 = up, 180 = left.
 screw_angle_deg = 180;
 // Clearance hole, NOT a printed #10-24 thread. Adjust for printer fit.
 screw_clearance_diameter_mm = 5.3;
 // Extra radial travel beyond the screw diameter: 4 mm gives +/-2 mm adjustment.
 // Slot overall length = screw_clearance_diameter_mm + screw_slot_travel_mm.
 screw_slot_travel_mm = 4.0;

 /* [Photointerrupter notches] */
 // Two rectangular notches, exactly 180 degrees apart.
 // Produces two optical pulses per revolution.
 notch_count = 2;
 notch_width_mm = 38.0;
 notch_depth_mm = 8.0;
 // First notch angle; second is automatically opposite.
 // Independent of mounting-hole angle.
 notch_angle_deg = 0;

 /* [Engraved labels] */
 text_enabled = true;
 label_1 = "Doc Brown";
 label_2 = "Heeluxe Time Machine";
 text_size_mm = 8.0;
 text_spacing_mm = 8.0;
 text_depth_mm = 0.6;
 text_font = "Liberation Sans:style=Bold";
 // Second label automatically sits 180 degrees from the first.
 text_center_angle_deg = 90;

 /* [Output] */
 // Enable for 2D DXF/SVG export; disable for STL/3D printing.
 export_2d = false;

 /* [Hidden] */
 $fn = 360;

 outer_diameter_mm = outer_diameter_in * 25.4;
 radius = outer_diameter_mm / 2;
 center_hole_diameter_mm = shaft_diameter_mm + shaft_clearance_mm;
 rim_inner_radius = radius - rim_width_mm;
 hub_radius = hub_diameter_mm / 2;

 slot_inner_center = screw_offset_mm - screw_slot_travel_mm / 2;
 slot_outer_center = screw_offset_mm + screw_slot_travel_mm / 2;

 text_radius = radius - rim_width_mm / 2;

 assert(thickness_mm > 0,
        "Thickness must be positive");

 assert(!text_enabled || (text_depth_mm > 0 && text_depth_mm < thickness_mm),
        "Engraving depth must be positive and less than disk thickness");

 assert(center_hole_diameter_mm > 0,
        "Center opening must be positive");

 assert(notch_count == 2,
        "This disk requires exactly two notches");

 assert(notch_width_mm > 0 && notch_width_mm < outer_diameter_mm,
        "Notch width must be positive and smaller than disk diameter");

 assert(notch_depth_mm > 0 && notch_depth_mm < radius - center_hole_diameter_mm / 2,
        "Notch must leave material between notch and center opening");

 assert(screw_clearance_diameter_mm > 0,
        "Screw hole must be positive");

 assert(screw_slot_travel_mm >= 0,
        "Slot travel cannot be negative");

 assert(slot_inner_center - screw_clearance_diameter_mm / 2 > 0,
        "Slot extends across the shaft center");

 assert(slot_outer_center + screw_clearance_diameter_mm / 2 < radius - notch_depth_mm,
        "Disk too small for mount and notches");

 assert(spoke_count >= 3 && spoke_count == floor(spoke_count),
        "Use at least three spokes");

 assert(spoke_width_mm > 0 && spoke_width_mm < hub_diameter_mm,
        "Invalid spoke width");

 assert(rim_width_mm > notch_depth_mm,
        "Rim width must exceed notch depth");

 assert(hub_radius < rim_inner_radius,
        "Outer diameter too small for hub and rim");

 assert(hub_radius > slot_outer_center + screw_clearance_diameter_mm / 2 + 2,
        "Hub must leave at least 2 mm around mounting screw");

 // Main disk outline: solid hub, outer rim, and connecting spokes.
 module spoked_blank() {
     union() {
         circle(d = hub_diameter_mm);

         difference() {
             circle(d = outer_diameter_mm);
             circle(r = rim_inner_radius);
         }

         // Extend spokes into both hub and rim for a single connected part.
         for (i = [0 : spoke_count - 1]) {
             rotate(spoke_angle_deg + i * 360 / spoke_count)
                 translate([0, -spoke_width_mm / 2])
                     square([
                         rim_inner_radius + rim_width_mm / 2,
                         spoke_width_mm
                     ]);
         }
     }
 }

 // Two equally spaced rectangular optical notches.
 module optical_notches() {
     for (i = [0 : notch_count - 1]) {
         rotate(notch_angle_deg + i * 360 / notch_count)
             translate([
                 radius - notch_depth_mm,
                 -notch_width_mm / 2
             ])
                 square([
                     notch_depth_mm + 1,
                     notch_width_mm
                 ]);
     }
 }

 // Complete 2D cutting profile.
 module disk_profile() {
     difference() {
         spoked_blank();

         // Center shaft clearance.
         circle(d = center_hole_diameter_mm);

         // Radial mounting slot.
         // Opens into shaft clearance at its inner end,
         // allowing the full +/-2 mm adjustment range.
         rotate(screw_angle_deg)
             hull() {
                 translate([slot_inner_center, 0])
                     circle(d = screw_clearance_diameter_mm);

                 translate([slot_outer_center, 0])
                     circle(d = screw_clearance_diameter_mm);
             }

         // Cut two notches exactly 180 degrees apart.
         optical_notches();
     }
 }

 // Engraved curved lettering.
 module arc_text(label, center_angle) {
     step_angle = text_spacing_mm / text_radius * 180 / PI;

     if (len(label) > 0) {
         for (i = [0 : len(label) - 1]) {
             angle = center_angle
                     + ((len(label) - 1) / 2 - i) * step_angle;

             rotate([0, 0, angle])
                 translate([
                     text_radius,
                     0,
                     thickness_mm - text_depth_mm
                 ])
                     rotate([0, 0, -90])
                         linear_extrude(height = text_depth_mm + 0.01)
                             text(
                                 label[i],
                                 size = text_size_mm,
                                 font = text_font,
                                 halign = "center",
                                 valign = "center"
                             );
         }
     }
 }

 // Output selection.
 if (export_2d) {

     // Cutting outline only.
     // Shallow lettering is a separate machining operation.
     disk_profile();

 } else {

     difference() {
         linear_extrude(
             height = thickness_mm,
             convexity = 10
         )
             disk_profile();

         if (text_enabled) {
             arc_text(
                 label_1,
                 text_center_angle_deg
             );

             arc_text(
                 label_2,
                 text_center_angle_deg + 180
             );
         }
     }
 }

 echo(
     outer_diameter_mm = outer_diameter_mm,
     center_opening_mm = center_hole_diameter_mm,
     screw_center_offset_mm = screw_offset_mm,
     notch_count = notch_count,
     notch_spacing_deg = 360 / notch_count,
     notch_width_mm = notch_width_mm,
     notch_depth_mm = notch_depth_mm
 );

 // Print flat in opaque material.
 // Match thickness to the sensor's actual slot.
 // Check shaft clearance, screw length/engagement,
 // and rim runout before powered use.
