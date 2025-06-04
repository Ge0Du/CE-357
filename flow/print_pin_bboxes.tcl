# ----------------------------------------------------------------------------
# print_pin_bboxes.tcl
#
# Usage: openroad -no_init print_pin_bboxes.tcl
#
# This script will:
#   1) Read in the ASAP7 technology LEF and any macro LEFs you need.
#   2) Read in your placed DEF.
#   3) Iterate over every hard macro and print each pin’s
#      full bounding box (xLo,yLo,xHi,yHi) on the lowest routing layer.
#   4) Print all row Y‐positions so you can compare “pin yLo” to “row yLo”.
#
# Edit the variables below (TECH_LEF, RAM_LEF, DEF_FILE) to match your paths.
# ----------------------------------------------------------------------------
lappend auto_path /home/du/Documents/OpenROAD-flow-scripts/tools/OpenROAD/build/lib/odb/tcl
# dump_odbtcl_cmds.tcl
package require odbtcl

# List everything under ::odb
puts "\n-- all commands in ::odb --"
foreach c [lsort [namespace children ::odb]] {
    puts $c
}
# === 1) User‐modifiable paths ===
# Path to the ASAP7 technology LEF (defines layers, pitches, etc.)
set TECH_LEF    "~/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7_tech_1x_201209.lef"

# Path to the RAM macro LEF(s) used in your design. Add more lines if needed.
# (You must load every fake‐RAM LEF that appears in your DEF.)
set RAM_LEF_256x32  "~/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram7_256x32.lef"
# Example for other sizes:
# set RAM_LEF_512x8  "~/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram7_512x8.lef"

# Path to your placed‐DEF file
set DEF_FILE    "~/Documents/OpenROAD-flow-scripts/flow/custom_macro_placement.def"

# Additional SRAM/DFT LEFs that appear in your DEF
set SRAM_FILE     "~/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_28_SRAM_1x_220121a.lef"
set SRAM_FILE1    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_28_L_1x_220121a.lef"
set SRAM_FILE2    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_28_R_1x_220121a.lef"
set SRAM_FILE3    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_28_SL_1x_220121a.lef"
set SRAM_FILE4    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_DFFHQNH2V2X.lef"
set SRAM_FILE5    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_DFFHQNV2X.lef"
set SRAM_FILE6    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_DFFHQNV4X.lef"
set SRAM_FILE7    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/asap7sc7p5t_DFHV2X.lef"
set SRAM_FILE8    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram_32x46.lef"
set SRAM_FILE9    "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram_64x20.lef"
set SRAM_FILE10   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram_64x22.lef"
set SRAM_FILE11   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram_256x64.lef"
set SRAM_FILE12   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram_256x128.lef"
set SRAM_FILE13   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeregfile_64x64.lef"
set SRAM_FILE14   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram7_64x21.lef"
set SRAM_FILE15   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram7_256x34.lef"
set SRAM_FILE16   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram7_2048x39.lef"
set SRAM_FILE17   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeregfile_32x46.lef"
set SRAM_FILE18   "/home/du/Documents/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeregfile_128x64.lef"

# ----------------------------------------------------------------------------
# === 2) Read LEFs and DEF ===
# ----------------------------------------------------------------------------

# 2.1) Read the ASAP7 technology LEF first
puts "Reading tech LEF: $TECH_LEF"
if {![file exists [file normalize $TECH_LEF]]} {
    puts stderr "ERROR: Cannot find TECH_LEF at $TECH_LEF"
    exit 1
}
read_lef [file normalize $TECH_LEF]

# 2.2) Read each SRAM/DFT LEF in any order
foreach f [list \
        $SRAM_FILE1  $SRAM_FILE2  $SRAM_FILE3  $SRAM_FILE4 \
        $SRAM_FILE5  $SRAM_FILE6  $SRAM_FILE7  $SRAM_FILE8 \
        $SRAM_FILE9  $SRAM_FILE10 $SRAM_FILE11 $SRAM_FILE12 \
        $SRAM_FILE13 $SRAM_FILE14 $SRAM_FILE15 $SRAM_FILE16 \
        $SRAM_FILE17 $SRAM_FILE18] {
    if {![file exists [file normalize $f]]} {
        puts stderr "ERROR: Cannot find LEF at $f"
        exit 1
    }
    read_lef [file normalize $f]
}

# 2.3) Read any additional SRAM macro LEF(s)
puts "Reading SRAM LEF: $SRAM_FILE"
if {![file exists [file normalize $SRAM_FILE]]} {
    puts stderr "ERROR: Cannot find SRAM_LEF at $SRAM_FILE"
    exit 1
}
read_lef [file normalize $SRAM_FILE]

# 2.4) Read every fake‐RAM LEF that your DEF refers to
puts "Reading RAM macro LEF: $RAM_LEF_256x32"
if {![file exists [file normalize $RAM_LEF_256x32]]} {
    puts stderr "ERROR: Cannot find RAM_LEF_256x32 at $RAM_LEF_256x32"
    exit 1
}
read_lef [file normalize $RAM_LEF_256x32]

# ----------------------------------------------------------------------------
# 2.5) Read the placed DEF
# ----------------------------------------------------------------------------
puts "Reading DEF: $DEF_FILE"
if {![file exists [file normalize $DEF_FILE]]} {
    puts stderr "ERROR: Cannot find DEF_FILE at $DEF_FILE"
    exit 1
}
read_def [file normalize $DEF_FILE]
# ----------------------------------------------------------------------------
# === 3) Print each hard macro’s pin bounding box ===
# ----------------------------------------------------------------------------

puts "\n=== Printing pin bounding boxes for every HARD macro ==="

# We skip “get_db” entirely.  Instead, let db_inst_iter walk the top‐level block by default:
set instIter [db_inst_iter]
while {[db_inst_iter_has_more $instIter]} {
    set instObj [db_inst_iter_next $instIter]

    # Skip any “soft” (standard‐cell) instances—only print HARD macros:
    if {![db_inst_is_hard $instObj]} {
        continue
    }

    # Print the macro instance’s name
    set instName [db_inst_name $instObj]
    puts "\nInst: $instName"

    # For each pin (ITerm) of that macro, grab its MTerm name + full bbox on the lowest layer
    set itermIter [db_inst_iterm_iter $instObj]
    while {[db_inst_iterm_iter_has_more $itermIter]} {
        set itermObj [db_inst_iterm_iter_next $itermIter]

        # Pin name = MTerm name
        set mtermObj  [db_iterm_mterm $itermObj]
        set pinName   [db_mterm_name $mtermObj]

        # db_iterm_bbox returns {xLo yLo xHi yHi} on the lowest routing layer
        set bboxList  [db_iterm_bbox $itermObj]
        lassign $bboxList lx ly ux uy

        puts "  Pin $pinName   bbox=($lx,$ly)-($ux,$uy)"
    }
}

# ----------------------------------------------------------------------------
# === 4) Print all row Y‐positions (bottom‐edge) ===
# ----------------------------------------------------------------------------
puts "\n=== Row Y‐positions (bottom‐edge of each row) ==="
set rowIter [db_row_iter]
while {[db_row_iter_has_more $rowIter]} {
    set rowObj  [db_row_iter_next $rowIter]
    set rowName [db_row_name $rowObj]

    # db_row_bbox → {xLo yLo xHi yHi}, so yLo is the bottom edge
    set rowBBox [db_row_bbox $rowObj]
    lassign $rowBBox xlo ylo xhi yhi

    puts "  $rowName at Y = $ylo"
}

puts "\nDone.\n"