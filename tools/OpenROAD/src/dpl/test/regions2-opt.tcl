# regions1
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_def regions2-opt.def
improve_placement
check_placement

set def_file [make_result_file regions2-opt.def]
write_def $def_file
diff_file $def_file regions2-opt.defok
