# macro_place_util.tcl

# Check if macros exist
if {[find_macros] != ""} {

  # Default report and blockage files for RTL-MP
  if {![env_var_exists_and_non_empty RTLMP_RPT_DIR]} {
    set ::env(RTLMP_RPT_DIR) "$::env(OBJECTS_DIR)/rtlmp"
  }
  if {![env_var_exists_and_non_empty RTLMP_RPT_FILE]} {
    set ::env(RTLMP_RPT_FILE) "partition.txt"
  }
  if {![env_var_exists_and_non_empty RTLMP_BLOCKAGE_FILE]} {
    set ::env(RTLMP_BLOCKAGE_FILE) "$::env(OBJECTS_DIR)/rtlmp/partition.txt.blockage"
  }

  # Macro wrapper replacement (if defined)
  if {[env_var_exists_and_non_empty MACRO_WRAPPERS]} {
    source $::env(MACRO_WRAPPERS)

    set wrapped_macros [dict keys [dict get $wrapper around]]
    set db [ord::get_db]
    set block [ord::get_db_block]

    foreach inst [$block getInsts] {
      if {[lsearch -exact $wrapped_macros [[$inst getMaster] getName]] > -1} {
        set new_master [dict get $wrapper around [[$inst getMaster] getName]]
        puts "Replacing [[$inst getMaster] getName] with $new_master for [$inst getName]"
        $inst swapMaster [$db findMaster $new_master]
      }
    }
  }

  # Setup halo and blockage width
  lassign $::env(MACRO_PLACE_HALO) halo_x halo_y
  set blockage_width [expr max($halo_x, $halo_y)]
  if {[env_var_exists_and_non_empty MACRO_BLOCKAGE_HALO]} {
    set blockage_width $::env(MACRO_BLOCKAGE_HALO)
  }

  # Decide which placer to run
  if {[info exists ::env(USE_CUSTOM_PLACER)] && $::env(USE_CUSTOM_PLACER) == "1"} {
    puts "Running custom macro_place plugin..."
    macro_place
    puts "macro_place complete."
    write_def custom_macro_placement.def
    puts "custom def completed"
  } else {
    puts "Running default RTL-MP..."

    set additional_rtlmp_args ""
    append additional_rtlmp_args " -halo_width $halo_x"
    append additional_rtlmp_args " -halo_height $halo_y"
    append additional_rtlmp_args " -target_util [place_density_with_lb_addon]"

    append_env_var additional_rtlmp_args RTLMP_MAX_LEVEL -max_num_level 1
    append_env_var additional_rtlmp_args RTLMP_MAX_INST -max_num_inst 1
    append_env_var additional_rtlmp_args RTLMP_MIN_INST -min_num_inst 1
    append_env_var additional_rtlmp_args RTLMP_MAX_MACRO -max_num_macro 1
    append_env_var additional_rtlmp_args RTLMP_MIN_MACRO -min_num_macro 1
    append_env_var additional_rtlmp_args RTLMP_MIN_AR -min_ar 1
    append_env_var additional_rtlmp_args RTLMP_SIGNATURE_NET_THRESHOLD -signature_net_threshold 1
    append_env_var additional_rtlmp_args RTLMP_AREA_WT -area_weight 1
    append_env_var additional_rtlmp_args RTLMP_WIRELENGTH_WT -wirelength_weight 1
    append_env_var additional_rtlmp_args RTLMP_OUTLINE_WT -outline_weight 1
    append_env_var additional_rtlmp_args RTLMP_BOUNDARY_WT -boundary_weight 1
    append_env_var additional_rtlmp_args RTLMP_NOTCH_WT -notch_weight 1
    append_env_var additional_rtlmp_args RTLMP_DEAD_SPACE -target_dead_space 1
    append_env_var additional_rtlmp_args RTLMP_RPT_DIR -report_directory 1
    append_env_var additional_rtlmp_args RTLMP_FENCE_LX -fence_lx 1
    append_env_var additional_rtlmp_args RTLMP_FENCE_LY -fence_ly 1
    append_env_var additional_rtlmp_args RTLMP_FENCE_UX -fence_ux 1
    append_env_var additional_rtlmp_args RTLMP_FENCE_UY -fence_uy 1

    set all_args $additional_rtlmp_args
    if {[env_var_exists_and_non_empty RTLMP_ARGS]} {
      set all_args $::env(RTLMP_ARGS)
    }

    rtl_macro_placer {*}$all_args
    puts "RTL-MP macro_placement complete."
    write_def baseline_macro_placement.def
    puts "RTL-MP def completed"
  }

  # Add placement blockages based on macro halo
  source $::env(SCRIPTS_DIR)/placement_blockages.tcl
  block_channels $blockage_width

} else {
  puts "No macros found. Skipping macro placement."
}
