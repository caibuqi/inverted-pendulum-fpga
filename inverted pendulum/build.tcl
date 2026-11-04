# =====================================================================
#  Headless build for the pendulum project.
#      gw_sh.exe build.tcl
# =====================================================================
open_project "inverted pendulum.gprj"
set_option -top_module top
run all
