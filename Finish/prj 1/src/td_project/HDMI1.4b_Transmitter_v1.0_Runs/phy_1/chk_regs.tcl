import_device eagle_s20.db -package EG4S20BG256
open_project {HDMI1.4b_Transmitter_v1.0.prj} -noanalyze
import_db HDMI1.4b_Transmitter_v1.0_pr.db
foreach p {{*sub_vec_v*} {*sub_len_v*} {*sub_s[0]*} {*beat_s[0]*} {*sub_s*} {*beat_s*} {*/s1} {*/cdc_*_s1*}} {
    set r [get_regs -hier $p]
    puts "CHK $p => [llength $r] : [lrange $r 0 3]"
}
