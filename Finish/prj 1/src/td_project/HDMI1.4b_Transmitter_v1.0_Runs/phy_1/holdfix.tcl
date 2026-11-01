import_device eagle_s20.db -package EG4S20BG256
open_project {HDMI1.4b_Transmitter_v1.0.prj} -noanalyze
import_db HDMI1.4b_Transmitter_v1.0_pr.db
fix_hold
update_timing -mode final
report_timing_status -file HDMI1.4b_Transmitter_v1.0_phy.ts
report_timing_summary -file HDMI1.4b_Transmitter_v1.0_pr.timing
report_area -io_info -file HDMI1.4b_Transmitter_v1.0_phy.area
export_db HDMI1.4b_Transmitter_v1.0_pr.db
bitgen -bit HDMI1.4b_Transmitter_v1.0.bit
