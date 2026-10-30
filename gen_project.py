"""Generate the final contest TD project. RTL and constraints are unchanged."""
from pathlib import Path

HERE = Path(__file__).resolve().parent
PRJ = "media_player"
TOP = "top"

VERILOG = [
    ("rtl/sdram/enc_file/global_def.v", True),
    ("rtl/video_define.v", True),
    "rtl/ctrl/key_input.v",
    "rtl/storage/fat32_bmp_source.v",
    "rtl/storage/fat32_scaled_loader.v",
    "rtl/video/bmp_bilinear_scaler.v",
    "rtl/video/frame_write_ready.v",
    "rtl/ctrl/media_ctrl.v",
    "rtl/ctrl/seg_pages.v",
    "rtl/ctrl/seg_scan8.v",
    "rtl/ctrl/tour_ctrl.v",
    "rtl/ctrl/presentation_policy.v",
    "rtl/ctrl/power_ctrl.v",
    "rtl/ctrl/sd_recovery.v",
    "rtl/ctrl/key_actions.v",
    "rtl/ctrl/uptime_clock.v",
    "rtl/ctrl/uart_logger.v",
    "rtl/ctrl/alarm_buzzer.v",
    "rtl/video/spectrum_peak.v",
    "rtl/audio/audio_tone_pcm.v",
    "rtl/audio/alarm_tone_sel.v",
    "rtl/audio/audio_arc_calculate.v",
    "rtl/osd/video_coord.v",
    "rtl/osd/pixel_adjust.v",
    "rtl/osd/glyph_rom.v",
    "rtl/osd/text_rom.v",
    "rtl/osd/osd_overlay.v",
    "rtl/osd/subtitle_store.v",
    "rtl/osd/subtitle_commit.v",
    "rtl/hdmi/hdmi_1_4b_transmitter_core_wrapper.enc.v",
    "rtl/hdmi/lane_lvds_10_1.v",
    "rtl/hdmi/hdmi_phy_warpper.v",
    "rtl/hdmi/video_rgb_to_axis_640x480.v",
    "rtl/hdmi/startup_pulse.v",
    "rtl/color_bar.v",
    "rtl/video/frame_fifo_read.v",
    "rtl/video/frame_fifo_write.v",
    "rtl/frame_read_write.v",
    "rtl/sd_card/sd_card_cmd.v",
    "rtl/sd_card/sd_card_sec_read_write.v",
    "rtl/sd_card/sd_card_top.v",
    "rtl/sd_card/spi_master.v",
    "rtl/sdram/enc_file/sdr_as_ram.enc.v",
    "rtl/sdram/enc_file/sdr_init_ref.enc.v",
    "rtl/sdram/enc_file/sdr_wrrd.enc.v",
    "rtl/sdram/sdram.v",
    "rtl/video_delay.v",
    "rtl/video_timing_data.v",
    "rtl/video/frame_transition.v",
    "rtl/storage/sd_sector_arbiter3.v",
    "rtl/storage/subtitle_loader.v",
    "rtl/storage/fat32_guide_reader.v",
    "rtl/audio/pcm_bank_buffer.v",
    "rtl/audio/guide_audio_output.v",
    "rtl/audio/spectrum_analyzer.v",
    "rtl/video/spectrum_overlay.v",
    "rtl/reset_sync.v",
    "rtl/video/image_metadata.v",
    "rtl/video/metadata_cdc.v",
    "rtl/video/frame_status.v",
    "rtl/osd/runtime_info_overlay.v",
    "rtl/osd/info_font.v",
    "rtl/top.v",
]
IPC = [
    "ip/sys_pll.ipc",
    "ip/video_pll.ipc",
    "ip/afifo_16_32_256.ipc",
    "ip/afifo_32_16_256.ipc",
]
ADC = ["constraints/top.adc"]
SDC = ["constraints/top.sdc"]


def file_entry(path, order, belong="design_1", global_inc=False):
    attrs = ""
    if global_inc:
        attrs += '                    <Attr Name="GlobalIncluded" Val="true"/>\n'
    attrs += (
        '                    <Attr Name="UsedInSyn" Val="true"/>\n'
        '                    <Attr Name="UsedInP&amp;R" Val="true"/>\n'
        f'                    <Attr Name="BelongTo" Val="{belong}"/>\n'
        f'                    <Attr Name="CompileOrder" Val="{order}"/>\n'
    )
    return f'            <File Path="{path}">\n                <FileInfo>\n{attrs}                </FileInfo>\n            </File>\n'


def main():
    order = 1
    verilog_xml = ""
    for item in VERILOG:
        path, global_inc = item if isinstance(item, tuple) else (item, False)
        verilog_xml += file_entry(path, order, global_inc=global_inc)
        order += 1
    ip_xml = ""
    for path in IPC:
        ip_xml += file_entry(path, order)
        order += 1
    adc_xml = "".join(file_entry(path, i + 1, "constraint_1") for i, path in enumerate(ADC))
    sdc_xml = "".join(file_entry(path, i + 1, "constraint_1") for i, path in enumerate(SDC, start=len(ADC)))
    xml = f'''<?xml version="1.0" encoding="UTF-8"?>
<Project Version="3" Minor="2">
    <TD_Version>6.2.168116</TD_Version><Name>{PRJ}</Name>
    <HardWare><Family>EG4</Family><Device>EG4S20BG256</Device><Speed></Speed></HardWare>
    <Source_Files><Verilog>
{verilog_xml}        </Verilog><VHDL></VHDL><ADC_FILE>
{adc_xml}        </ADC_FILE><SDC_FILE>
{sdc_xml}        </SDC_FILE><IP_FILE>
{ip_xml}        </IP_FILE></Source_Files>
    <FileSets><FileSet Name="design_1" Type="DesignFiles"></FileSet><FileSet Name="constraint_1" Type="ConstrainFiles"></FileSet></FileSets>
    <TOP_MODULE><LABEL>{TOP}</LABEL><MODULE>{TOP}</MODULE><CREATEINDEX>user</CREATEINDEX></TOP_MODULE>
    <Property></Property><Device_Settings></Device_Settings><Configurations></Configurations>
    <Runs><Run Name="syn_1" Type="Synthesis" ConstraintSet="constraint_1" Description="" Active="true"><Strategy Name="Default_Synthesis_Strategy"></Strategy><UserParams></UserParams></Run><Run Name="phy_1" Type="PhysicalDesign" ConstraintSet="constraint_1" Description="" SynRun="syn_1" Active="true"><Strategy Name="Default_PhysicalDesign_Strategy"></Strategy><UserParams></UserParams></Run></Runs>
    <Project_Settings></Project_Settings>
</Project>
'''
    (HERE / f"{PRJ}.al").write_text(xml, encoding="utf-8")
    entries = [x[0] if isinstance(x, tuple) else x for x in VERILOG] + IPC + ADC + SDC
    missing = [path for path in entries if not (HERE / path).exists()]
    print(f"wrote {PRJ}.al; missing files: {missing if missing else 'none'}")


if __name__ == "__main__":
    main()
