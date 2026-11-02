create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 4.167 -waveform {0.000 2.083} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]

rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]


set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] \
    -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

# Luminance bundled-data CDC only; do NOT cut whole pixel/AE clock groups.
# Data is held one frame. Capture follows three destination sync stages.
set_false_path -from [get_regs -hier {u_ae_luma_sampler/O_luma_toggle*}] -to [get_regs -hier {u_ae_set/S_luma_toggle_meta*}]
set_max_delay -datapath_only 40.0 -from [get_regs -hier {u_ae_luma_sampler/O_luma_sum*}] -to [get_regs -hier {u_ae_set/S_luma_sum_d1*}]
set_min_delay 0.0 -from [get_regs -hier {u_ae_luma_sampler/O_luma_sum*}] -to [get_regs -hier {u_ae_set/S_luma_sum_d1*}]

# Debounced mechanical follow-enable enters the intentional two-flop pixel
# synchronizer. Cut ONLY source -> first stage, not the complete clock pair.
# Exact names are important: wildcard collections can include packed aliases.
# The first -> second stage and all controller paths remain normally timed.
set_false_path -from [get_regs {S_sw2_stable}] -to [get_regs {S_servo_enable_1d}]
