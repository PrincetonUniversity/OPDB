# In-order to use this constraint file, you will need to set an environment
# variable called FO4 that is equal to the fan-out-of-4 value for the process.

create_clock [get_ports clk]  -period 100*$env(FO4)  -waveform {0 50*$env(FO4)}
set_input_delay  -clock clk -max 85*$env(FO4) [all_inputs]
set_input_delay  -clock clk -min 67*$env(FO4) [all_inputs]
set_output_delay -clock clk -max 85*$env(FO4) [all_outputs]
set_output_delay -clock clk -min 67*$env(FO4) [all_outputs]
