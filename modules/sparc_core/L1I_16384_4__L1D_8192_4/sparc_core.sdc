# In-order to use this constraint file, you will need to set an environment
# variable called FO4 that is equal to the fan-out-of-4 value for the process.

create_clock [get_ports gclk]  -period 100*$env(FO4)  -waveform {0 50*$env(FO4)}
set_input_delay  -clock gclk -max 30*$env(FO4) [all_inputs]
set_input_delay  -clock gclk -min 22*$env(FO4) [all_inputs]
set_output_delay -clock gclk -max 70*$env(FO4) [all_outputs]
set_output_delay -clock gclk -min 62*$env(FO4) [all_outputs]
