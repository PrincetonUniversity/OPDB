// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 *   Description:
 *      This module implements a 16-bit counter to be used by Execution
 *      Drafting
 */

module sparc_ifu_esl_counter
#(
    parameter   COUNT_BIT_WIDTH = 16
)
(
    input                               clk,
    input                               rst_n,

    // Counter control
    input                               step,
    input                               clear,
    input                               set,

    // Counter output
    output reg [COUNT_BIT_WIDTH-1:0]    count_f
);

    //
    // Signal Declarations
    //

    // Counter next state
    reg [COUNT_BIT_WIDTH-1:0]          count_next;

    //
    // Sequential logic
    //
    
    // State flip-flops
    always @ (posedge clk)
    begin
        if (~rst_n)
            count_f <= {COUNT_BIT_WIDTH{1'b0}};
        else
            count_f <= count_next;
    end

    //
    // Combinational logic
    //

    always @ *
    begin
        count_next = count_f;
        if (clear)
            count_next = {COUNT_BIT_WIDTH{1'b0}};
        else if (set)
            count_next = {{(COUNT_BIT_WIDTH-1){1'b0}}, 1'b1};
        else if (step)
            count_next = count_f + {{(COUNT_BIT_WIDTH-1){1'b0}}, 1'b1};
    end

endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 *  Description:
 *      This module implements the portions of the Execution Drafting FSM
 *      that are specific to HTSM.
 */

module sparc_ifu_esl_htsm
#(
    parameter DIVERGED = 0,
    parameter DIVERGED_DRAFT = 1,
    parameter CONVERGED = 2,
    parameter CONVERGED_DRAFT = 3,
    parameter DIVERGED_DIFFERENT_CODE_RCFG0 = 4,
    parameter RCFG1 = 5,
    parameter RCFG2 = 6,
    parameter RCFG3 = 7
)
(
    input               clk,
    input               rst_n,

    // Trigger for condition where this module
    // makes the thread select decision
    input               esl_htsm_trigger,
   
    // Current state of ESL FSM
    input [2:0]         esl_state_f,

    // Thread ready signals
    input [3:0]         swl_esl_thr_sprdy_or_urdy,

    // Instructions valid in S
    input [3:0]         fcl_esl_tinst_vld_s,

    // Pseudo-random bit
    input               pseudo_rand_f,

    // Difference in VA PCs lessthan the threshold
    input               esl_pc_va_diff_lt_thresh_s,

    // The counter reached the configured timeout value
    input               esl_counter_timeout,

    // The LSB of the shift register
    input               esl_shiftreg_lsb,

    // Thread with minimum PC
    input [3:0]         esl_min_pc_va_s,

    // VA PCs equal (including offset)
    input               esl_pcs_va_eql_s,

    // Thread in the F stage
    input [3:0]         fcl_esl_thr_f,

    // Information about thread instructions
    input               esl_tirs_eql_s,
    input               esl_ops_eql_s,
    input               esl_pcs_pa_eql_s,

    // Unresolved branch in pipe
    input               esl_unresolved_br,

    // Branch or trap PC in bf or f
    input               esl_brtrp_target_pc_bf_f,

    // Outputs to core and ED FSM
    output reg [2:0]    esl_htsm_state_next,
    output reg [3:0]    esl_htsm_fcl_nextthr_bf,
    output reg          esl_htsm_fcl_switch_bf,
    output reg          esl_htsm_fcl_ntr_s,
    output reg          esl_htsm_lfsr_step,
    output reg          esl_htsm_timeout_counter_step,
    output reg          esl_htsm_timeout_counter_clear,
    output reg          esl_htsm_timeout_counter_set,
    output reg          esl_htsm_pc_va_diff_offset_we,
    output reg          esl_htsm_pc_va_diff_offset_clear,
    output reg          esl_htsm_timeout_shiftreg_step,
    output reg          esl_htsm_timeout_shiftreg_set,
    output reg          esl_htsm_timeout_counter_cmp_config
);

    //
    // Signal Declarations
    //
    
    // Register to keep track of which thread we are issuing
    // when sliding thread past each other
    reg esl_htsm_sliding_thread_f;
    reg esl_htsm_sliding_thread_next;

    //
    // Sequential Logic
    //
    
    // Flip-flop to keep track of which thread is sliding
    always @ (posedge clk)
    begin
        if (~rst_n)
            esl_htsm_sliding_thread_f <= 1'b0;
        else
            esl_htsm_sliding_thread_f <= esl_htsm_sliding_thread_next;
    end
    

    //
    // Combinational Logic
    //

    always @ *
    begin
        // LFSR should always be running
        esl_htsm_lfsr_step = 1'b1;

        // Default is to clear the counter
        esl_htsm_timeout_counter_step = 1'b0;
        esl_htsm_timeout_counter_clear = 1'b1;
        esl_htsm_timeout_counter_set = 1'b0;

        // Default is not to set the offset register
        esl_htsm_pc_va_diff_offset_we = 1'b0;
        esl_htsm_pc_va_diff_offset_clear = 1'b0;

        // Default is to set the timeout shift register
        esl_htsm_timeout_shiftreg_step = 1'b0;
        esl_htsm_timeout_shiftreg_set = 1'b1;

        // Default is to keep the same sliding thread
        esl_htsm_sliding_thread_next = esl_htsm_sliding_thread_f;

        // Default to comparing with configured timeout
        esl_htsm_timeout_counter_cmp_config = 1'b1;

        // Look for trigger in diverged state
        if (esl_htsm_trigger && (esl_state_f == DIVERGED))
        begin
            // Always compare to configured timeout when in diverged
            esl_htsm_timeout_counter_cmp_config = 1'b1;
            // If at least one thread is not ready, we will just wait
            // TODO: We may want to change this
            if (!(swl_esl_thr_sprdy_or_urdy[0] && swl_esl_thr_sprdy_or_urdy[1]))
            begin
                esl_htsm_state_next = DIVERGED;
                esl_htsm_fcl_nextthr_bf = 4'b0000;
                esl_htsm_fcl_switch_bf = 1'b0;
                esl_htsm_fcl_ntr_s = 1'b1; // Note: this switches out any running threads

                // Leave counter alone
                esl_htsm_timeout_counter_step = 1'b0;
                esl_htsm_timeout_counter_clear = 1'b0;
                esl_htsm_timeout_counter_set = 1'b0;

                // Leave the shift register alone
                esl_htsm_timeout_shiftreg_step = 1'b0;
                esl_htsm_timeout_shiftreg_set = 1'b0;

                // Keep same sliding thread
                esl_htsm_sliding_thread_next = esl_htsm_sliding_thread_f;
            end
            // Otherwise, both threads are ready
            else
            begin
                // If both instructions are not ready, we will just wait
                // TODO: We may want to change this
                if (!(fcl_esl_tinst_vld_s[0] && fcl_esl_tinst_vld_s[1]))
                begin
                    esl_htsm_state_next = DIVERGED;
                    // If both instructions are not ready, alternate fetching threads
                    if (!fcl_esl_tinst_vld_s[0] && !fcl_esl_tinst_vld_s[1])
                    begin
                        if (fcl_esl_thr_f == 4'b0001)
                            esl_htsm_fcl_nextthr_bf = 4'b0010;
                        else
                            esl_htsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr0 instruction is not ready, select it
                    else if(!fcl_esl_tinst_vld_s[0])
                    begin
                        esl_htsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr1 instruction is not ready, select it
                    else if (!fcl_esl_tinst_vld_s[1])
                    begin
                        esl_htsm_fcl_nextthr_bf = 4'b0010;
                    end
                    else
                    begin
                        esl_htsm_fcl_nextthr_bf = 4'bxxxx;
                    end
                    esl_htsm_fcl_switch_bf = 1'b1;
                    esl_htsm_fcl_ntr_s = 1'b1;

                    // Leave counter alone
                    esl_htsm_timeout_counter_step = 1'b0;
                    esl_htsm_timeout_counter_clear = 1'b0;
                    esl_htsm_timeout_counter_set = 1'b0;

                    // Leave the shift register alone
                    esl_htsm_timeout_shiftreg_step = 1'b0;
                    esl_htsm_timeout_shiftreg_set = 1'b0;

                    // Keep same sliding thread
                    esl_htsm_sliding_thread_next = esl_htsm_sliding_thread_f;
                end
                // Otherwise, both threads are ready and both instructions are ready                     
                else
                begin
                    // Check if instructions match or opcodes match
                    if (esl_tirs_eql_s || esl_ops_eql_s)
                    begin
                        // If phys address of PCs are equal,
                        // we have converged
                        if (esl_pcs_pa_eql_s && !esl_unresolved_br && !esl_brtrp_target_pc_bf_f)
                            esl_htsm_state_next = CONVERGED_DRAFT;
                        // Otherwise, we will just draft these
                        // instructions and come back to diverged
                        else
                            esl_htsm_state_next = DIVERGED_DRAFT; 

                        // Select thr0 for next cycle
                        esl_htsm_fcl_nextthr_bf = 4'b0001;
                        esl_htsm_fcl_switch_bf = 1'b1;
                        esl_htsm_fcl_ntr_s = 1'b1;

                        // Clear counter
                        esl_htsm_timeout_counter_step = 1'b0;
                        esl_htsm_timeout_counter_clear = 1'b1;
                        esl_htsm_timeout_counter_set = 1'b0;

                        // Set the shift register back to 1
                        esl_htsm_timeout_shiftreg_step = 1'b0;
                        esl_htsm_timeout_shiftreg_set = 1'b1;

                        // Thread 0 becomes sliding thread
                        esl_htsm_sliding_thread_next = 1'b0;
                    end
                    else
                    begin
                        // If the VA PCs + offset are equal, we must go to diverged
                        // different code, issuing thread0, and begin sliding threads
                        // past each other
                        if (esl_pcs_va_eql_s)
                        begin
                            // Go to diverged different code
                            esl_htsm_state_next = DIVERGED_DIFFERENT_CODE_RCFG0;

                            // Select thread 0
                            esl_htsm_fcl_nextthr_bf = 4'b0001;
                            
                            // Executed one for thread0 which is where we need
                            // to start, so we will reset this counter here to
                            // start counting for thread 1
                            esl_htsm_timeout_counter_step = 1'b0;
                            esl_htsm_timeout_counter_clear = 1'b0;
                            esl_htsm_timeout_counter_set = 1'b1;
                            
                            // Shift the shift register
                            esl_htsm_timeout_shiftreg_step = 1'b1;
                            esl_htsm_timeout_shiftreg_set = 1'b0;

                            // Thread 1 is now the sliding thread
                            esl_htsm_sliding_thread_next = 1'b1; 
                        end
                        // Otherwise, do the same as STSM except choose random thread
                        // when beyond the threshold
                        else
                        begin
                            // Always stay in diverged state
                            esl_htsm_state_next = DIVERGED;
                            // Select thread with the minimum VA PC if the difference is less
                            // than a threshold, otherwise alternate
                            if (esl_pc_va_diff_lt_thresh_s)
                            begin
                                // If we have hit the timeout in selecting the minimum PC,
                                // select the maximum PC
                                if (esl_counter_timeout)
                                begin
                                    if (esl_min_pc_va_s == 4'b0001)
                                        esl_htsm_fcl_nextthr_bf = 4'b0010;
                                    else
                                        esl_htsm_fcl_nextthr_bf = 4'b0001;

                                    // Clear the timer
                                    esl_htsm_timeout_counter_step = 1'b0;
                                    esl_htsm_timeout_counter_clear = 1'b1;
                                    esl_htsm_timeout_counter_set = 1'b0;
                                end
                                // Otherwise, select minimum PC and increment timeout
                                else
                                begin
                                    esl_htsm_fcl_nextthr_bf = esl_min_pc_va_s;

                                    // Increment counter
                                    esl_htsm_timeout_counter_step = 1'b1;
                                    esl_htsm_timeout_counter_clear = 1'b0;
                                    esl_htsm_timeout_counter_set = 1'b0;
                                end
                            end
                            else
                            begin
                                // Select random thread
                                if (pseudo_rand_f)
                                    esl_htsm_fcl_nextthr_bf = 4'b0010;
                                else
                                    esl_htsm_fcl_nextthr_bf = 4'b0001;
 
                                // Clear the timer
                                esl_htsm_timeout_counter_step = 1'b0;
                                esl_htsm_timeout_counter_clear = 1'b1;
                                esl_htsm_timeout_counter_set = 1'b0;
                            end
                        end
                        esl_htsm_fcl_switch_bf = 1'b1;
                        esl_htsm_fcl_ntr_s = 1'b1;

                        // Reset the shiftreg
                        esl_htsm_timeout_shiftreg_step = 1'b0;
                        esl_htsm_timeout_shiftreg_set = 1'b1;

                        // Keep same sliding thread
                        esl_htsm_sliding_thread_next = esl_htsm_sliding_thread_f;
                    end
                end
            end
        end
        else if (esl_htsm_trigger && (esl_state_f == DIVERGED_DIFFERENT_CODE_RCFG0))
        begin
            // Always compare to shift register in this state
            esl_htsm_timeout_counter_cmp_config = 1'b0;
            // If at least one thread is not ready, we will just wait
            // TODO: We may want to change this
            if (!(swl_esl_thr_sprdy_or_urdy[0] && swl_esl_thr_sprdy_or_urdy[1]))
            begin
                esl_htsm_state_next = DIVERGED_DIFFERENT_CODE_RCFG0;
                esl_htsm_fcl_nextthr_bf = 4'b0000;
                esl_htsm_fcl_switch_bf = 1'b0;
                esl_htsm_fcl_ntr_s = 1'b1; // Note: this switches out any running threads

                // Leave the timeout counter alone
                esl_htsm_timeout_counter_step = 1'b0;
                esl_htsm_timeout_counter_clear = 1'b0;
                esl_htsm_timeout_counter_set = 1'b0;

                // Leave the shift register alone
                esl_htsm_timeout_shiftreg_step = 1'b0;
                esl_htsm_timeout_shiftreg_set = 1'b0;

                // Keep same sliding thread
                esl_htsm_sliding_thread_next = esl_htsm_sliding_thread_f;

                // Do not set offset
                esl_htsm_pc_va_diff_offset_we = 1'b0;
                esl_htsm_pc_va_diff_offset_clear = 1'b0;
            end
            // Otherwise, both threads are ready
            else
            begin 
                // If both instructions are not ready, we will just wait
                // TODO: We may want to change this
                if (!(fcl_esl_tinst_vld_s[0] && fcl_esl_tinst_vld_s[1]))
                begin
                    esl_htsm_state_next = DIVERGED_DIFFERENT_CODE_RCFG0;
                    // If both instructions are not ready, alternate fetching threads
                    if (!fcl_esl_tinst_vld_s[0] && !fcl_esl_tinst_vld_s[1])
                    begin
                        if (fcl_esl_thr_f == 4'b0001)
                            esl_htsm_fcl_nextthr_bf = 4'b0010;
                        else
                            esl_htsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr0 instruction is not ready, select it
                    else if(!fcl_esl_tinst_vld_s[0])
                    begin
                        esl_htsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr1 instruction is not ready, select it
                    else if (!fcl_esl_tinst_vld_s[1])
                    begin
                        esl_htsm_fcl_nextthr_bf = 4'b0010;
                    end
                    else
                    begin
                        esl_htsm_fcl_nextthr_bf = 4'bxxxx;
                    end
                    esl_htsm_fcl_switch_bf = 1'b1;
                    esl_htsm_fcl_ntr_s = 1'b1;

                    // Leave the timeout counter alone
                    esl_htsm_timeout_counter_step = 1'b0;
                    esl_htsm_timeout_counter_clear = 1'b0;
                    esl_htsm_timeout_counter_set = 1'b0;

                    // Leave the shift register alone
                    esl_htsm_timeout_shiftreg_step = 1'b0;
                    esl_htsm_timeout_shiftreg_set = 1'b0;

                    // Keep same sliding thread
                    esl_htsm_sliding_thread_next = esl_htsm_sliding_thread_f;

                    // Do not set offset
                    esl_htsm_pc_va_diff_offset_we = 1'b0;
                    esl_htsm_pc_va_diff_offset_clear = 1'b0;
                end
                // Otherwise, both threads are ready and both instructions are ready                     
                else
                begin
                    // Check if instructions match or opcodes match
                    if (esl_tirs_eql_s || esl_ops_eql_s)
                    begin
                        // If phys address of PCs are equal,
                        // we have converged, update offset
                        if (esl_pcs_pa_eql_s && !esl_unresolved_br && !esl_brtrp_target_pc_bf_f)
                        begin
                            esl_htsm_state_next = CONVERGED_DRAFT;

                            // Update offset to current difference in VA's
                            esl_htsm_pc_va_diff_offset_we = 1'b1;
                            esl_htsm_pc_va_diff_offset_clear = 1'b0;
                        end
                        // Otherwise, we will just draft these
                        // instructions and come back to diverged
                        else
                        begin
                            esl_htsm_state_next = DIVERGED_DRAFT;
    
                            // Do not update offset
                            esl_htsm_pc_va_diff_offset_we = 1'b0;
                            esl_htsm_pc_va_diff_offset_clear = 1'b0;
                        end

                        // Select thr0 for next cycle
                        esl_htsm_fcl_nextthr_bf = 4'b0001;
                        esl_htsm_fcl_switch_bf = 1'b1;
                        esl_htsm_fcl_ntr_s = 1'b1;

                        // Clear the timeout counter
                        esl_htsm_timeout_counter_step = 1'b0;
                        esl_htsm_timeout_counter_clear = 1'b1;
                        esl_htsm_timeout_counter_set = 1'b0;

                        // Reset the shift register
                        esl_htsm_timeout_shiftreg_step = 1'b0;
                        esl_htsm_timeout_shiftreg_set = 1'b1;

                        // Sliding thread is back to thread 0
                        esl_htsm_sliding_thread_next = 1'b0;
                    end
                    else
                    begin
                        // The LSB of the shift register should only
                        // ever be 1 after the shift register rolls
                        // around and not when we first enter this state
                        // (shiftreg should be 16'b2 entering this state
                        // for the first time). In the case the shiftreg
                        // rolls around, we declare diverged
                        if (esl_shiftreg_lsb)
                        begin
                            esl_htsm_state_next = DIVERGED;
                                
                            // Issue the current sliding thread
                            if (esl_htsm_sliding_thread_f)
                                esl_htsm_fcl_nextthr_bf = 4'b0010;
                            else
                                esl_htsm_fcl_nextthr_bf = 4'b0001;
                            esl_htsm_fcl_switch_bf = 1'b1;
                            esl_htsm_fcl_ntr_s = 1'b1;

                            // Clear the timeout counter
                            esl_htsm_timeout_counter_step = 1'b0;
                            esl_htsm_timeout_counter_clear = 1'b1;
                            esl_htsm_timeout_counter_set = 1'b0;

                            // Reset the shift register
                            esl_htsm_timeout_shiftreg_step = 1'b0;
                            esl_htsm_timeout_shiftreg_set = 1'b1;

                            // Sliding thread is back to thread 0
                            esl_htsm_sliding_thread_next = 1'b0;
                        end
                        // Check if the counter reached the shift register value
                        else if (esl_counter_timeout)
                        begin
                            esl_htsm_state_next = DIVERGED_DIFFERENT_CODE_RCFG0;
                            
                            // If so, issue the sliding thread,
                            // reset shift reg counter, shift
                            // the shift register, and invert
                            // sliding thread
                            if (esl_htsm_sliding_thread_f)
                                esl_htsm_fcl_nextthr_bf = 4'b0010;
                            else
                                esl_htsm_fcl_nextthr_bf = 4'b0001;
                            esl_htsm_fcl_switch_bf = 1'b1;
                            esl_htsm_fcl_ntr_s = 1'b1;

                            // Set the timeout counter
                            esl_htsm_timeout_counter_step = 1'b0;
                            esl_htsm_timeout_counter_clear = 1'b0;
                            esl_htsm_timeout_counter_set = 1'b1;

                            // Step the shift register
                            esl_htsm_timeout_shiftreg_step = 1'b1;
                            esl_htsm_timeout_shiftreg_set = 1'b1;

                            // Invert sliding thread
                            esl_htsm_sliding_thread_next = ~esl_htsm_sliding_thread_f;
                        end
                        else
                        begin
                            esl_htsm_state_next = DIVERGED_DIFFERENT_CODE_RCFG0;
                            
                            // If not, issue the sliding thread and
                            // increment counter
                            if (esl_htsm_sliding_thread_f)
                                esl_htsm_fcl_nextthr_bf = 4'b0010;
                            else
                                esl_htsm_fcl_nextthr_bf = 4'b0001;
                            esl_htsm_fcl_switch_bf = 1'b1;
                            esl_htsm_fcl_ntr_s = 1'b1;
                           
                            // Step the timeout counter
                            esl_htsm_timeout_counter_step = 1'b1;
                            esl_htsm_timeout_counter_clear = 1'b0;
                            esl_htsm_timeout_counter_set = 1'b0;
 
                            // Leave the shift register alone
                            esl_htsm_timeout_shiftreg_step = 1'b0;
                            esl_htsm_timeout_shiftreg_set = 1'b0;

                            // Same sliding thread
                            esl_htsm_sliding_thread_next = esl_htsm_sliding_thread_f;
                        end

                        // Do not set the offset register
                        esl_htsm_pc_va_diff_offset_we = 1'b0; 
                        esl_htsm_pc_va_diff_offset_clear = 1'b0;
                    end
                end
            end
        end
        else if (esl_htsm_trigger && (esl_state_f == RCFG1))
        begin
            // Try to recover
            esl_htsm_state_next = DIVERGED;
            esl_htsm_fcl_nextthr_bf = 4'b0000;
            esl_htsm_fcl_switch_bf = 1'b0;
            esl_htsm_fcl_ntr_s = 1'b0;
        end
        else if (esl_htsm_trigger && (esl_state_f == RCFG2))
        begin
            // Try to recover
            esl_htsm_state_next = DIVERGED;
            esl_htsm_fcl_nextthr_bf = 4'b0000;
            esl_htsm_fcl_switch_bf = 1'b0;
            esl_htsm_fcl_ntr_s = 1'b0;
        end
        else if (esl_htsm_trigger && (esl_state_f == RCFG3))
        begin
            // Try to recover
            esl_htsm_state_next = DIVERGED;
            esl_htsm_fcl_nextthr_bf = 4'b0000;
            esl_htsm_fcl_switch_bf = 1'b0;
            esl_htsm_fcl_ntr_s = 1'b0;
        end
        else
        begin
            esl_htsm_state_next = 3'bxx;
            esl_htsm_fcl_nextthr_bf = 4'bxxxx;
            esl_htsm_fcl_switch_bf = 1'bx;
            esl_htsm_fcl_ntr_s = 1'bx;
        end
    end

endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
//  Description:
//      An LFSR for the Execution Drafting random thread synchronization method.
//      The LFSR provides pseudo-randomness in selecting a random thread to execute.
//      It is a 16-bit LFSR with the following polynomial:
//          x^16 + x^14 + x^13 + x^11 + 1
*/

module sparc_ifu_esl_lfsr 
(
    input               clk,
    input               rst_n,

    // Seeding the LFSR
    input [15:0]        lfsr_seed,
    input               lfsr_ld,

    // Enable signal for LFSR
    input               lfsr_step,

    // LFSR state output
    output reg [15:0]   lfsr_state_f,

    // LFSR bit stream output
    output reg          lfsr_out_f
);

    //
    // Signal Declarations
    //
    
    // LFSR next state
    reg [15:0]          lfsr_state_next;

    //
    // Sequential logic
    //

    // State flip-flops
    always @ (posedge clk)
    begin
        if (~rst_n)
            lfsr_state_f <= 16'b0;
        else
            lfsr_state_f <= lfsr_state_next;
    end
    
    //
    // Combinational Logic
    // 
    
    always @ *
    begin
        // Next state logic
        lfsr_state_next = lfsr_state_f;
        if (rst_n && lfsr_ld)
            lfsr_state_next = lfsr_seed;
        else if (rst_n && lfsr_step)
            lfsr_state_next = {lfsr_state_f[14:0], 
                               lfsr_state_f[15] ^ lfsr_state_f[13] ^ lfsr_state_f[12] ^ lfsr_state_f[10]};

        // Output bitstream comes from the 16th state bit
        lfsr_out_f = lfsr_state_f[15];
    end

endmodule // sparc_esl_lfsr
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 *  Description:
 *      This module implements the portions of the Execution Drafting FSM
 *      that are specific to RTSM.
 */

module sparc_ifu_esl_rtsm
#(
    parameter DIVERGED = 0,
    parameter DIVERGED_DRAFT = 1,
    parameter CONVERGED = 2,
    parameter CONVERGED_DRAFT = 3,
    parameter DIVERGED_DIFFERENT_CODE_RCFG0 = 4,
    parameter RCFG1 = 5,
    parameter RCFG2 = 6,
    parameter RCFG3 = 7
)
(
    // Trigger for condition where this module
    // makes the thread select decision
    input               esl_rtsm_trigger,

    // Current state of ESL FSM
    input [2:0]         esl_state_f,

    // Thread ready signals
    input [3:0]         swl_esl_thr_sprdy_or_urdy,

    // Instructions valid in S
    input [3:0]         fcl_esl_tinst_vld_s, 

    // Pseudo-random bit
    input               pseudo_rand_f,
  
    // Thread in the F stage
    input [3:0]         fcl_esl_thr_f,
 
    // Information about thread instructions
    input               esl_tirs_eql_s,
    input               esl_ops_eql_s,
    input               esl_pcs_pa_eql_s,

    // Unresolved branch in pipe
    input               esl_unresolved_br,

    // Branch or trap PC in bf or f
    input               esl_brtrp_target_pc_bf_f,
 
    // Outputs to core and ED FSM
    output reg [2:0]    esl_rtsm_state_next,   
    output reg [3:0]    esl_rtsm_fcl_nextthr_bf,
    output reg          esl_rtsm_fcl_switch_bf,
    output reg          esl_rtsm_fcl_ntr_s,
    output reg          esl_rtsm_lfsr_step,
    output reg          esl_rtsm_timeout_counter_step,
    output reg          esl_rtsm_timeout_counter_clear,
    output reg          esl_rtsm_timeout_counter_set,
    output reg          esl_rtsm_pc_va_diff_offset_we,
    output reg          esl_rtsm_pc_va_diff_offset_clear,
    output reg          esl_rtsm_timeout_shiftreg_step,
    output reg          esl_rtsm_timeout_shiftreg_set,
    output reg          esl_rtsm_timeout_counter_cmp_config
);

    //
    // Signal Declarations
    //

    //
    // Sequential Logic
    //
    
    //
    // Combinational Logic
    //

    always @ *
    begin
        // LFSR should always be running
        esl_rtsm_lfsr_step = 1'b1;

        // Counter should never be running (always clear)
        esl_rtsm_timeout_counter_step = 1'b0;
        esl_rtsm_timeout_counter_clear = 1'b1; 
        esl_rtsm_timeout_counter_set = 1'b0;

        // Never use offset register
        esl_rtsm_pc_va_diff_offset_we = 1'b0;
        esl_rtsm_pc_va_diff_offset_clear = 1'b1;

        // Never use shift register
        esl_rtsm_timeout_shiftreg_step = 1'b0;
        esl_rtsm_timeout_shiftreg_set = 1'b1;

        // Doesn't really matter what we set this to
        // as we never check whether counter times out
        esl_rtsm_timeout_counter_cmp_config = 1'b1;

        // Look for trigger in diverged state
        if (esl_rtsm_trigger && (esl_state_f == DIVERGED))
        begin
            // If at least one thread is not ready, we will just wait
            // TODO: We may want to change this
            if (!(swl_esl_thr_sprdy_or_urdy[0] && swl_esl_thr_sprdy_or_urdy[1]))
            begin
                esl_rtsm_state_next = DIVERGED;
                esl_rtsm_fcl_nextthr_bf = 4'b0000;
                esl_rtsm_fcl_switch_bf = 1'b0;
                esl_rtsm_fcl_ntr_s = 1'b1; // Note: this switches out any running threads
            end
            // Otherwise, both threads are ready
            else
            begin
                // If both instructions are not ready, we will just wait
                // TODO: We may want to change this
                if (!(fcl_esl_tinst_vld_s[0] && fcl_esl_tinst_vld_s[1]))
                begin
                    esl_rtsm_state_next = DIVERGED;
                    // If both instructions are not ready, alternate fetching threads
                    if (!fcl_esl_tinst_vld_s[0] && !fcl_esl_tinst_vld_s[1])
                    begin
                        if (fcl_esl_thr_f == 4'b0001)
                            esl_rtsm_fcl_nextthr_bf = 4'b0010;
                        else
                            esl_rtsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr0 instruction is not ready, select it
                    else if(!fcl_esl_tinst_vld_s[0])
                    begin
                        esl_rtsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr1 instruction is not ready, select it
                    else if (!fcl_esl_tinst_vld_s[1])
                    begin
                        esl_rtsm_fcl_nextthr_bf = 4'b0010;
                    end
                    esl_rtsm_fcl_switch_bf = 1'b1;
                    esl_rtsm_fcl_ntr_s = 1'b1;
                end
                // Otherwise, both threads are ready and both instructions are ready                     
                else
                begin
                    // Check if instructions match or opcodes match
                    if (esl_tirs_eql_s || esl_ops_eql_s)
                    begin
                        // If phys address of PCs are equal,
                        // we have converged
                        if (esl_pcs_pa_eql_s && !esl_unresolved_br && !esl_brtrp_target_pc_bf_f)
                            esl_rtsm_state_next = CONVERGED_DRAFT;
                        // Otherwise, we will just draft these
                        // instructions and come back to diverged
                        else
                            esl_rtsm_state_next = DIVERGED_DRAFT; 

                        // Select thr0 for next cycle
                        esl_rtsm_fcl_nextthr_bf = 4'b0001;
                        esl_rtsm_fcl_switch_bf = 1'b1;
                        esl_rtsm_fcl_ntr_s = 1'b1;
                    end
                    else
                    begin
                        // Always stay in diverged state
                        esl_rtsm_state_next = DIVERGED;
                        // Select random thread
                        if (pseudo_rand_f)
                            esl_rtsm_fcl_nextthr_bf = 4'b0010;
                        else
                            esl_rtsm_fcl_nextthr_bf = 4'b0001;
                        esl_rtsm_fcl_switch_bf = 1'b1;
                        esl_rtsm_fcl_ntr_s = 1'b1;
                    end
                end
            end
        end
        else if (esl_rtsm_trigger && (esl_state_f == DIVERGED_DIFFERENT_CODE_RCFG0))
        begin
            // Try to recover
            esl_rtsm_state_next = DIVERGED;
            esl_rtsm_fcl_nextthr_bf = 4'b0000;
            esl_rtsm_fcl_switch_bf = 1'b0;
            esl_rtsm_fcl_ntr_s = 1'b0; 
        end
        else if (esl_rtsm_trigger && (esl_state_f == RCFG1))
        begin
            // Try to recover
            esl_rtsm_state_next = DIVERGED;
            esl_rtsm_fcl_nextthr_bf = 4'b0000;
            esl_rtsm_fcl_switch_bf = 1'b0;
            esl_rtsm_fcl_ntr_s = 1'b0;
        end
        else if (esl_rtsm_trigger && (esl_state_f == RCFG2))
        begin
            // Try to recover
            esl_rtsm_state_next = DIVERGED;
            esl_rtsm_fcl_nextthr_bf = 4'b0000;
            esl_rtsm_fcl_switch_bf = 1'b0;
            esl_rtsm_fcl_ntr_s = 1'b0;
        end
        else if (esl_rtsm_trigger && (esl_state_f == RCFG3))
        begin
            // Try to recover
            esl_rtsm_state_next = DIVERGED;
            esl_rtsm_fcl_nextthr_bf = 4'b0000;
            esl_rtsm_fcl_switch_bf = 1'b0;
            esl_rtsm_fcl_ntr_s = 1'b0;
        end
        else
        begin
            esl_rtsm_state_next = 3'bx;
            esl_rtsm_fcl_nextthr_bf = 4'bxxxx;
            esl_rtsm_fcl_switch_bf = 1'bx;
            esl_rtsm_fcl_ntr_s = 1'bx;
        end
    end

endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 *   Description:
 *      This module implements a 49-bit shift register with wrap around
 *      to be used by Execution Drafting
 */

module sparc_ifu_esl_shiftreg
#(
    parameter SHIFT_REG_BIT_WIDTH = 16
)
(
    input                                   clk,
    input                                   rst_n,

    // Counter control
    input                                   step,
    input                                   set,

    // Counter output
    output reg [SHIFT_REG_BIT_WIDTH-1:0]    shift_reg_f
);

    //
    // Signal Declarations
    //

    // Counter next state
    reg [SHIFT_REG_BIT_WIDTH-1:0]          shift_reg_next;

    //
    // Sequential logic
    //
    
    // State flip-flops
    always @ (posedge clk)
    begin
        if (~rst_n)
            shift_reg_f <= {SHIFT_REG_BIT_WIDTH{1'b0}};
        else
            shift_reg_f <= shift_reg_next;
    end

    //
    // Combinational logic
    //

    always @ *
    begin
        shift_reg_next = shift_reg_f;
        if (set)
            shift_reg_next = {{(SHIFT_REG_BIT_WIDTH-1){1'b0}}, 1'b1};
        else if (step)
            shift_reg_next = {shift_reg_f[SHIFT_REG_BIT_WIDTH-2:0], shift_reg_f[SHIFT_REG_BIT_WIDTH-1]};
    end

endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 *  Description:
 *      This module implements the portions of the Execution Drafting FSM
 *      that are specific to STSM.
 */

module sparc_ifu_esl_stsm
#(
    parameter DIVERGED = 0,
    parameter DIVERGED_DRAFT = 1,
    parameter CONVERGED = 2,
    parameter CONVERGED_DRAFT = 3,
    parameter DIVERGED_DIFFERENT_CODE_RCFG0 = 4,
    parameter RCFG1 = 5,
    parameter RCFG2 = 6,
    parameter RCFG3 = 7
)
(
    // Trigger for condition where this module
    // makes the thread select decision
    input               esl_stsm_trigger,
   
    // Current state of ESL FSM
    input [2:0]         esl_state_f,

    // Thread ready signals
    input [3:0]         swl_esl_thr_sprdy_or_urdy,

    // Instructions valid in S
    input [3:0]         fcl_esl_tinst_vld_s,

    // Difference in VA PCs lessthan the threshold
    input               esl_pc_va_diff_lt_thresh_s,

    // The counter reached the timeout value
    input               esl_counter_timeout,

    // Thread with minimum PC
    input [3:0]         esl_min_pc_va_s,

    // Thread in the F stage
    input [3:0]         fcl_esl_thr_f,

    // Information about thread instructions
    input               esl_tirs_eql_s,
    input               esl_ops_eql_s,
    input               esl_pcs_pa_eql_s,

    // Unresolved branch in pipe
    input               esl_unresolved_br,

    // Branch or trap target PC in bf or f
    input               esl_brtrp_target_pc_bf_f,

    // Outputs to core and ED FSM
    output reg [2:0]    esl_stsm_state_next,
    output reg [3:0]    esl_stsm_fcl_nextthr_bf,
    output reg          esl_stsm_fcl_switch_bf,
    output reg          esl_stsm_fcl_ntr_s,
    output reg          esl_stsm_lfsr_step,
    output reg          esl_stsm_timeout_counter_step,
    output reg          esl_stsm_timeout_counter_clear,
    output reg          esl_stsm_timeout_counter_set,
    output reg          esl_stsm_pc_va_diff_offset_we,
    output reg          esl_stsm_pc_va_diff_offset_clear,
    output reg          esl_stsm_timeout_shiftreg_step,
    output reg          esl_stsm_timeout_shiftreg_set,
    output reg          esl_stsm_timeout_counter_cmp_config
);

    //
    // Signal Declarations
    //

    //
    // Sequential Logic
    //
    
    //
    // Combinational Logic
    //

    always @ *
    begin
        // LFSR should never be running
        esl_stsm_lfsr_step = 1'b0;

        // Default is to clear the counter
        esl_stsm_timeout_counter_step = 1'b0;
        esl_stsm_timeout_counter_clear = 1'b1;
        esl_stsm_timeout_counter_set = 1'b0;

        // Never use offset register
        esl_stsm_pc_va_diff_offset_we = 1'b0;
        esl_stsm_pc_va_diff_offset_clear = 1'b1;

        // Never use the shift register
        esl_stsm_timeout_shiftreg_step = 1'b0;
        esl_stsm_timeout_shiftreg_set = 1'b1;

        // Always compare to configured timeout
        esl_stsm_timeout_counter_cmp_config = 1'b1;

        // Look for trigger in diverged state
        if (esl_stsm_trigger && (esl_state_f == DIVERGED))
        begin
            // If at least one thread is not ready, we will just wait
            // TODO: We may want to change this
            if (!(swl_esl_thr_sprdy_or_urdy[0] && swl_esl_thr_sprdy_or_urdy[1]))
            begin
                esl_stsm_state_next = DIVERGED;
                esl_stsm_fcl_nextthr_bf = 4'b0000;
                esl_stsm_fcl_switch_bf = 1'b0;
                esl_stsm_fcl_ntr_s = 1'b1; // Note: this switches out any running threads

                // Leave counter alone
                esl_stsm_timeout_counter_step = 1'b0;
                esl_stsm_timeout_counter_clear = 1'b0;
                esl_stsm_timeout_counter_set = 1'b0;
            end
            // Otherwise, both threads are ready
            else
            begin
                // If both instructions are not ready, we will just wait
                // TODO: We may want to change this
                if (!(fcl_esl_tinst_vld_s[0] && fcl_esl_tinst_vld_s[1]))
                begin
                    esl_stsm_state_next = DIVERGED;
                    // If both instructions are not ready, alternate fetching threads
                    if (!fcl_esl_tinst_vld_s[0] && !fcl_esl_tinst_vld_s[1])
                    begin
                        if (fcl_esl_thr_f == 4'b0001)
                            esl_stsm_fcl_nextthr_bf = 4'b0010;
                        else
                            esl_stsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr0 instruction is not ready, select it
                    else if(!fcl_esl_tinst_vld_s[0])
                    begin
                        esl_stsm_fcl_nextthr_bf = 4'b0001;
                    end
                    // If only thr1 instruction is not ready, select it
                    else if (!fcl_esl_tinst_vld_s[1])
                    begin
                        esl_stsm_fcl_nextthr_bf = 4'b0010;
                    end
                    esl_stsm_fcl_switch_bf = 1'b1;
                    esl_stsm_fcl_ntr_s = 1'b1;

                    // Leave counter alone
                    esl_stsm_timeout_counter_step = 1'b0;
                    esl_stsm_timeout_counter_clear = 1'b0;
                    esl_stsm_timeout_counter_set = 1'b0;
                end
                // Otherwise, both threads are ready and both instructions are ready                     
                else
                begin
                    // Check if instructions match or opcodes match
                    if (esl_tirs_eql_s || esl_ops_eql_s)
                    begin
                        // If phys address of PCs are equal,
                        // we have converged
                        if (esl_pcs_pa_eql_s && !esl_unresolved_br && !esl_brtrp_target_pc_bf_f)
                            esl_stsm_state_next = CONVERGED_DRAFT;
                        // Otherwise, we will just draft these
                        // instructions and come back to diverged
                        else
                            esl_stsm_state_next = DIVERGED_DRAFT; 

                        // Select thr0 for next cycle
                        esl_stsm_fcl_nextthr_bf = 4'b0001;
                        esl_stsm_fcl_switch_bf = 1'b1;
                        esl_stsm_fcl_ntr_s = 1'b1;

                        // Clear counter
                        esl_stsm_timeout_counter_step = 1'b0;
                        esl_stsm_timeout_counter_clear = 1'b1;
                        esl_stsm_timeout_counter_set = 1'b0;
                    end
                    else
                    begin
                        // Always stay in diverged state
                        esl_stsm_state_next = DIVERGED;
                        // Select thread with the minimum VA PC if the difference is less
                        // than a threshold, otherwise alternate
                        if (esl_pc_va_diff_lt_thresh_s)
                        begin
                            // If we have hit the timeout in selecting the minimum PC,
                            // select the maximum PC
                            if (esl_counter_timeout)
                            begin
                                if (esl_min_pc_va_s == 4'b0001)
                                    esl_stsm_fcl_nextthr_bf = 4'b0010;
                                else
                                    esl_stsm_fcl_nextthr_bf = 4'b0001;

                                // Clear the timer
                                esl_stsm_timeout_counter_step = 1'b0;
                                esl_stsm_timeout_counter_clear = 1'b1;
                                esl_stsm_timeout_counter_set = 1'b0;
                            end
                            // Otherwise, select minimum PC and increment timeout
                            else
                            begin
                                esl_stsm_fcl_nextthr_bf = esl_min_pc_va_s;
                                // Increment counter
                                esl_stsm_timeout_counter_step = 1'b1;
                                esl_stsm_timeout_counter_clear = 1'b0;
                                esl_stsm_timeout_counter_set = 1'b0;
                            end
                        end
                        else
                        begin
                            if (fcl_esl_thr_f == 4'b0001)
                                esl_stsm_fcl_nextthr_bf = 4'b0010;
                            else
                                esl_stsm_fcl_nextthr_bf = 4'b0001;
                            
                            // Clear the timer
                            esl_stsm_timeout_counter_step = 1'b0;
                            esl_stsm_timeout_counter_clear = 1'b1;
                            esl_stsm_timeout_counter_set = 1'b0;
                        end
                        esl_stsm_fcl_switch_bf = 1'b1;
                        esl_stsm_fcl_ntr_s = 1'b1;
                    end
                end
            end     
        end
        else if (esl_stsm_trigger && (esl_state_f == DIVERGED_DIFFERENT_CODE_RCFG0))
        begin
            // Try to recover
            esl_stsm_state_next = DIVERGED;
            esl_stsm_fcl_nextthr_bf = 4'b0000;
            esl_stsm_fcl_switch_bf = 1'b0;
            esl_stsm_fcl_ntr_s = 1'b0;
        end
        else if (esl_stsm_trigger && (esl_state_f == RCFG1))
        begin
            // Try to recover
            esl_stsm_state_next = DIVERGED;
            esl_stsm_fcl_nextthr_bf = 4'b0000;
            esl_stsm_fcl_switch_bf = 1'b0;
            esl_stsm_fcl_ntr_s = 1'b0; 
        end
        else if (esl_stsm_trigger && (esl_state_f == RCFG2))
        begin
            // Try to recover
            esl_stsm_state_next = DIVERGED;
            esl_stsm_fcl_nextthr_bf = 4'b0000;
            esl_stsm_fcl_switch_bf = 1'b0;
            esl_stsm_fcl_ntr_s = 1'b0;
        end
        else if (esl_stsm_trigger && (esl_state_f == RCFG3))
        begin
            // Try to recover
            esl_stsm_state_next = DIVERGED;
            esl_stsm_fcl_nextthr_bf = 4'b0000;
            esl_stsm_fcl_switch_bf = 1'b0;
            esl_stsm_fcl_ntr_s = 1'b0;
        end
        else
        begin
            esl_stsm_state_next = 3'bxx;
            esl_stsm_fcl_nextthr_bf = 4'bxxxx;
            esl_stsm_fcl_switch_bf = 1'bx;
            esl_stsm_fcl_ntr_s = 1'bx;
        end
    end

endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 *  Description:
 *      This module implements the generic portions of the Execution Drafting
 *      FSM.  Anything common to all synchronization methods is implemented
 *      here.  
 *
 *  Note: While the interface to this module supports 4 threads, the logic
 *  only supports 2
 */

// Copyright (c) 2015 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//==================================================================================================
//  Filename      : define.h
//  Created On    : 2014-02-20
//  Last Modified : 2018-11-16 17:14:11
//  Revision      :
//  Author        : Yaosheng Fu
//  Company       : Princeton University
//  Email         : yfu@princeton.edu
//
//  Description   : main header file defining global architecture parameters
//
//
//==================================================================================================




// Uncomment to define USE_GENERIC_SRAM_IMPLEMENTATION to use the old unsynthesizable BRAM
// `define USE_GENERIC_SRAM_IMPLEMENTATION




/*
Copyright (c) 2015 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/////////////////////////////////////////////////////////////////////////////////////////////
// 63         50 49      42 41      34 33           30 29      22 21                 0   
// ------------------------------------------------------------------------------------
// |            |          |          |               |          |                    |
// |  Chip ID   |  Dest X  |  Dest Y  |  Final Route  |  Length  |    Header Payload  | 
// |            |          |          |               |          |                    |
// ------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////











 //whether the routing is based on chipid or x y position
 //`define    ROUTING_CHIP_ID
 

 //defines for different topology, only one should be active
 //`define    NETWORK_TOPO_2D_MESH
 //`define    NETWORK_TOPO_3D_MESH
 

// Tile config

// devices.xml





// NoC interface





















// NodeID decomposition








//========================
//Packet format
//=========================

//Header decomposition































// these shifted fields are added for convienience
// HEADER 2








// HEADER 3








//NoC header information










// Width of MSG_ADDR field - you're probably looking for PHY_ADDR_WIDTH


//Coherence information





//Requests from L15 to L2
// Should always make #0 an error








//condition satisfied

//condition not satisfied

//Both SWAP and LDSTUB are the same for L2









//RISC-V AMO requests









//RISC-V AMO L2-internal phase 1









//RISC-V AMO L2-internal phase 2












//Forward requests from L2 to L15







//Memory requests from L2 to DRAM






//Forward acks from L15 to L2







//Memory acks from memory to L2









//Acks from L2 to L15


//TODO



//Only exist within L2





//`define MSG_TYPE_LOAD_REQ           8'd31 if this is enabled, don't use 31





// These should be defined in l2.vh, not the global defines











//Physical address










//Transition data size












//`define HOME_ID_MASK_X          10:10
//Additional fields for Sharer Domain ID and Logical Sharer ID
//For coherence domain restriction only


// Tri: dynamically adjust these parameters based on how many tiles are available
//  Assumption: 8x8 topology























































//`define DMBR_TAG_WIDTH 4

//Clumpy Shared Memory






////////////////////////////////////////////
// SOME CONFIGURATION REGISTERS DEFINES
////////////////////////////////////////////
// example: read/write to csm_en would be 0xba_0000_0100

// `define ASI_ADDRESS_MASK    `L15_ADDR_TYPE
// `define CONFIG_ASI_ADDRESS  `L15_ADDR_TYPE_WIDTH'hba










// DMBR Config register 1 fields















// DMBR Config register 2 fields



//Home allocation method






//Additional fields for Sharer Domain ID and Logical Sharer ID
//For coherence domain restriction only

































//`define TTE_CSM_WIDTH           64
//`define TTE_CSM                 63:0
//`define TTE_CSM_VALID           63
//`define TTE_CSM_SZL             62:61
//`define TTE_CSM_NFO             60
//`define TTE_CSM_IE              59
//`define TTE_CSM_SOFT2           58:49
//`define TTE_CSM_SZH             48
//`define TTE_CSM_DIAG            47:40
//`define TTE_CSM_RES1            39
//`define TTE_CSM_SDID            38:29
//`define TTE_CSM_HDID            28:19
//`define TTE_CSM_LSID            18:13
//`define TTE_CSM_SOFT            12:8
//`define TTE_CSM_RES2            7
//`define TTE_CSM_LOCK            6
//`define TTE_CSM_CP              5
//`define TTE_CSM_CV              4
//`define TTE_CSM_E               3
//`define TTE_CSM_P               2
//`define TTE_CSM_W               1
//`define TTE_CSM_RES3            0







//`define HOME_ID_X_POS_WIDTH         3
//`define HOME_ID_X_POS               2:0
//`define HOME_ID_Y_POS_WIDTH         3
//`define HOME_ID_Y_POS               5:3

// Packet format for home id





/////////////////////////////////////
// BIST
/////////////////////////////////////

// the data width from tap to individual sram wrappers



//deprecated































/////////////////////////////////////
// IDs for JTAG-Core interface
/////////////////////////////////////

// 48b for writing the PC reset vector

// 94b for reading the sscan data











// Execution Drafting Synchronization Method Values





// Execution Drafting timeout counter bit width


// Configuration registers












// Execution Drafting configuration register bit positions








// Execution Drafting configuration register default values
// ED disabled, STSM sync method, LFSR seed = 16'b0, LFSR load = 1'b0,
// Counter Timeout = 16'd32



//Clumpy sharer memory configuration registers

























module sparc_ifu_esl_fsm
(
    input               clk,
    input               rst_n,

    // Configuration bits input
    input               config_esl_en,
    input [1:0]         config_esl_sync_method,

    // Counter timeout configuration value
    input [15:0]        config_esl_counter_timeout,

    // LFSR management signals
    input [15:0]        config_esl_lfsr_seed,
    input               config_esl_lfsr_ld,

    // Thread active signals from FSM
    input [3:0]         swl_esl_thr_active,

    // Thread ready signals from FSM, both non-speculative
    // ready and speculative ready
    input [3:0]         swl_esl_thr_urdy,
    input [3:0]         swl_esl_thr_sprdy_or_urdy,
    input [3:0]         swl_esl_thr_sprdy_or_urdy_next,

    // Whether to use speculative rdy vector, meaning
    // there is no non-speculative ready thread
    input               swl_esl_use_spec,

    // Instructions valid in S
    input [3:0]         fcl_esl_tinst_vld_s,

    // Thread in F stage
    input [3:0]         fcl_esl_thr_f,

    // Rollback on this cycle
    input               fcl_esl_rb_stg_s,

    // Information about threads in thread select stage
    input               esl_tirs_eql_s,
    input               esl_ops_eql_s,
    input               esl_pcs_pa_eql_s,
    input               esl_ctrl_flow_diverged_late_s,
    input               esl_pcs_pa_page_bndry_s,
    input               esl_any_trap_bf,
    input [3:0]         esl_min_pc_va_s,
    input               esl_pc_va_diff_lt_thresh_s,
    input               esl_pcs_va_eql_s,
    input               esl_unresolved_br,
    input               esl_brtrp_target_pc_bf_f,
    input               swl_esl_icmiss_out,

    // Synchronization output
    output reg [3:0]    esl_fcl_nextthr_bf,
    output reg          esl_fcl_stall_bf,
    output reg          esl_fcl_switch_bf,
    output reg          esl_fdp_sync_pcs_bf,
    output reg          esl_fcl_ntr_s,
    output reg          esl_fdp_issue_prev_inst_next_s,
    output reg          esl_fcl_force_running_s,
    output reg          esl_pc_va_diff_offset_we,
    output reg          esl_pc_va_diff_offset_clear
);

    //
    // Parameter Definitions
    //

    // State definitions
    localparam  DIVERGED = 0;
    localparam  DIVERGED_DRAFT = 1;
    localparam  CONVERGED = 2;
    localparam  CONVERGED_DRAFT = 3;
    localparam  DIVERGED_DIFFERENT_CODE_RCFG0 = 4;
    localparam  RCFG1 = 5;
    localparam  RCFG2 = 6;
    localparam  RCFG3 = 7;

    //
    // Signal Declarations
    //
    
    // LFSR control and output
    reg         esl_lfsr_step;
    wire        pseudo_rand_f;

    // Timeout counter control and output
    reg                                         esl_timeout_counter_step;
    reg                                         esl_timeout_counter_clear;
    reg                                         esl_timeout_counter_set;
    wire [16-1:0]    esl_timeout_counter_count_f;

    // Shift Register control and output
    reg                                         esl_timeout_shiftreg_step;
    reg                                         esl_timeout_shiftreg_set;
    wire [16-1:0]    esl_timeout_shiftreg_f;

    // Control signals for comparator for timeout control.
    // 1'b1 - Compare to configured timeout
    // 1'b0 - Compare to shift register
    reg                                         esl_timeout_counter_cmp_config;

    // Next state output decisions
    wire [2:0]  esl_stsm_state_next;
    wire [2:0]  esl_rtsm_state_next;
    wire [2:0]  esl_htsm_state_next;
    wire [2:0]  esl_rcfg_state_next;

    // Thread select output decisions
    wire [3:0]  esl_stsm_fcl_nextthr_bf;
    wire [3:0]  esl_rtsm_fcl_nextthr_bf;
    wire [3:0]  esl_htsm_fcl_nextthr_bf;
    wire [3:0]  esl_rcfg_fcl_nextthr_bf;

    // Thread switch output decisions
    wire        esl_stsm_fcl_switch_bf;
    wire        esl_rtsm_fcl_switch_bf;
    wire        esl_htsm_fcl_switch_bf;
    wire        esl_rcfg_fcl_switch_bf;

    // Thread switch out/ready output decisions
    wire        esl_stsm_fcl_ntr_s;
    wire        esl_rtsm_fcl_ntr_s;
    wire        esl_htsm_fcl_ntr_s;
    wire        esl_rcfg_fcl_ntr_s;

    // LFSR step outputs
    wire        esl_stsm_lfsr_step;
    wire        esl_rtsm_lfsr_step;
    wire        esl_htsm_lfsr_step;
    wire        esl_rcfg_lfsr_step;

    // Timeout counter step and clear outputs
    wire        esl_stsm_timeout_counter_step;
    wire        esl_stsm_timeout_counter_clear;
    wire        esl_stsm_timeout_counter_set;
    wire        esl_rtsm_timeout_counter_step;
    wire        esl_rtsm_timeout_counter_clear;
    wire        esl_rtsm_timeout_counter_set;
    wire        esl_htsm_timeout_counter_step;
    wire        esl_htsm_timeout_counter_clear;
    wire        esl_htsm_timeout_counter_set;
    wire        esl_rcfg_timeout_counter_step;
    wire        esl_rcfg_timeout_counter_clear;
    wire        esl_rcfg_timeout_counter_set;

    // Shift register step and set outputs
    wire        esl_stsm_timeout_shiftreg_step;
    wire        esl_stsm_timeout_shiftreg_set;
    wire        esl_rtsm_timeout_shiftreg_step;
    wire        esl_rtsm_timeout_shiftreg_set;
    wire        esl_htsm_timeout_shiftreg_step;
    wire        esl_htsm_timeout_shiftreg_set;
    wire        esl_rcfg_timeout_shiftreg_step;
    wire        esl_rcfg_timeout_shiftreg_set;

    // Timeout counter comparison control signal output
    wire        esl_stsm_timeout_counter_cmp_config;
    wire        esl_rtsm_timeout_counter_cmp_config;
    wire        esl_htsm_timeout_counter_cmp_config;
    wire        esl_rcfg_timeout_counter_cmp_config;

    // Counter timeout comparison result
    reg         esl_counter_timeout;

    // Offset register control outputs
    wire        esl_stsm_pc_va_diff_offset_we;
    wire        esl_stsm_pc_va_diff_offset_clear;
    wire        esl_rtsm_pc_va_diff_offset_we;
    wire        esl_rtsm_pc_va_diff_offset_clear;
    wire        esl_htsm_pc_va_diff_offset_we;
    wire        esl_htsm_pc_va_diff_offset_clear;
    wire        esl_rcfg_pc_va_diff_offset_we;
    wire        esl_rcfg_pc_va_diff_offset_clear;

    // State declarations
    reg [2:0]   state_f;
    reg [2:0]   state_prev_f;
    reg [2:0]   state_next;

    // Signals to trigger the sync method submodules
    // that we need a thread select decision from it
    reg         esl_stsm_trigger;
    reg         esl_rtsm_trigger;
    reg         esl_htsm_trigger;
    reg         esl_rcfg_trigger; 

    //
    // Sequential Logic
    //

    // State sequential logic
    always @ (posedge clk)
    begin
        if (~rst_n)
        begin
            state_f <= DIVERGED;
            state_prev_f <= DIVERGED;
        end
        else
        begin
            state_f <= state_next;
            state_prev_f <= state_f;
        end
    end

    //
    // Combinational Logic
    //

    // Next state and output logic
    always @ *
    begin
        // Default values
        state_next = state_f;
        esl_fcl_nextthr_bf = 4'b0000;
        esl_fcl_stall_bf = 1'b0;
        esl_fcl_switch_bf = 1'b0;
        esl_fdp_sync_pcs_bf = 1'b0;
        esl_fcl_ntr_s = 1'b0;
        esl_fdp_issue_prev_inst_next_s = 1'b0;
        esl_fcl_force_running_s = 1'b0;
        esl_stsm_trigger = 1'b0;
        esl_rtsm_trigger = 1'b0;
        esl_htsm_trigger = 1'b0;
        esl_rcfg_trigger = 1'b0;

        // If no threads are active, just idle
        if (swl_esl_thr_active == 4'b0000)
        begin
            state_next = DIVERGED;
            esl_fcl_nextthr_bf = 4'b0000;
            esl_fcl_stall_bf = 1'b0;
            esl_fcl_switch_bf = 1'b0;
            esl_fdp_sync_pcs_bf = 1'b0;
            esl_fcl_ntr_s = 1'b0; 
            esl_fdp_issue_prev_inst_next_s = 1'b0;
            esl_fcl_force_running_s = 1'b0;
        end
        // If only thr0 is active, just select it
        else if (swl_esl_thr_active == 4'b0001)
        begin
            state_next = DIVERGED;
            esl_fcl_nextthr_bf = 4'b0001;
            esl_fcl_stall_bf = 1'b0;
            esl_fcl_switch_bf = swl_esl_thr_sprdy_or_urdy[0];
            esl_fdp_sync_pcs_bf = 1'b0;
            esl_fcl_ntr_s = swl_esl_thr_sprdy_or_urdy[0];
            esl_fdp_issue_prev_inst_next_s = 1'b0;
            esl_fcl_force_running_s = 1'b0;
        end
        // If only thr1 is active, just select it
        else if (swl_esl_thr_active == 4'b0010)
        begin
            state_next = DIVERGED;
            esl_fcl_nextthr_bf = 4'b0010;
            esl_fcl_stall_bf = 1'b0;
            esl_fcl_switch_bf = swl_esl_thr_sprdy_or_urdy[1];
            esl_fdp_sync_pcs_bf = 1'b0;
            esl_fcl_ntr_s = swl_esl_thr_sprdy_or_urdy[1];
            esl_fdp_issue_prev_inst_next_s = 1'b0;
            esl_fcl_force_running_s = 1'b0;
        end
        // Otherwise, if both threads are active - need to synchronize
        else if (swl_esl_thr_active == 4'b0011)
        begin
            case (state_f)
                DIVERGED:
                begin
                    // Never stall, sync PCs, issue prev instruction 
                    // next, or force running in diverged case
                    esl_fcl_stall_bf = 1'b0;
                    esl_fdp_sync_pcs_bf = 1'b0;
                    esl_fdp_issue_prev_inst_next_s = 1'b0;
                    esl_fcl_force_running_s = 1'b0;
                    // Trigger configured sync method module
                    // and accept outputs from it
                    case (config_esl_sync_method)
                        2'b00:
                        begin
                            esl_stsm_trigger    = 1'b1;
                            state_next          = esl_stsm_state_next;
                            esl_fcl_nextthr_bf  = esl_stsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_stsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_stsm_fcl_ntr_s;
                        end
                        2'b01:
                        begin
                            esl_rtsm_trigger    = 1'b1;
                            state_next          = esl_rtsm_state_next;
                            esl_fcl_nextthr_bf  = esl_rtsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rtsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rtsm_fcl_ntr_s; 
                        end
                        2'b10:
                        begin
                            esl_htsm_trigger    = 1'b1;
                            state_next          = esl_htsm_state_next;
                            esl_fcl_nextthr_bf  = esl_htsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_htsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_htsm_fcl_ntr_s;
                        end
                        2'b11:
                        begin
                            esl_rcfg_trigger    = 1'b1;
                            state_next          = esl_rcfg_state_next;
                            esl_fcl_nextthr_bf  = esl_rcfg_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rcfg_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rcfg_fcl_ntr_s;
                        end
                        default:
                        begin
                            state_next          = 3'bx;
                            esl_fcl_nextthr_bf  = 4'bx;
                            esl_fcl_switch_bf   = 1'bx;
                            esl_fcl_ntr_s       = 1'bx;
                        end
                    endcase
                end
                DIVERGED_DRAFT:
                begin
                    // Select thr1 and go back to diverged
                    state_next = DIVERGED;
                    esl_fcl_nextthr_bf = 4'b0010;
                    esl_fcl_stall_bf = 1'b0;
                    esl_fcl_switch_bf = 1'b1;
                    esl_fdp_sync_pcs_bf = 1'b0;
                    esl_fcl_ntr_s = 1'b1;
                    esl_fdp_issue_prev_inst_next_s = 1'b0;
                    esl_fcl_force_running_s = 1'b0;
                end
                CONVERGED:
                begin
                    // If we diverge due to a page boundary, a branch,
                    // a trap, or a rollback, start fetching for thr1 
                    // again and go to diverged (should reconverge very
                    // quickly after a rollback)
                    if (esl_pcs_pa_page_bndry_s | esl_ctrl_flow_diverged_late_s | 
                        esl_any_trap_bf | fcl_esl_rb_stg_s)
                    begin
                        state_next = DIVERGED;
                        esl_fcl_nextthr_bf = 4'b0010;
                        esl_fcl_stall_bf = 1'b0;
                        esl_fcl_switch_bf = swl_esl_thr_sprdy_or_urdy[1];
                        // BUG FIX: Fixes bug of executing an instruction
                        // twice in this case. The instruction for thread
                        // 0 may have just executed on the previous cycle
                        // if we went from CONVERGED->DIVERGED->CONVERGED
                        // without stalling at all.  Thus, the PC just got
                        // updated and we need to synchronize for one more
                        // cycle.  However, in all other cases we should
                        // not sync the PCs, since they may diverge and
                        // we are asking to fetch for thread 1 on the next
                        // cycle, which causes thread 1's PC to change.
                        // BE CAREFUL CHANGING THIS
                        if (state_prev_f != CONVERGED)
                            esl_fdp_sync_pcs_bf = 1'b1;
                        else
                            esl_fdp_sync_pcs_bf = 1'b0;
                        esl_fcl_ntr_s = swl_esl_thr_sprdy_or_urdy[1];
                        esl_fdp_issue_prev_inst_next_s = 1'b0;
                        esl_fcl_force_running_s = 1'b0;
                    end
                    // If at least one thread is not ready, we will just wait
                    // TODO: We may want to change this
                    else if (!(swl_esl_thr_sprdy_or_urdy[0] && 
                               (swl_esl_thr_sprdy_or_urdy[1] || swl_esl_thr_sprdy_or_urdy_next[1])))
                    begin
                        state_next = CONVERGED;
                        esl_fcl_nextthr_bf = 4'b0000;
                        esl_fcl_stall_bf = 1'b0;
                        esl_fcl_switch_bf = 1'b0;
                        // BUG FIX: In the case an I-cache miss occurs,
                        // we do not want to sync the pcs until we know
                        // we are not going to diverge due to a branch later.
                        // If we have a branch followed by another instruction
                        // that misses in the I-cache, thread1 does not
                        // get info that it also missed in the cache
                        // and may skip the instruction that missed
                        if (swl_esl_icmiss_out && esl_unresolved_br)
                            esl_fdp_sync_pcs_bf = 1'b0;
                        else
                            esl_fdp_sync_pcs_bf = 1'b1;
                        esl_fcl_ntr_s = 1'b1; // Note: This switches out any running thread
                        esl_fdp_issue_prev_inst_next_s = 1'b0;
                        esl_fcl_force_running_s = 1'b0;
                    end
                    // Otherwise, both threads are ready
                    else
                    begin 
                        // If instruction for thr0 is not ready, need to wait
                        if (!fcl_esl_tinst_vld_s[0])
                        begin
                            state_next = CONVERGED;
                            esl_fcl_nextthr_bf = 4'b0001;
                            esl_fcl_stall_bf = 1'b0;
                            esl_fcl_switch_bf = 1'b1;
                            esl_fdp_sync_pcs_bf = 1'b1;
                            esl_fcl_ntr_s = 1'b1;
                            esl_fdp_issue_prev_inst_next_s = 1'b0;
                            esl_fcl_force_running_s = 1'b0;
                        end
                        // Otherwise, both threads are ready and 
                        // thr0 instruction is ready
                        else
                        begin
                            state_next = CONVERGED_DRAFT;
                            esl_fcl_nextthr_bf = 4'b0001;
                            esl_fcl_stall_bf = 1'b0;
                            esl_fcl_switch_bf = 1'b1;
                            esl_fdp_sync_pcs_bf = 1'b1;
                            esl_fcl_ntr_s = 1'b1;
                            esl_fdp_issue_prev_inst_next_s = 1'b0;
                            esl_fcl_force_running_s = 1'b0;
                        end
                    end 
                end
                CONVERGED_DRAFT:
                begin
                    // If we diverge due to a page boundary, a branch,
                    // a trap, or a rollback, start fetching for thr1 
                    // again and go to diverged (should reconverge very
                    // quickly after a rollback)  
                    if (esl_pcs_pa_page_bndry_s | esl_ctrl_flow_diverged_late_s |
                        esl_any_trap_bf | fcl_esl_rb_stg_s)
                    begin
                        state_next = DIVERGED;
                        esl_fcl_nextthr_bf = 4'b0010;
                        esl_fcl_stall_bf = 1'b0;
                        esl_fcl_switch_bf = swl_esl_thr_sprdy_or_urdy[1];
                        esl_fdp_sync_pcs_bf = 1'b0;
                        esl_fcl_ntr_s = swl_esl_thr_sprdy_or_urdy[1];
                        // BUFIX: If the instruction was rolled back,
                        // then NOP is in the previous instruction
                        // register as the previous instruction
                        // was killed. It may be possible to 
                        // grab that instruction from not being
                        // MUXED to a NOP before the execdraft
                        // replay mux to improve performance, but
                        // leaving this as a TODO
                        // BUG FIX: Trap takes precedence over rollback,
                        // need to still draft in case of trap
                        if (fcl_esl_rb_stg_s && !esl_any_trap_bf)
                        begin
                            esl_fdp_issue_prev_inst_next_s = 1'b0;
                            esl_fcl_force_running_s = 1'b0;
                        end
                        else
                        begin
                            esl_fdp_issue_prev_inst_next_s = 1'b1;
                            esl_fcl_force_running_s = 1'b1;
                        end
                    end
                    // Otherwise, we issue previous instruction for thr1
                    // and do not fetch
                    else
                    begin
                        state_next = CONVERGED;
                        esl_fcl_nextthr_bf = 4'b0010;
                        esl_fcl_stall_bf = 1'b1;
                        esl_fcl_switch_bf = 1'b1;
                        esl_fdp_sync_pcs_bf = 1'b1;
                        esl_fcl_ntr_s = 1'b1;
                        esl_fdp_issue_prev_inst_next_s = 1'b1;
                        esl_fcl_force_running_s = 1'b1;
                    end
                end
                DIVERGED_DIFFERENT_CODE_RCFG0:
                begin
                    // Never stall, sync PCs, issue prev instruction 
                    // next, or force running in diverged case
                    esl_fcl_stall_bf = 1'b0;
                    esl_fdp_sync_pcs_bf = 1'b0;
                    esl_fdp_issue_prev_inst_next_s = 1'b0;
                    esl_fcl_force_running_s = 1'b0;
                    // Trigger configured sync method module
                    // and accept outputs from it
                    case (config_esl_sync_method)
                        2'b00:
                        begin
                            esl_stsm_trigger    = 1'b1;
                            state_next          = esl_stsm_state_next;
                            esl_fcl_nextthr_bf  = esl_stsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_stsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_stsm_fcl_ntr_s;
                        end
                        2'b01:
                        begin
                            esl_rtsm_trigger    = 1'b1;
                            state_next          = esl_rtsm_state_next;
                            esl_fcl_nextthr_bf  = esl_rtsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rtsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rtsm_fcl_ntr_s;
                        end
                        2'b10:
                        begin
                            esl_htsm_trigger    = 1'b1;
                            state_next          = esl_htsm_state_next;
                            esl_fcl_nextthr_bf  = esl_htsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_htsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_htsm_fcl_ntr_s;
                        end
                        2'b11:
                        begin
                            esl_rcfg_trigger    = 1'b1;
                            state_next          = esl_rcfg_state_next;
                            esl_fcl_nextthr_bf  = esl_rcfg_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rcfg_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rcfg_fcl_ntr_s;
                        end
                        default:
                        begin
                            state_next          = 3'bx;
                            esl_fcl_nextthr_bf  = 4'bx;
                            esl_fcl_switch_bf   = 1'bx;
                            esl_fcl_ntr_s       = 1'bx;
                        end
                    endcase 
                end
                RCFG1:
                begin
                    // Never stall, sync PCs, issue prev instruction 
                    // next, or force running in diverged case
                    esl_fcl_stall_bf = 1'b0;
                    esl_fdp_sync_pcs_bf = 1'b0;
                    esl_fdp_issue_prev_inst_next_s = 1'b0;
                    esl_fcl_force_running_s = 1'b0;
                    // Trigger configured sync method module
                    // and accept outputs from it
                    case (config_esl_sync_method)
                        2'b00:
                        begin
                            esl_stsm_trigger    = 1'b1;
                            state_next          = esl_stsm_state_next;
                            esl_fcl_nextthr_bf  = esl_stsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_stsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_stsm_fcl_ntr_s;
                        end
                        2'b01:
                        begin
                            esl_rtsm_trigger    = 1'b1;
                            state_next          = esl_rtsm_state_next;
                            esl_fcl_nextthr_bf  = esl_rtsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rtsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rtsm_fcl_ntr_s;
                        end
                        2'b10:
                        begin
                            esl_htsm_trigger    = 1'b1;
                            state_next          = esl_htsm_state_next;
                            esl_fcl_nextthr_bf  = esl_htsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_htsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_htsm_fcl_ntr_s;
                        end
                        2'b11:
                        begin
                            esl_rcfg_trigger    = 1'b1;
                            state_next          = esl_rcfg_state_next;
                            esl_fcl_nextthr_bf  = esl_rcfg_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rcfg_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rcfg_fcl_ntr_s;
                        end
                        default:
                        begin
                            state_next          = 3'bx;
                            esl_fcl_nextthr_bf  = 4'bx;
                            esl_fcl_switch_bf   = 1'bx;
                            esl_fcl_ntr_s       = 1'bx;
                        end
                    endcase
                end
                RCFG2:
                begin
                    // Never stall, sync PCs, issue prev instruction 
                    // next, or force running in diverged case
                    esl_fcl_stall_bf = 1'b0;
                    esl_fdp_sync_pcs_bf = 1'b0;
                    esl_fdp_issue_prev_inst_next_s = 1'b0;
                    esl_fcl_force_running_s = 1'b0;
                    // Trigger configured sync method module
                    // and accept outputs from it
                    case (config_esl_sync_method)
                        2'b00:
                        begin
                            esl_stsm_trigger    = 1'b1;
                            state_next          = esl_stsm_state_next;
                            esl_fcl_nextthr_bf  = esl_stsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_stsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_stsm_fcl_ntr_s;
                        end
                        2'b01:
                        begin
                            esl_rtsm_trigger    = 1'b1;
                            state_next          = esl_rtsm_state_next;
                            esl_fcl_nextthr_bf  = esl_rtsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rtsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rtsm_fcl_ntr_s;
                        end
                        2'b10:
                        begin
                            esl_htsm_trigger    = 1'b1;
                            state_next          = esl_htsm_state_next;
                            esl_fcl_nextthr_bf  = esl_htsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_htsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_htsm_fcl_ntr_s;
                        end
                        2'b11:
                        begin
                            esl_rcfg_trigger    = 1'b1;
                            state_next          = esl_rcfg_state_next;
                            esl_fcl_nextthr_bf  = esl_rcfg_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rcfg_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rcfg_fcl_ntr_s;
                        end
                        default:
                        begin
                            state_next          = 3'bx;
                            esl_fcl_nextthr_bf  = 4'bx;
                            esl_fcl_switch_bf   = 1'bx;
                            esl_fcl_ntr_s       = 1'bx;
                        end
                    endcase
                end
                RCFG3:
                begin
                    // Never stall, sync PCs, issue prev instruction 
                    // next, or force running in diverged case
                    esl_fcl_stall_bf = 1'b0;
                    esl_fdp_sync_pcs_bf = 1'b0;
                    esl_fdp_issue_prev_inst_next_s = 1'b0;
                    esl_fcl_force_running_s = 1'b0;
                    // Trigger configured sync method module
                    // and accept outputs from it
                    case (config_esl_sync_method)
                        2'b00:
                        begin
                            esl_stsm_trigger    = 1'b1;
                            state_next          = esl_stsm_state_next;
                            esl_fcl_nextthr_bf  = esl_stsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_stsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_stsm_fcl_ntr_s;
                        end
                        2'b01:
                        begin
                            esl_rtsm_trigger    = 1'b1;
                            state_next          = esl_rtsm_state_next;
                            esl_fcl_nextthr_bf  = esl_rtsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rtsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rtsm_fcl_ntr_s;
                        end
                        2'b10:
                        begin
                            esl_htsm_trigger    = 1'b1;
                            state_next          = esl_htsm_state_next;
                            esl_fcl_nextthr_bf  = esl_htsm_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_htsm_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_htsm_fcl_ntr_s;
                        end
                        2'b11:
                        begin
                            esl_rcfg_trigger    = 1'b1;
                            state_next          = esl_rcfg_state_next;
                            esl_fcl_nextthr_bf  = esl_rcfg_fcl_nextthr_bf;
                            esl_fcl_switch_bf   = esl_rcfg_fcl_switch_bf;
                            esl_fcl_ntr_s       = esl_rcfg_fcl_ntr_s;
                        end
                        default:
                        begin
                            state_next          = 3'bx;
                            esl_fcl_nextthr_bf  = 4'bx;
                            esl_fcl_switch_bf   = 1'bx;
                            esl_fcl_ntr_s       = 1'bx;
                        end
                    endcase
                end
                default:
                begin
                    state_next = 3'bx;
                    esl_fcl_nextthr_bf = 4'bxxxx;
                    esl_fcl_stall_bf = 1'bx;
                    esl_fcl_switch_bf = 1'bx;
                    esl_fdp_sync_pcs_bf = 1'bx;
                    esl_fcl_ntr_s = 1'bx;
                    esl_fdp_issue_prev_inst_next_s = 1'bx;
                    esl_fcl_force_running_s = 1'bx;
                end 
            endcase
        end
    end

    // Signal multiplexing for signals going into shared submodules (counter, LFSR
    // and offset register)
    always @ *
    begin
        case (config_esl_sync_method)
            2'b00:
            begin
                esl_lfsr_step = esl_stsm_lfsr_step;
                esl_timeout_counter_step = esl_stsm_timeout_counter_step;
                esl_timeout_counter_clear = esl_stsm_timeout_counter_clear;
                esl_timeout_counter_set = esl_stsm_timeout_counter_set;
                esl_pc_va_diff_offset_we = esl_stsm_pc_va_diff_offset_we;
                esl_pc_va_diff_offset_clear = esl_stsm_pc_va_diff_offset_clear;
                esl_timeout_shiftreg_step = esl_stsm_timeout_shiftreg_step;
                esl_timeout_shiftreg_set = esl_stsm_timeout_shiftreg_set;
                esl_timeout_counter_cmp_config = esl_stsm_timeout_counter_cmp_config;
            end
            2'b01:
            begin
                esl_lfsr_step = esl_rtsm_lfsr_step;
                esl_timeout_counter_step = esl_rtsm_timeout_counter_step;
                esl_timeout_counter_clear = esl_rtsm_timeout_counter_clear;
                esl_timeout_counter_set = esl_rtsm_timeout_counter_set;
                esl_pc_va_diff_offset_we = esl_rtsm_pc_va_diff_offset_we;
                esl_pc_va_diff_offset_clear = esl_rtsm_pc_va_diff_offset_clear;
                esl_timeout_shiftreg_step = esl_rtsm_timeout_shiftreg_step;
                esl_timeout_shiftreg_set = esl_rtsm_timeout_shiftreg_set;
                esl_timeout_counter_cmp_config = esl_rtsm_timeout_counter_cmp_config;
            end
            2'b10:
            begin
                esl_lfsr_step = esl_htsm_lfsr_step;
                esl_timeout_counter_step = esl_htsm_timeout_counter_step;
                esl_timeout_counter_clear = esl_htsm_timeout_counter_clear;
                esl_timeout_counter_set = esl_htsm_timeout_counter_set;
                esl_pc_va_diff_offset_we = esl_htsm_pc_va_diff_offset_we;
                esl_pc_va_diff_offset_clear = esl_htsm_pc_va_diff_offset_clear;
                esl_timeout_shiftreg_step = esl_htsm_timeout_shiftreg_step;
                esl_timeout_shiftreg_set = esl_htsm_timeout_shiftreg_set;
                esl_timeout_counter_cmp_config = esl_htsm_timeout_counter_cmp_config;
            end
            2'b11:
            begin
                esl_lfsr_step = esl_rcfg_lfsr_step;
                esl_timeout_counter_step = esl_rcfg_timeout_counter_step;
                esl_timeout_counter_clear = esl_rcfg_timeout_counter_clear;
                esl_timeout_counter_set = esl_rcfg_timeout_counter_set;
                esl_pc_va_diff_offset_we = esl_rcfg_pc_va_diff_offset_we;
                esl_pc_va_diff_offset_clear = esl_rcfg_pc_va_diff_offset_clear;
                esl_timeout_shiftreg_step = esl_rcfg_timeout_shiftreg_step;
                esl_timeout_shiftreg_set = esl_rcfg_timeout_shiftreg_set;
                esl_timeout_counter_cmp_config = esl_rcfg_timeout_counter_cmp_config;
            end
            default:
            begin
                esl_lfsr_step = 1'bx;
                esl_timeout_counter_step = 1'bx;
                esl_timeout_counter_clear = 1'bx;
                esl_timeout_counter_set = 1'bx;
                esl_pc_va_diff_offset_we = 1'bx;
                esl_pc_va_diff_offset_clear = 1'bx;
                esl_timeout_shiftreg_step = 1'bx;
                esl_timeout_shiftreg_set = 1'bx;
            end
        endcase 
    end

    // Counter threshold comparisons
    always @ *
    begin
        esl_counter_timeout = (esl_timeout_counter_count_f >= 
                              (esl_timeout_counter_cmp_config ? config_esl_counter_timeout 
                                                              : esl_timeout_shiftreg_f));
    end

    //
    // Sub-module instantiations
    //

    // LFSR for pseudo random bits (shared resource)
    sparc_ifu_esl_lfsr lfsr
    (
        .clk (clk),
        .rst_n (rst_n),
        .lfsr_seed (config_esl_lfsr_seed),
        .lfsr_ld (config_esl_lfsr_ld),
        .lfsr_step (config_esl_en & esl_lfsr_step),
        .lfsr_state_f (),
        .lfsr_out_f (pseudo_rand_f)
    );

    // Timeout counter (shared resource)
    sparc_ifu_esl_counter 
    #(
        .COUNT_BIT_WIDTH (16)
    ) cfg_timeout_counter
    (
        .clk (clk),
        .rst_n (rst_n),
        .step (config_esl_en & esl_timeout_counter_step),
        .clear (config_esl_en & esl_timeout_counter_clear),
        .set (config_esl_en & esl_timeout_counter_set),
        .count_f (esl_timeout_counter_count_f)
    );

    // Shift register which sets one of the possible timeouts
    // for above counter (shared resource)
    sparc_ifu_esl_shiftreg 
    #(
        .SHIFT_REG_BIT_WIDTH (16)
    ) timeout_shiftreg
    (
        .clk (clk),
        .rst_n (rst_n),
        .step (config_esl_en & esl_timeout_shiftreg_step),
        .set (config_esl_en & esl_timeout_shiftreg_set),
        .shift_reg_f (esl_timeout_shiftreg_f)
    );

    // STSM synchronization mechanism
    sparc_ifu_esl_stsm 
    #(
        .DIVERGED (DIVERGED),
        .DIVERGED_DRAFT (DIVERGED_DRAFT),
        .CONVERGED (CONVERGED),
        .CONVERGED_DRAFT (CONVERGED_DRAFT),
        .DIVERGED_DIFFERENT_CODE_RCFG0 (DIVERGED_DIFFERENT_CODE_RCFG0),
        .RCFG1 (RCFG1),
        .RCFG2 (RCFG2),
        .RCFG3 (RCFG3)
    ) stsm
    (
        .esl_stsm_trigger (esl_stsm_trigger),
        .esl_state_f (state_f),
        .swl_esl_thr_sprdy_or_urdy (swl_esl_thr_sprdy_or_urdy),
        .fcl_esl_tinst_vld_s (fcl_esl_tinst_vld_s),
        .esl_pc_va_diff_lt_thresh_s (esl_pc_va_diff_lt_thresh_s),
        .esl_counter_timeout (esl_counter_timeout),
        .esl_min_pc_va_s (esl_min_pc_va_s),
        .fcl_esl_thr_f (fcl_esl_thr_f),
        .esl_tirs_eql_s (esl_tirs_eql_s),
        .esl_ops_eql_s (esl_ops_eql_s),
        .esl_pcs_pa_eql_s (esl_pcs_pa_eql_s),
        .esl_unresolved_br (esl_unresolved_br),
        .esl_brtrp_target_pc_bf_f (esl_brtrp_target_pc_bf_f),
        .esl_stsm_state_next (esl_stsm_state_next),
        .esl_stsm_fcl_nextthr_bf (esl_stsm_fcl_nextthr_bf),
        .esl_stsm_fcl_switch_bf (esl_stsm_fcl_switch_bf),
        .esl_stsm_fcl_ntr_s (esl_stsm_fcl_ntr_s),
        .esl_stsm_lfsr_step (esl_stsm_lfsr_step),
        .esl_stsm_timeout_counter_step (esl_stsm_timeout_counter_step),
        .esl_stsm_timeout_counter_clear (esl_stsm_timeout_counter_clear),
        .esl_stsm_timeout_counter_set (esl_stsm_timeout_counter_set),
        .esl_stsm_pc_va_diff_offset_we (esl_stsm_pc_va_diff_offset_we),
        .esl_stsm_pc_va_diff_offset_clear (esl_stsm_pc_va_diff_offset_clear),
        .esl_stsm_timeout_shiftreg_step (esl_stsm_timeout_shiftreg_step),
        .esl_stsm_timeout_shiftreg_set (esl_stsm_timeout_shiftreg_set),
        .esl_stsm_timeout_counter_cmp_config (esl_stsm_timeout_counter_cmp_config)
    );

    // RTSM synchronization mechanism
    sparc_ifu_esl_rtsm 
    #(
        .DIVERGED (DIVERGED),
        .DIVERGED_DRAFT (DIVERGED_DRAFT),
        .CONVERGED (CONVERGED),
        .CONVERGED_DRAFT (CONVERGED_DRAFT),
        .DIVERGED_DIFFERENT_CODE_RCFG0 (DIVERGED_DIFFERENT_CODE_RCFG0),
        .RCFG1 (RCFG1),
        .RCFG2 (RCFG2),
        .RCFG3 (RCFG3)
    ) rtsm
    (
        .esl_rtsm_trigger (esl_rtsm_trigger),
        .esl_state_f (state_f),
        .swl_esl_thr_sprdy_or_urdy (swl_esl_thr_sprdy_or_urdy),
        .fcl_esl_tinst_vld_s (fcl_esl_tinst_vld_s),
        .pseudo_rand_f (pseudo_rand_f),
        .fcl_esl_thr_f (fcl_esl_thr_f),
        .esl_tirs_eql_s (esl_tirs_eql_s),
        .esl_ops_eql_s (esl_ops_eql_s),
        .esl_pcs_pa_eql_s (esl_pcs_pa_eql_s),
        .esl_unresolved_br (esl_unresolved_br),
        .esl_brtrp_target_pc_bf_f (esl_brtrp_target_pc_bf_f),
        .esl_rtsm_state_next (esl_rtsm_state_next),
        .esl_rtsm_fcl_nextthr_bf (esl_rtsm_fcl_nextthr_bf),
        .esl_rtsm_fcl_switch_bf (esl_rtsm_fcl_switch_bf),
        .esl_rtsm_fcl_ntr_s (esl_rtsm_fcl_ntr_s),
        .esl_rtsm_lfsr_step (esl_rtsm_lfsr_step),
        .esl_rtsm_timeout_counter_step (esl_rtsm_timeout_counter_step),
        .esl_rtsm_timeout_counter_clear (esl_rtsm_timeout_counter_clear),
        .esl_rtsm_timeout_counter_set (esl_rtsm_timeout_counter_set),
        .esl_rtsm_pc_va_diff_offset_we (esl_rtsm_pc_va_diff_offset_we),
        .esl_rtsm_pc_va_diff_offset_clear (esl_rtsm_pc_va_diff_offset_clear),
        .esl_rtsm_timeout_shiftreg_step (esl_rtsm_timeout_shiftreg_step),
        .esl_rtsm_timeout_shiftreg_set (esl_rtsm_timeout_shiftreg_set),
        .esl_rtsm_timeout_counter_cmp_config (esl_rtsm_timeout_counter_cmp_config)
    );

    // HTSM synchronization mechanism
    sparc_ifu_esl_htsm
    #(
        .DIVERGED (DIVERGED),
        .DIVERGED_DRAFT (DIVERGED_DRAFT),
        .CONVERGED (CONVERGED),
        .CONVERGED_DRAFT (CONVERGED_DRAFT),
        .DIVERGED_DIFFERENT_CODE_RCFG0 (DIVERGED_DIFFERENT_CODE_RCFG0),
        .RCFG1 (RCFG1),
        .RCFG2 (RCFG2),
        .RCFG3 (RCFG3)
    ) htsm
    (
        .clk (clk),
        .rst_n (rst_n),
        .esl_htsm_trigger (esl_htsm_trigger),
        .esl_state_f (state_f),
        .swl_esl_thr_sprdy_or_urdy (swl_esl_thr_sprdy_or_urdy),
        .fcl_esl_tinst_vld_s (fcl_esl_tinst_vld_s),
        .pseudo_rand_f (pseudo_rand_f),
        .esl_pc_va_diff_lt_thresh_s (esl_pc_va_diff_lt_thresh_s),
        .esl_counter_timeout (esl_counter_timeout),
        .esl_shiftreg_lsb (esl_timeout_shiftreg_f[0]),
        .esl_min_pc_va_s (esl_min_pc_va_s),
        .esl_pcs_va_eql_s (esl_pcs_va_eql_s),
        .fcl_esl_thr_f (fcl_esl_thr_f),
        .esl_tirs_eql_s (esl_tirs_eql_s),
        .esl_ops_eql_s (esl_ops_eql_s),
        .esl_pcs_pa_eql_s (esl_pcs_pa_eql_s),
        .esl_unresolved_br (esl_unresolved_br),
        .esl_brtrp_target_pc_bf_f (esl_brtrp_target_pc_bf_f),
        .esl_htsm_state_next (esl_htsm_state_next),
        .esl_htsm_fcl_nextthr_bf (esl_htsm_fcl_nextthr_bf),
        .esl_htsm_fcl_switch_bf (esl_htsm_fcl_switch_bf),
        .esl_htsm_fcl_ntr_s (esl_htsm_fcl_ntr_s),
        .esl_htsm_lfsr_step (esl_htsm_lfsr_step),
        .esl_htsm_timeout_counter_step (esl_htsm_timeout_counter_step),
        .esl_htsm_timeout_counter_clear (esl_htsm_timeout_counter_clear),
        .esl_htsm_timeout_counter_set (esl_htsm_timeout_counter_set),
        .esl_htsm_pc_va_diff_offset_we (esl_htsm_pc_va_diff_offset_we),
        .esl_htsm_pc_va_diff_offset_clear (esl_htsm_pc_va_diff_offset_clear),
        .esl_htsm_timeout_shiftreg_step (esl_htsm_timeout_shiftreg_step),
        .esl_htsm_timeout_shiftreg_set (esl_htsm_timeout_shiftreg_set),
        .esl_htsm_timeout_counter_cmp_config (esl_htsm_timeout_counter_cmp_config)
    );

endmodule
// Copyright (c) 2015 Princeton University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 *  Description:
 *      This module encapsulates different Execution Drafting synchronization methods
 *      into one module and multiplexes between them based on configuration bits.  It
 *      also supplies more succinct inputs to the synchronization methods derived from
 *      signals from the IFU and derives the correct outputs to go to the IFU.
 *
 *  Note: While the interface to this module supports 4 threads, the logic only supports 2
 */

module sparc_ifu_esl
(
    input               clk,
    input               rst_n,

    // Configuration bits input
    input               config_esl_en,
    input [1:0]         config_esl_sync_method,

    // LFSR management signals for RTSM
    input [15:0]        config_esl_lfsr_seed,
    input               config_esl_lfsr_ld,

    // Threshold configuration signal for PC difference
    input [49:0]        config_esl_pc_diff_thresh,

    // Counter timeout configuration value
    input [15:0]        config_esl_counter_timeout,

    // Active threads from thread FSMs
    input [3:0]         swl_esl_thr_active,

    // Threads ready from thread FSMs, both
    // non-speculative ready and speculative ready
    input [3:0]         swl_esl_thr_urdy,
    input [3:0]         swl_esl_thr_sprdy_or_urdy,
    input [3:0]         swl_esl_thr_sprdy_or_urdy_next,

    // Use speculative ready vector, meaning no
    // thread is non-speculatively ready
    input               swl_esl_use_spec,

    // Thread in F stage
    input [3:0]         fcl_esl_thr_f,

    // Instructions to be in S2 TIRs next
    input [32:0]        fdp_esl_t0inst_next_s2,

    input [32:0]        fdp_esl_t1inst_next_s2,















    // Physical address of above instructions
    input [39:2]        fdp_esl_t0inst_paddr_next_s2,

    input [39:2]        fdp_esl_t1inst_paddr_next_s2,















    // Virtual address of above instructions
    input [48:0]        fdp_esl_t0pc_next_s2,

    input [48:0]        fdp_esl_t1pc_next_s2,















    // Above instructions valid on the next cycle?
    input [3:0]         fcl_esl_tinst_vld_next_s,

    // Branch taken in E and M stage (taken in M stage if br was taken in previous stage)
    input               fcl_esl_brtaken_e,
    input               fcl_esl_brtaken_m,

    // Threads in E and M
    input [3:0]         fcl_esl_thr_e,
    input [3:0]         fcl_esl_thr_m,

    // Valid instructions in D, E and M
    input               fcl_esl_inst_vld_d,
    input               fcl_esl_inst_vld_e,
    input               fcl_esl_inst_vld_m,

    // Trap on this cycle
    input [3:0]         fcl_esl_thr_trap_bf,

    // Rollback on this cycle
    input               fcl_esl_rb_stg_s,

    // Unresolved branch instructions
    input               dcl_esl_br_inst_d,
    input               dcl_esl_br_inst_e,

    // Whether a branch target pc is in bf of f
    input [3:0]         fdp_esl_brtrp_target_pc_bf_f,

    // Whether there is an outstanding I-cache miss to thr0
    input               swl_esl_icmiss_out,

    // Next thread to fetch for
    output [3:0]        esl_fcl_nextthr_bf,

    // Stall fetch (i.e. should it fetch)
    output              esl_fcl_stall_bf,

    // Whether to switch to esl_fcl_nextthr_bf
    output              esl_fcl_switch_bf,

    // Sync PCs between leader and follower threads
    output              esl_fdp_sync_pcs_bf,

    // If the thread to be issued is ready (i.e. should it be issued)
    output              esl_fcl_ntr_s,

    // Issue previously issued instruction for
    // next thread
    output              esl_fdp_issue_prev_inst_next_s,

    // Output signal to force s2 to be running on next cycle
    output              esl_fcl_force_running_s
);

    //
    // Parameter Definitions
    //

    //
    // Signal Declarations
    //

    // Derived inputs to thread select modules
    reg         esl_tirs_eql_s;
    reg [1:0]   esl_t0inst_next_op1;
    reg [1:0]   esl_t1inst_next_op1;
    reg [2:0]   esl_t0inst_next_op2;
    reg [2:0]   esl_t1inst_next_op2;
    reg [5:0]   esl_t0inst_next_op3;
    reg [5:0]   esl_t1inst_next_op3;
    reg [8:0]   esl_t0inst_next_opf;
    reg [8:0]   esl_t1inst_next_opf;
    reg [2:0]   esl_t0inst_next_br_rcond;
    reg [2:0]   esl_t1inst_next_br_rcond;
    reg [3:0]   esl_t0inst_next_br_cond;
    reg [3:0]   esl_t1inst_next_br_cond;
    reg [4:0]   esl_t0inst_next_fcn;
    reg [4:0]   esl_t1inst_next_fcn;
    reg [5:0]   esl_t0inst_next_opf_low;
    reg [5:0]   esl_t1inst_next_opf_low;
    reg [3:0]   esl_t0inst_next_mv_cond;
    reg [3:0]   esl_t1inst_next_mv_cond;
    reg [2:0]   esl_t0inst_next_mv_rcond;
    reg [2:0]   esl_t1inst_next_mv_rcond;
    reg         esl_t0inst_next_x;
    reg         esl_t1inst_next_x;
    reg [4:0]   esl_t0inst_next_rd;
    reg [4:0]   esl_t1inst_next_rd;
    reg         esl_tirs_op1_eql;
    reg         esl_tirs_op2_eql;
    reg         esl_tirs_op3_eql;
    reg         esl_tirs_opf_eql;
    reg         esl_tirs_br_rcond_eql;
    reg         esl_tirs_br_cond_eql;
    reg         esl_tirs_fcn_eql;
    reg         esl_tirs_opf_low_eql;
    reg         esl_tirs_mv_cond_eql;
    reg         esl_tirs_mv_rcond_eql;
    reg         esl_tirs_x_eql;
    reg         esl_tirs_rd_eql;
    reg         esl_ops_eql_s;
    reg         esl_pcs_pa_eql_s;
    reg         esl_ctrl_flow_diverged_late_s;
    reg         esl_pcs_pa_page_bndry_s;
    reg         esl_any_trap_bf;
    reg [49:0]  esl_pc_va_diff_s;
    reg [49:0]  esl_pc_va_diff_offset_s;
    reg [49:0]  esl_pc_va_diff_abs_s;
    reg [3:0]   esl_min_pc_va_s;
    reg         esl_pc_va_diff_lt_thresh_s;
    reg         esl_pcs_va_eql_s;
    reg         esl_unresolved_br;
    reg         esl_brtrp_target_pc_bf_f;

    // Register that may be updated with PC VA difference
    // for offset calculation
    reg [49:0]  esl_pc_va_diff_offset_f;

    // Control outputs from ESL FSM to offset register
    wire        esl_pc_va_diff_offset_we;
    wire        esl_pc_va_diff_offset_clear;













    //
    // Sequential Logic
    //

    // Register that gets updated with PC VA difference
    // when instructed to do so
    always @ (posedge clk)
    begin
        if (~rst_n | esl_pc_va_diff_offset_clear)
            esl_pc_va_diff_offset_f <= 50'b0;
        else if (esl_pc_va_diff_offset_we)
            esl_pc_va_diff_offset_f <= esl_pc_va_diff_s;
    end

    //
    // Combinational Logic
    //

    // Derive inputs to sub-synchronization modules
    always @ *
    begin
        // Thread instructions equal
        esl_tirs_eql_s = (fdp_esl_t0inst_next_s2 == fdp_esl_t1inst_next_s2);
      
        // Thread instructions opcodes
        esl_t0inst_next_op1 = fdp_esl_t0inst_next_s2[31:30];
        esl_t1inst_next_op1 = fdp_esl_t1inst_next_s2[31:30];
        esl_t0inst_next_op2 = fdp_esl_t0inst_next_s2[24:22];
        esl_t1inst_next_op2 = fdp_esl_t1inst_next_s2[24:22];
        esl_t0inst_next_op3 = fdp_esl_t0inst_next_s2[24:19];
        esl_t1inst_next_op3 = fdp_esl_t1inst_next_s2[24:19];
        esl_t0inst_next_opf = fdp_esl_t0inst_next_s2[13:5];
        esl_t1inst_next_opf = fdp_esl_t1inst_next_s2[13:5];
        esl_t0inst_next_br_rcond = fdp_esl_t0inst_next_s2[27:25];
        esl_t1inst_next_br_rcond = fdp_esl_t1inst_next_s2[27:25];
        esl_t0inst_next_br_cond = fdp_esl_t0inst_next_s2[28:25];
        esl_t1inst_next_br_cond = fdp_esl_t1inst_next_s2[28:25];
        esl_t0inst_next_fcn = fdp_esl_t0inst_next_s2[29:25];
        esl_t1inst_next_fcn = fdp_esl_t1inst_next_s2[29:25];
        esl_t0inst_next_opf_low = fdp_esl_t0inst_next_s2[10:5];
        esl_t1inst_next_opf_low = fdp_esl_t1inst_next_s2[10:5];
        esl_t0inst_next_mv_cond = fdp_esl_t0inst_next_s2[17:14];
        esl_t1inst_next_mv_cond = fdp_esl_t1inst_next_s2[17:14];
        esl_t0inst_next_mv_rcond = fdp_esl_t0inst_next_s2[12:10];
        esl_t1inst_next_mv_rcond = fdp_esl_t1inst_next_s2[12:10];
        esl_t0inst_next_x = fdp_esl_t0inst_next_s2[12];
        esl_t1inst_next_x = fdp_esl_t1inst_next_s2[12];
        esl_t0inst_next_rd = fdp_esl_t0inst_next_s2[29:25];
        esl_t1inst_next_rd = fdp_esl_t1inst_next_s2[29:25];    
    
        // Thread instruction opcodes equal
        esl_tirs_op1_eql = (esl_t0inst_next_op1 == esl_t1inst_next_op1);
        esl_tirs_op2_eql = (esl_t0inst_next_op2 == esl_t1inst_next_op2);
        esl_tirs_op3_eql = (esl_t0inst_next_op3 == esl_t1inst_next_op3);
        esl_tirs_opf_eql = (esl_t0inst_next_opf == esl_t1inst_next_opf);
        esl_tirs_br_rcond_eql = (esl_t0inst_next_br_rcond == esl_t1inst_next_br_rcond);
        esl_tirs_br_cond_eql = (esl_t0inst_next_br_cond == esl_t1inst_next_br_cond);
        esl_tirs_fcn_eql = (esl_t0inst_next_fcn == esl_t1inst_next_fcn);
        esl_tirs_opf_low_eql = (esl_t0inst_next_opf_low == esl_t1inst_next_opf_low);
        esl_tirs_mv_cond_eql = (esl_t0inst_next_mv_cond == esl_t1inst_next_mv_cond);
        esl_tirs_mv_rcond_eql = (esl_t0inst_next_mv_rcond == esl_t1inst_next_mv_rcond);
        esl_tirs_x_eql = (esl_t0inst_next_x == esl_t1inst_next_x);
        esl_tirs_rd_eql = (esl_t0inst_next_rd == esl_t1inst_next_rd);

        // Determine if opcode of two instructions is equal
        // This logic is derived from SPARC V9 instruction formats
        case (esl_t0inst_next_op1)
            2'b00:
            begin
                // Branchs and SETHI, need to check op2
                
                // If branch on integer register with predication, need to
                // check rcond code
                if (esl_t0inst_next_op2 == 3'b011)
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op2_eql & esl_tirs_br_rcond_eql;
                // If branch on floating-point condition codes or branch on floating
                // point condition codes with prediction or branch on integer
                // condition codes or branch on integer condition codes with
                // prediction, need to check cond code
                else if (esl_t0inst_next_op2 == 3'b110 || esl_t0inst_next_op2 == 3'b101 || 
                         esl_t0inst_next_op2 == 3'b010 || esl_t0inst_next_op2 == 3'b001)
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op2_eql & esl_tirs_br_cond_eql;
                // Otherwise, just need to check op2
                else
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op2_eql;
            end
            2'b01:
            begin
                // Call instruction, just need to check op1
                esl_ops_eql_s = esl_tirs_op1_eql;
            end
            2'b10:
            begin
                // Arithmetic and Misc. (FP) instructions

                // If Done/Retry, need to check fcn
                if (esl_t0inst_next_op3 == 6'b111110)
                begin
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_fcn_eql;
                end
                // If floating-point add or subtract or floating-point compare
                // or floating-point to integer conversion or convert between
                // floating point formats or convert integer to floating-point
                // or floating-point move or floating-point muldiv, or floating-point
                // square root or floating-point move on condition or impl. dep., 
                // need to check opf
                else if (esl_t0inst_next_op3 == 6'b110100 || esl_t0inst_next_op3 == 6'b110101 ||
                         esl_t0inst_next_op3 == 6'b110110)
                begin
                    // If floating point move on condition, need to check cond
                    if (esl_t0inst_next_op3 == 6'b110101 &&
                        (esl_t0inst_next_opf_low == 6'b000001 ||
                         esl_t0inst_next_opf_low == 6'b000010 ||
                         esl_t0inst_next_opf_low == 6'b000011))
                        esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_opf_low_eql &
                                        esl_tirs_mv_cond_eql;
                    // If floating point move on integer condition, need to check rcond
                    else if (esl_t0inst_next_op3 == 6'b110101 &&
                        (esl_t0inst_next_opf_low[4:0] == 5'b00101 ||
                         esl_t0inst_next_opf_low[4:0] == 5'b00110 ||
                         esl_t0inst_next_opf_low[4:0] == 5'b00111))
                        esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_opf_low_eql &
                                        esl_tirs_mv_rcond_eql;
                    // Otherwise, just need to check opf
                    else
                        esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_opf_eql;
                end
                // If move on condition, need to check cond
                else if (esl_t0inst_next_op3 == 6'b101100)
                begin
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_mv_cond_eql;
                end
                // If move on regiser condition, need to check rcond
                else if (esl_t0inst_next_op3 == 6'b101111)
                begin
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_mv_rcond_eql;
                end
                // If save or restore, need to check fcn
                else if (esl_t0inst_next_op3 == 6'b110001)
                begin
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_fcn_eql;
                end
                // If a shift type, need to check x
                else if (esl_t0inst_next_op3 == 6'b100101 || esl_t0inst_next_op3 == 6'b100110 ||
                         esl_t0inst_next_op3 == 6'b100111)
                begin    
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_x_eql;
                end
                // If software initiated reset, need to check rd
                else if (esl_t0inst_next_op3 == 6'b110000 && esl_t0inst_next_rd == 5'b01111)
                begin
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_rd_eql;
                end
                // If trap on integer condition codes, need to check cond
                else if (esl_t0inst_next_op3 == 6'b111010)
                begin
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_br_cond_eql;
                end
                // Otherwise, just need to check op3
                else
                begin
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql;
                end
            end
            2'b11:
            begin
                // Loads and Stores, need to check op3
                
                // If load/store floating point state register, need to check rd
                if (esl_t0inst_next_op3 == 6'b100001 || esl_t0inst_next_op3 == 6'b100101)
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_rd_eql;
                // If prefetch, need to check fcn
                else if (esl_t0inst_next_op3 == 6'b101101 || esl_t0inst_next_op3 == 6'b111101)
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql & esl_tirs_fcn_eql;
                // Otherwise, just need to check op3
                else
                    esl_ops_eql_s = esl_tirs_op1_eql & esl_tirs_op3_eql;
            end
            default:
                esl_ops_eql_s = 1'bx;
        endcase

        // Thread PC physical addresses equal
        esl_pcs_pa_eql_s = (fdp_esl_t0inst_paddr_next_s2 == fdp_esl_t1inst_paddr_next_s2) &
                           fcl_esl_tinst_vld_next_s[0] & fcl_esl_tinst_vld_next_s[1];

        // Control flow diverged
        esl_ctrl_flow_diverged_late_s = ((fcl_esl_brtaken_e & ~fcl_esl_brtaken_m) |
                                         (~fcl_esl_brtaken_e & fcl_esl_brtaken_m)) &
                                        (fcl_esl_thr_e == 4'b0010) &
                                        (fcl_esl_thr_m == 4'b0001) &
                                        fcl_esl_inst_vld_e & fcl_esl_inst_vld_m;

        // Check if this is the last instruction in a page (i.e. PA mod 8K is 8K - 4)
        esl_pcs_pa_page_bndry_s = ({fdp_esl_t0inst_paddr_next_s2, 2'b0} % 40'd8192) == 40'd8188;

        // Check if any trap occurred
        esl_any_trap_bf = fcl_esl_thr_trap_bf[0] | fcl_esl_thr_trap_bf[1];

        // Difference in PCs
        esl_pc_va_diff_s = ({1'b0, fdp_esl_t0pc_next_s2} - {1'b0, fdp_esl_t1pc_next_s2});

        // Difference in PCs plus offset
        esl_pc_va_diff_offset_s = esl_pc_va_diff_s + esl_pc_va_diff_offset_f;
        
        // Take absolute value of difference
        esl_pc_va_diff_abs_s = esl_pc_va_diff_offset_s;
        if (esl_pc_va_diff_abs_s[49])
            esl_pc_va_diff_abs_s = (~esl_pc_va_diff_abs_s) + 1;

        // Get minimum PC
        if (~esl_pc_va_diff_offset_s[49])
            esl_min_pc_va_s = 4'b0010;
        else
            esl_min_pc_va_s = 4'b0001;

        // Evaluate if difference is less than thresh
        esl_pc_va_diff_lt_thresh_s = (esl_pc_va_diff_abs_s < config_esl_pc_diff_thresh);

        // Evaluate if VA PCs are equal (including offset)
        esl_pcs_va_eql_s = (esl_pc_va_diff_offset_s == 50'b0);

        // Any unresolved branches in pipe?
        esl_unresolved_br = (dcl_esl_br_inst_d & fcl_esl_inst_vld_d) |
                            (dcl_esl_br_inst_e & fcl_esl_inst_vld_e);

        // A branch or trap PC is in bf or f
        esl_brtrp_target_pc_bf_f = |fdp_esl_brtrp_target_pc_bf_f[1:0];
    end

    //
    // Sub-module Instantiations
    //

    // FSM submodule - implements generic FSM logic for all synchronization
    // methods and multiplexes the synchronization method specific portion
    // according to config_esl_sync_method
    sparc_ifu_esl_fsm esl_fsm
    (
        .clk (clk),
        .rst_n (rst_n),
        .config_esl_en (config_esl_en),
        .config_esl_sync_method (config_esl_sync_method),
        .config_esl_counter_timeout (config_esl_counter_timeout),
        .config_esl_lfsr_seed (config_esl_lfsr_seed),
        .config_esl_lfsr_ld (config_esl_lfsr_ld),
        .swl_esl_thr_active (swl_esl_thr_active),
        .swl_esl_thr_urdy (swl_esl_thr_urdy),
        .swl_esl_thr_sprdy_or_urdy (swl_esl_thr_sprdy_or_urdy),
        .swl_esl_thr_sprdy_or_urdy_next (swl_esl_thr_sprdy_or_urdy_next),
        .swl_esl_use_spec (swl_esl_use_spec),
        .fcl_esl_tinst_vld_s (fcl_esl_tinst_vld_next_s),
        .fcl_esl_thr_f (fcl_esl_thr_f),
        .fcl_esl_rb_stg_s (fcl_esl_rb_stg_s),
        .esl_tirs_eql_s (esl_tirs_eql_s),
        .esl_ops_eql_s (esl_ops_eql_s),
        .esl_pcs_pa_eql_s (esl_pcs_pa_eql_s),
        .esl_ctrl_flow_diverged_late_s (esl_ctrl_flow_diverged_late_s),
        .esl_pcs_pa_page_bndry_s (esl_pcs_pa_page_bndry_s),
        .esl_any_trap_bf (esl_any_trap_bf),
        .esl_min_pc_va_s (esl_min_pc_va_s),
        .esl_pc_va_diff_lt_thresh_s (esl_pc_va_diff_lt_thresh_s),
        .esl_pcs_va_eql_s (esl_pcs_va_eql_s),
        .esl_unresolved_br (esl_unresolved_br),
        .esl_brtrp_target_pc_bf_f (esl_brtrp_target_pc_bf_f),
        .swl_esl_icmiss_out (swl_esl_icmiss_out),
        .esl_fcl_nextthr_bf (esl_fcl_nextthr_bf),
        .esl_fcl_stall_bf (esl_fcl_stall_bf),
        .esl_fcl_switch_bf (esl_fcl_switch_bf),
        .esl_fdp_sync_pcs_bf (esl_fdp_sync_pcs_bf),
        .esl_fcl_ntr_s (esl_fcl_ntr_s),
        .esl_fdp_issue_prev_inst_next_s (esl_fdp_issue_prev_inst_next_s),
        .esl_fcl_force_running_s (esl_fcl_force_running_s),
        .esl_pc_va_diff_offset_we (esl_pc_va_diff_offset_we),
        .esl_pc_va_diff_offset_clear (esl_pc_va_diff_offset_clear)
    );

endmodule
