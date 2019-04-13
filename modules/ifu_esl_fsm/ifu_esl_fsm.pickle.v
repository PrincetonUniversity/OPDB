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
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: iop.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
//-*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
//  Description:	Global header file that contain definitions that 
//                      are common/shared at the IOP chip level
*/
////////////////////////////////////////////////////////////////////////


// Address Map Defines
// ===================




// CMP space



// IOP space




                               //`define ENET_ING_CSR     8'h84
                               //`define ENET_EGR_CMD_CSR 8'h85















// L2 space



// More IOP space





//Cache Crossbar Width and Field Defines
//======================================













































//bits 133:128 are shared by different fields
//for different packet types.
























//`define CPX_INV_PA_HI   116
//`define CPX_INV_PA_LO   112






// cache invalidation format
// `define CPX_INV_DCACHE_WORD0_VAL 0
// `define CPX_INV_ICACHE_WORD0_VAL 1
// `define CPX_INV_WORD0_WAY 5:2
// `define CPX_INV_DCACHE_WORD0_VAL 6
// `define CPX_INV_ICACHE_WORD0_VAL 7
// `define CPX_INV_WORD0_WAY 11:8
// `define CPX_INV_DCACHE_WORD0_VAL 12
// // `define CPX_INV_ICACHE_WORD0_VAL 13
// `define CPX_INV_WORD0_WAY 17:14
// `define CPX_INV_DCACHE_WORD0_VAL 18
// // `define CPX_INV_ICACHE_WORD0_VAL 19
// `define CPX_INV_WORD0_WAY 23:20




// 4 extra bits for bigger icache/dcache
// up to 512KB l1 icache, 256KB l1 dcache

































//Pico defines













//End cache crossbar defines


// Number of COS supported by EECU 



// 
// BSC bus sizes
// =============
//

// General




// CTags













// reinstated temporarily




// CoS






// L2$ Bank



// L2$ Req













// L2$ Ack








// Enet Egress Command Unit














// Enet Egress Packet Unit













// This is cleaved in between Egress Datapath Ack's








// Enet Egress Datapath
















// In-Order / Ordered Queue: EEPU
// Tag is: TLEN, SOF, EOF, QID = 15






// Nack + Tag Info + CTag




// ENET Ingress Queue Management Req












// ENET Ingress Queue Management Ack








// Enet Ingress Packet Unit












// ENET Ingress Packet Unit Ack







// In-Order / Ordered Queue: PCI
// Tag is: CTAG





// PCI-X Request











// PCI_X Acknowledge











//
// BSC array sizes
//================
//












// ECC syndrome bits per memory element




//
// BSC Port Definitions
// ====================
//
// Bits 7 to 4 of curr_port_id








// Number of ports of each type


// Bits needed to represent above


// How wide the linked list pointers are
// 60b for no payload (2CoS)
// 80b for payload (2CoS)

//`define BSC_OBJ_PTR   80
//`define BSC_HD1_HI    69
//`define BSC_HD1_LO    60
//`define BSC_TL1_HI    59
//`define BSC_TL1_LO    50
//`define BSC_CT1_HI    49
//`define BSC_CT1_LO    40
//`define BSC_HD0_HI    29
//`define BSC_HD0_LO    20
//`define BSC_TL0_HI    19
//`define BSC_TL0_LO    10
//`define BSC_CT0_HI     9
//`define BSC_CT0_LO     0


































// I2C STATES in DRAMctl







//
// IOB defines
// ===========
//



















//`define IOB_INT_STAT_WIDTH   32
//`define IOB_INT_STAT_HI      31
//`define IOB_INT_STAT_LO       0

















































// fixme - double check address mapping
// CREG in `IOB_INT_CSR space










// CREG in `IOB_MAN_CSR space





































// Address map for TAP access of SPARC ASI













//
// CIOP UCB Bus Width
// ==================
//
//`define IOB_EECU_WIDTH       16  // ethernet egress command
//`define EECU_IOB_WIDTH       16

//`define IOB_NRAM_WIDTH       16  // NRAM (RLDRAM previously)
//`define NRAM_IOB_WIDTH        4




//`define IOB_ENET_ING_WIDTH   32  // ethernet ingress
//`define ENET_ING_IOB_WIDTH    8

//`define IOB_ENET_EGR_WIDTH    4  // ethernet egress
//`define ENET_EGR_IOB_WIDTH    4

//`define IOB_ENET_MAC_WIDTH    4  // ethernet MAC
//`define ENET_MAC_IOB_WIDTH    4




//`define IOB_BSC_WIDTH         4  // BSC
//`define BSC_IOB_WIDTH         4







//`define IOB_CLSP_WIDTH        4  // clk spine unit
//`define CLSP_IOB_WIDTH        4





//
// CIOP UCB Buf ID Type
// ====================
//



//
// Interrupt Device ID
// ===================
//
// Caution: DUMMY_DEV_ID has to be 9 bit wide
//          for fields to line up properly in the IOB.



//
// Soft Error related definitions 
// ==============================
//



//
// CMP clock
// =========
//




//
// NRAM/IO Interface
// =================
//










//
// NRAM/ENET Interface
// ===================
//







//
// IO/FCRAM Interface
// ==================
//






//
// PCI Interface
// ==================
// Load/store size encodings
// -------------------------
// Size encoding
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 100 - quad






//
// JBI<->SCTAG Interface
// =======================
// Outbound Header Format



























// Inbound Header Format




















//
// JBI->IOB Mondo Header Format
// ============================
//














// JBI->IOB Mondo Bus Width/Cycle
// ==============================
// Cycle  1 Header[15:8]
// Cycle  2 Header[ 7:0]
// Cycle  3 J_AD[127:120]
// Cycle  4 J_AD[119:112]
// .....
// Cycle 18 J_AD[  7:  0]


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
 
// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: sys.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// -*- verilog -*-
////////////////////////////////////////////////////////////////////////
/*
//
// Description:		Global header file that contain definitions that 
//                      are common/shared at the systme level
*/
////////////////////////////////////////////////////////////////////////
//
// Setting the time scale
// If the timescale changes, JP_TIMESCALE may also have to change.
`timescale	1ps/1ps
`default_nettype wire

//
// Number of threads in a core
// ===========================
//

//`define CONFIG_NUM_THREADS // This must be defined for any of below to work
//`define THREADS_1
//`define THREADS_2
//`define THREADS_3


//
// JBUS clock
// =========
//
// `define SYSCLK_PERIOD   5000


// Afara Link Defines
// ==================

// Reliable Link




// Afara Link Objects


// Afara Link Object Format - Reliable Link










// Afara Link Object Format - Congestion



  







// Afara Link Object Format - Acknowledge











// Afara Link Object Format - Request

















// Afara Link Object Format - Message



// Acknowledge Types




// Request Types





// Afara Link Frame



//
// UCB Packet Type
// ===============
//

















//
// UCB Data Packet Format
// ======================
//






























// Size encoding for the UCB_SIZE_HI/LO field
// 000 - byte
// 001 - half-word
// 010 - word
// 011 - double-word
// 111 - quad-word







//
// UCB Interrupt Packet Format
// ===========================
//










//`define UCB_THR_HI             9      // (6) cpu/thread ID shared with
//`define UCB_THR_LO             4             data packet format
//`define UCB_PKT_HI             3      // (4) packet type shared with
//`define UCB_PKT_LO             0      //     data packet format







//
// FCRAM Bus Widths
// ================
//






//
// ENET clock periods
// ==================
//
// `define AXGRMII_CLK_PERIOD          6400 // 312.5MHz/2
// `define ENET_GMAC_CLK_PERIOD        8000 // 125MHz


//
// JBus Bridge defines
// =================
//
// `define      SYS_UPA_CLK        `SYS.upa_clk
// `define      SYS_J_CLK          `SYS.j_clk
// `define      SYS_P_CLK          `SYS.p_clk
// `define      SYS_G_CLK          `SYS.g_clk
// `define      JP_TIMESCALE       `timescale 1 ps / 1 ps
// `define      PCI_CLK_PERIOD     15152                  //  66 MHz
// `define      UPA_RD_CLK_PERIOD  6666                   // 150 MHz
// `define      UPA_REF_CLK_PERIOD 7576                   // 132 MHz
// `define      ICHIP_CLK_PERIOD   30304                  //  33 MHz


//
// PCI Device Address Configuration
// ================================
//























// Modified by Princeton University on June 9th, 2015
/*
* ========== Copyright Header Begin ==========================================
* 
* OpenSPARC T1 Processor File: tlu.h
* Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
* 
* The above named program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License version 2 as published by the Free Software Foundation.
* 
* The above named program is distributed in the hope that it will be 
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
* 
* You should have received a copy of the GNU General Public
* License along with this work; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
* 
* ========== Copyright Header End ============================================
*/
// ifu trap types





















//
// modified for hypervisor support
//
























//


// modified due to bug 2588
// `define	TSA_PSTATE_VRANGE2_LO 16 


//











//
// added due to Niagara SRAMs methodology
// The following defines have been replaced due
// the memory macro replacement from:
// bw_r_rf32x144 -> 2x bw_r_rf32x80
/*
`define	TSA_MEM_WIDTH     144 
`define	TSA_HTSTATE_HI    142 //  3 bits 
`define	TSA_HTSTATE_LO    140 
`define	TSA_TPC_HI        138 // 47 bits 
`define	TSA_TPC_LO         92
`define	TSA_TNPC_HI        90 // 47 bits
`define	TSA_TNPC_LO        44 
`define	TSA_TSTATE_HI      40 // 29 bits 
`define	TSA_TSTATE_LO      12 
`define	TSA_TTYPE_HI        8 //  9 bits
`define	TSA_TTYPE_LO        0
`define	TSA_MEM_CWP_LO	   12
`define	TSA_MEM_CWP_HI	   14
`define	TSA_MEM_PSTATE_LO  15
`define	TSA_MEM_PSTATE_HI  22
`define	TSA_MEM_ASI_LO	   23
`define	TSA_MEM_ASI_HI	   30
`define	TSA_MEM_CCR_LO	   31
`define	TSA_MEM_CCR_HI	   38
`define	TSA_MEM_GL_LO	   39 
`define	TSA_MEM_GL_HI	   40 
*/











//











// HPSTATE position definitions within wsr






// TSTATE postition definitions within wsr







// modified due to bug 2588


// added for bug 2584 




//







//
// tick_cmp and stick_cmp definitions





//
// PIB WRAP



// HPSTATE postition definitions






// HTBA definitions




// TBA definitions




















//
// added for the hypervisor support


// modified due to bug 2588
















//
// compressed PSTATE WSR definitions














//
// ASI_QUEUE for hypervisor
// Queues are: CPU_MONODO
//             DEV_MONODO
//             RESUMABLE_ERROR
//             NON_RESUMABLE_ERROR
//







// for address range checking
















//
// Niagara scratch-pads
// VA address of 0x20 and 0x28 are exclusive to hypervisor
// 







//
// range checking 







// PIB related definitions
// Bit definition for events









// 
// PIB related definitions
// PCR and PIC address definitions



// 
// PCR bit definitions







//









// PIC definitions








// PIC  mask bit position definitions










// added define from sparc_tlu_int.v 










//
// shadow scan related definitions 

// modified due to logic redistribution
// `define TCL_SSCAN_WIDTH 12 





// `define TCL_SSCAN_LO 51 




// 
// position definitions - TDP






// 
// position definitions - TCL




// 
// To speedup POR for verification purposes

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

//==================================================================================================
//  Filename      : jtag.vh
//  Created On    : 2014-01-31 12:52:57
//  Last Modified : 2015-01-28 16:54:05
//  Revision      :
//  Author        : Tri Nguyen
//  Company       : Princeton University
//  Email         : trin@princeton.edu
//
//  Description   : Parallel JTAG/debug controller defines
//==================================================================================================


/////////////////////////////////
// Chip ID in JTAG
/////////////////////////////////







/////////////////////////////////
// JTAG TAP
/////////////////////////////////
// copied from ctu.h








// `define TAP_CREG_ADDR          6'h08
// `define TAP_CREG_WDATA         6'h09
// `define TAP_CREG_RDATA         6'h0a
// `define TAP_CREG_SCRATCH       6'h0b
// `define TAP_IOB_WR             6'h0c
// `define TAP_IOB_RD             6'h0d
// `define TAP_IOB_WADDR          6'h0e
// `define TAP_IOB_WDATA          6'h0f
// `define TAP_IOB_RADDR          6'h10




// `define TAP_CREG_SCRATCH       6'h0b
// `define TAP_IOB_WR             6'h0c
// `define TAP_IOB_RD             6'h0d
// `define TAP_IOB_WADDR          6'h0e
// `define TAP_IOB_WDATA          6'h0f
// `define TAP_IOB_RADDR          6'h10

// `define TAP_MBIST_SERIAL       6'h14
// `define TAP_MBIST_PARALLEL     6'h15
// `define TAP_MBIST_RESULT       6'h16
// `define TAP_MBIST_ABORT        6'h17

// `define TAP_PLL_BYPASS         6'h18

// `define TAP_CLK_STOP_ID        6'h1a
// `define TAP_CLK_SEL            6'h1b  //mask ff00 for ck src

// `define TAP_SSCAN_T0           6'h1c
// `define TAP_SSCAN_T1           6'h1d
// `define TAP_SSCAN_T2           6'h1e
// `define TAP_SSCAN_T3           6'h1f

// `define TAP_SCAN_PARALLEL      6'h20
// `define TAP_SCAN_SERIAL        6'h21
// `define TAP_SCAN_MTEST_LONG    6'h22
// `define TAP_SCAN_MTEST_SHORT   6'h23
// `define TAP_SCAN_BYPASS_EN     6'h24
// `define TAP_SCAN_NSTEP         6'h25
// `define TAP_SCAN_DUMP          6'h26

// `define TAP_EFC_READ           6'h28 
// `define TAP_EFC_BYPASS_DATA    6'h29 
// `define TAP_EFC_BYPASS         6'h2a 
// `define TAP_EFC_READ_MODE      6'h2b 
// `define TAP_EFC_COL_ADDR       6'h2c
// `define TAP_EFC_ROW_ADDR       6'h2d
// `define TAP_EFC_DEST_SAMPLE    6'h2e





/////////////////////////////////
// CTAP register select defines
/////////////////////////////////

// `define CTAP_DATA_REG_WIDTH 64






/////////////////////////////////
// JTAG instructions
/////////////////////////////////

// header


// lengths of header





// definitions of operations
// `define JTAG_REQ_OP_READ_SHADOWSCAN 8'd1

// `define JTAG_REQ_OP_STALL_CORE 8'd3


// `define JTAG_REQ_OP_WRITE_PC 8'd6
// `define JTAG_REQ_OP_WRITE_THREADSTATE 8'd7
// `define JTAG_REQ_OP_CPX_INTERRUPT 8'd8








// definitions of misc field for read/write rtap











// `define JTAG_RTAP_ID__REG 16'd


// definitions of tileids
// `define CTAP_ID_BROADCAST 6'b111111



// mask of header




// misc is used for stall bit at bit 0

// address reg





// masks in address reg




// data reg



// // From CTAP to RTAP
// // these valid vector assumes 4b bus. so 32b vector would be 128b
// `define CTAP_REQ_VEC_WHOLE_PACKET 32'hffffffff
// // header has first 32b
// `define CTAP_REQ_VEC_HEADER 32'h000000ff
// // half is 64b, includes the addresses
// `define CTAP_REQ_VEC_HALF 32'h0000ffff





// RTAP returns






// RTAP states












// ORAM specifics



/////////////////////////////////
// UCB related
/////////////////////////////////




// CTAP_UCB_TILEID_MASK

// RTAP_INSTRUCTION_MASK
// RTAP_INSTRUCTION_RETURN_SHADOWSCAN
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
