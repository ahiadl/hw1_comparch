/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

// Global variables
SIM_coreState state;
int mem_ready;
int decode_depend_on_pipe;
/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
enum {
    READY = 0,
    NOT_READY = -1,
};

enum {
    NOT_DEPEND = 0,
    DEPEND     = 1,
};

enum {
    NOT_STALLED = 0,
    STALLED     = 1,
};

#define DEBUG 0


void insertNOP (PipeStageState* stage){
    stage->cmd.opcode    = CMD_NOP;
    stage->cmd.src1      = 0;
    stage->cmd.src2      = 0;
    stage->cmd.isSrc2Imm = false;
    stage->cmd.dst       = 0;
    stage->src1Val       = 0;
    stage->src2Val       = 0;
    stage->dstVal        = 0;
    stage->br_valid      = false;
}

void flush (SIM_coreState* state_inst){
    insertNOP(&state_inst->pipeStageState[EXECUTE]);
    insertNOP(&state_inst->pipeStageState[DECODE]);
    insertNOP(&state_inst->pipeStageState[FETCH]);
}

int IF_stage (SIM_coreState* state_inst, SIM_cmd *dst){
    SIM_MemInstRead(state_inst->pc+4, dst);
    state_inst->pipeStageState[FETCH].pc_4 = state_inst->pc+4;
    if(DEBUG) printf("IF_stage: pc_4: 0x%x, pc: 0x%x\n",state_inst->pipeStageState[FETCH].pc_4, state_inst->pc);
    return state_inst->pc+4;
}

void ID_stage (SIM_coreState* state_inst, SIM_coreState* next ){  
    next->pipeStageState[DECODE] = state_inst->pipeStageState[FETCH];
    next->pipeStageState[DECODE].src1Val = next->regFile[state_inst->pipeStageState[FETCH].cmd.src1];
    next->pipeStageState[DECODE].src2Val = state_inst->pipeStageState[FETCH].cmd.isSrc2Imm ? state_inst->pipeStageState[FETCH].cmd.src2 : next->regFile[state_inst->pipeStageState[FETCH].cmd.src2];
    next->pipeStageState[DECODE].dstVal  = (CMD_STORE == state_inst->pipeStageState[FETCH].cmd.opcode) ? next->regFile[state_inst->pipeStageState[FETCH].cmd.dst] : 0 ;
}

void EXE_stage(SIM_coreState* state_inst, PipeStageState* execute){
    *execute = state_inst->pipeStageState[DECODE]; 
    switch(execute->cmd.opcode){
        case CMD_ADD:
	        execute->dstVal = execute->src1Val + execute->src2Val;
	        break;	
	    case CMD_SUB:
            execute->dstVal = execute->src1Val - execute->src2Val;
            break;	
	    case CMD_BREQ:
	        execute->br_valid = (execute->src1Val==execute->src2Val);
	        break;
	    case CMD_BRNEQ:
	        execute->br_valid = (execute->src1Val!=execute->src2Val);
	        break;
	    case CMD_BR:
	        execute->br_valid = true;
            break;
	    default: break;
    }
}


void MEM_stage(SIM_coreState* state_inst, SIM_coreState* next){
    next->pipeStageState[MEMORY] = state_inst->pipeStageState[EXECUTE];
    next->pipeStageState[MEMORY].address = (next->pipeStageState[MEMORY].cmd.isSrc2Imm) ? next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].cmd.src2 :  
                       		     							                              next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].src2Val;
    switch(next->pipeStageState[MEMORY].cmd.opcode){
        case CMD_LOAD:
            next->pipeStageState[MEMORY].address = (next->pipeStageState[MEMORY].cmd.isSrc2Imm) ? next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].cmd.src2 :  
               		     							                                              next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].src2Val;
            mem_ready = SIM_MemDataRead(next->pipeStageState[MEMORY].address, &next->pipeStageState[MEMORY].dstVal);
	        break;
	    case CMD_STORE:
            next->pipeStageState[MEMORY].address = (next->pipeStageState[MEMORY].cmd.isSrc2Imm) ? next->pipeStageState[MEMORY].dstVal + next->pipeStageState[MEMORY].cmd.src2 :  
                  		     							                                          next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].src2Val;
            SIM_MemDataWrite(next->pipeStageState[MEMORY].address, next->pipeStageState[MEMORY].src1Val);
	        break;
	     default: 
            break;
	}
}

void branch_exec (SIM_coreState* state_inst){
    if(DEBUG) printf("pc_4 values: IF 0x%x ID 0x%x EXE 0x%x MEM 0x%x WB 0x%x \n", state_inst->pipeStageState[FETCH].pc_4,state_inst->pipeStageState[DECODE].pc_4,state_inst->pipeStageState[EXECUTE].pc_4,state_inst->pipeStageState[MEMORY].pc_4,state_inst->pipeStageState[WRITEBACK].pc_4);
     if (state_inst->pipeStageState[MEMORY].br_valid){
        decode_depend_on_pipe = 0;
        mem_ready = READY;
        state_inst->pc = state_inst->pipeStageState[MEMORY].pc_4 + state_inst->regFile[state_inst->pipeStageState[MEMORY].cmd.dst];
        if(DEBUG) printf ("pc_4: 0x%x; dst: 0x%x\n", state_inst->pipeStageState[MEMORY].pc_4,  state_inst->regFile[state_inst->pipeStageState[MEMORY].cmd.dst]);
        if(DEBUG) printf ("branch address: 0x%x;\n", state_inst->pc);
        flush(state_inst);
        if(DEBUG) printf ("flushed!\n");
     }
}


void write_to_regfile (SIM_coreState* state_inst){
   switch (state_inst->pipeStageState[WRITEBACK].cmd.opcode){
    case CMD_LOAD:
    case CMD_ADD :
    case CMD_SUB :
        state_inst->regFile[state_inst->pipeStageState[WRITEBACK].cmd.dst] = state_inst->pipeStageState[WRITEBACK].dstVal;  
        break;
    default: break;
    }
}

void WB_stage (SIM_coreState* state_inst,SIM_coreState* next) {
    next->pipeStageState[WRITEBACK] = state_inst->pipeStageState[MEMORY];
    if (split_regfile) write_to_regfile(next);
}

void reset_cmd (SIM_cmd* cmd){
    cmd->opcode = CMD_NOP;
    cmd->src1 = 0;
    cmd->src2 = 0;
    cmd->isSrc2Imm = false;
    cmd->dst = 0;
}

void reset_pipe_stage (PipeStageState* pipe){
    reset_cmd (&pipe->cmd);
    pipe->src1Val = 0;
    pipe->src2Val = 0;
    pipe->dstVal  = 0;
    pipe->br_valid = 0;
    pipe->address = 0;
    pipe->pc_4    = 0;
}

void reset_regFile (int32_t* regFile){
    int i;
    for (i=0; i<SIM_REGFILE_SIZE; i++) regFile[i]=0;
}

void reset_state (SIM_coreState* state_inst){
    state_inst->pc =0;
    reset_regFile(state_inst->regFile);
    int i;
    for (i=0; i<SIM_PIPELINE_DEPTH ; i++) reset_pipe_stage (&state_inst->pipeStageState[i]);
}

int SIM_CoreReset(void) {
    mem_ready = 0;
    decode_depend_on_pipe = 0;
    reset_state(&state);
    SIM_MemInstRead(state.pc, &state.pipeStageState[FETCH].cmd);
    return 0;
}

void copy_regFile (int32_t* src, int32_t* dst){
    int i;
    for (i=0; i<SIM_REGFILE_SIZE; i++) dst[i]=src[i];
}

int check_depend_on_stage (SIM_coreState* state_inst,int low_stage, int high_stage){
    int var1_vs_high_dst = 0;
    int var2_vs_high_dst = 0;
    int dst_vs_high_dst  = 0;
    int high_stage_cmd = state_inst->pipeStageState[high_stage].cmd.opcode;
    if (high_stage_cmd != CMD_LOAD && high_stage_cmd != CMD_ADD && high_stage_cmd != CMD_SUB) return NOT_DEPEND;
    switch (state_inst->pipeStageState[low_stage].cmd.opcode){                         
     case CMD_BR:
         var1_vs_high_dst = (state_inst->pipeStageState[low_stage].cmd.dst == state_inst->pipeStageState[high_stage].cmd.dst);
         break;
     case CMD_BREQ:
     case CMD_BRNEQ:
     case CMD_STORE:
        dst_vs_high_dst = (state_inst->pipeStageState[low_stage].cmd.dst == state_inst->pipeStageState[high_stage].cmd.dst);
     case CMD_ADD:
     case CMD_SUB:
     case CMD_LOAD:
         var1_vs_high_dst = (state_inst->pipeStageState[low_stage].cmd.src1 == state_inst->pipeStageState[high_stage].cmd.dst);
         var2_vs_high_dst =  state_inst->pipeStageState[low_stage].cmd.isSrc2Imm ? 0 : (state_inst->pipeStageState[low_stage].cmd.src2 == state_inst->pipeStageState[high_stage].cmd.dst);
         break;
     default: 
        break;
  }
  return (var1_vs_high_dst || var2_vs_high_dst || dst_vs_high_dst ) ? DEPEND : NOT_DEPEND;
}


int check_decode_depend(SIM_coreState* state_inst){
    int decode_vs_execute   = check_depend_on_stage(state_inst, DECODE, EXECUTE);
    int decode_vs_memory    = check_depend_on_stage(state_inst, DECODE, MEMORY);
    int decode_vs_writeback = split_regfile ? 0 : check_depend_on_stage(state_inst, DECODE, WRITEBACK);
    return (decode_vs_memory || decode_vs_execute || decode_vs_writeback);
}

void shift_backwards_pipe(SIM_coreState* state_inst){
    state_inst->pipeStageState[FETCH] = state_inst->pipeStageState[DECODE];
    state_inst->pipeStageState[DECODE] = state_inst->pipeStageState[EXECUTE];
    state_inst->pipeStageState[EXECUTE] = state_inst->pipeStageState[MEMORY];
    insertNOP(&state_inst->pipeStageState[MEMORY]);
}


/*******************************************************************************************/
/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
    if(DEBUG) printf("\n------------new_tick--------------\n");
   
    SIM_coreState next_state;                                                       ////////////////////////////////////
    reset_state(&next_state);                                                       ///
    if (!split_regfile) write_to_regfile (&state);                                  ///these lines prepare next_state for working 
    copy_regFile (state.regFile, next_state.regFile);                               ///
    branch_exec(&state);                                                            ///this line flushes the pipe in case of branch  
    next_state.pc = state.pc;                                                       ///////////////////////////////////


    int pipe_stalled = NOT_STALLED;
    if(DEBUG) printf("246 next.pc: 0x%x \n", next_state.pc);                                  // if we're still waiting for memory response take the pipe bacwards in order to be forwarded and update
    if (mem_ready == NOT_READY) {
        shift_backwards_pipe(&state);
        next_state.pc -= 4;
        pipe_stalled = STALLED;
    }
    else if(decode_depend_on_pipe){                                                 // rewind decode stage in case it shouldn't be forwarded (data hazards)
        next_state.pc -= 4;
        state.pipeStageState[FETCH] = state.pipeStageState[DECODE]; 
    }
    if(DEBUG) printf ("mem_ready: %d, ddop: %d\n", mem_ready, decode_depend_on_pipe); 
    if(DEBUG) printf("257 next.pc: 0x%x \n", next_state.pc);
    WB_stage(&state, &next_state);                                                  ////////////////////////////////
    MEM_stage(&state, &next_state);                                                 ///
    EXE_stage(&state, &next_state.pipeStageState[EXECUTE]);                         ///continue the pipe
    ID_stage(&state, &next_state);                                                  ///
    next_state.pc = IF_stage (&next_state,&next_state.pipeStageState[FETCH].cmd);   ////////////////////////////////
    if(DEBUG) printf("263 next.pc: 0x%x \n", next_state.pc); 
    if(decode_depend_on_pipe && NOT_STALLED == pipe_stalled){                       /// if we have data hazard and no load blocking, stall IF and ID stages as well as NOP to EXE
            insertNOP(&next_state.pipeStageState[EXECUTE]);                         
    }

    if(check_decode_depend(&next_state)){   
        decode_depend_on_pipe = DEPEND;
    }
    else
        decode_depend_on_pipe = NOT_DEPEND;
    if(DEBUG) printf("273 next.pc: 0x%x \n", next_state.pc); 
    state = next_state; 
}


/****************************************************************************************/

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
   *curState = state;
}


