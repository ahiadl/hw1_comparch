/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

// Global variables
bool forwarding;
bool split_reg;
SIM_coreState state;
int mem_ready;

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/

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

int check_memory(SIM_coreState* state_inst){
    if (!mem_ready) return 0;
    mem_ready = SIM_MemDataRead(state_inst->pipeStageState[MEMORY].address, &state_inst->pipeStageState[MEMORY].dstVal);
    return 1;
}

int IF_stage (uint32_t addr, SIM_cmd *dst){
    SIM_MemInstRead(addr+4, dst);
    return addr+4;
}

void ID_stage (SIM_coreState* state_inst, PipeStageState* decode){  
    decode->cmd = state_inst->pipeStageState[FETCH].cmd;
    decode->src1Val = state_inst->regFile[state_inst->pipeStageState[FETCH].cmd.src1];
//    printf ("\nsrc1Val (decode): %d origin: %d ", decode->src1Val, state_inst->pipeStageState[FETCH].cmd.src1);
    decode->src2Val = state_inst->pipeStageState[FETCH].cmd.isSrc2Imm ? state_inst->pipeStageState[FETCH].cmd.src2 : state_inst->regFile[state_inst->pipeStageState[FETCH].cmd.src2];
//    printf ("\nsrc2Val (decode): %d, origin: %d", decode->src2Val,state_inst->pipeStageState[FETCH].cmd.src2 );
    decode->dstVal  = (CMD_STORE == state_inst->pipeStageState[FETCH].cmd.opcode) ? state_inst->regFile[state_inst->pipeStageState[FETCH].cmd.dst] : 0 ;
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
            //next->pc += 4 ;
	        break;
	    case CMD_STORE:
            next->pipeStageState[MEMORY].address = (next->pipeStageState[MEMORY].cmd.isSrc2Imm) ? next->pipeStageState[MEMORY].dstVal + next->pipeStageState[MEMORY].cmd.src2 :  
                    		     							                                          next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].src2Val;
            SIM_MemDataWrite(next->pipeStageState[MEMORY].address, next->pipeStageState[MEMORY].src1Val);
            //next->pc += 4 ;
	        break;
        case CMD_BR:
	    case CMD_BREQ:
	    case CMD_BRNEQ:
            if (next->pipeStageState[MEMORY].br_valid){
	            next->pc = state_inst->pc - 8 + next->regFile[next->pipeStageState[MEMORY].cmd.dst];
		        flush(next);
	        }// else
	           //next->pc = (next->pc) + 4;
	        break;
	     default: 
            //next->pc += 4 ;
            break;
	}
}

void WB_stage (SIM_coreState* state_inst,SIM_coreState* next) {
    next->pipeStageState[WRITEBACK] = state_inst->pipeStageState[MEMORY];
    switch (state_inst->pipeStageState[MEMORY].cmd.opcode){
        case CMD_LOAD:
        case CMD_ADD :
        case CMD_SUB :
            next->regFile[next->pipeStageState[WRITEBACK].cmd.dst] = next->pipeStageState[WRITEBACK].dstVal;  
            break;
        default: break;
    }
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
    reset_state(&state);
    SIM_MemInstRead(state.pc, &state.pipeStageState[FETCH].cmd);
    return 0;
}

void copy_regFile (int32_t* src, int32_t* dst){
    int i;
    for (i=0; i<SIM_REGFILE_SIZE; i++) dst[i]=src[i];
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
    SIM_coreState next_state;
    reset_state(&next_state); 
    copy_regFile (state.regFile, next_state.regFile);
    if (!check_memory(&state)){
    	next_state.pc = IF_stage (state.pc,&next_state.pipeStageState[FETCH].cmd);
    	ID_stage(&state, &next_state.pipeStageState[DECODE]);
//        if (!forwarding) check_for_hazards
        EXE_stage(&state, &next_state.pipeStageState[EXECUTE]);
        MEM_stage(&state, &next_state);
    	WB_stage(&state, &next_state);	
    }else{ 
        next_state = state;
	    insertNOP(&next_state.pipeStageState[WRITEBACK]);
    }
    state = next_state; //TODO: think if should be placed inside the if.
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/


/*void copy_cmd (SIM_cmd* src, SIM_cmd* dst){
    *src=*dst;
}

void copy_pipe_stage (SIM_cmd*/



void SIM_CoreGetState(SIM_coreState *curState) {
   *curState = state;
}

































































































