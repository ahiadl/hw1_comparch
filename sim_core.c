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

void flush (SIM_coreState* state){
    insertNOP(&state->pipeStageState[EXECUTE]);
    insertNOP(&state->pipeStageState[DECODE]);
    insertNOP(&state->pipeStageState[FETCH]);
}

int check_memory(SIM_coreState* state){
    if (!mem_ready) return mem_ready;
    mem_ready = SIM_MemDataRead(state->pipeStageState[MEMORY].address, &state->pipeStageState[MEMORY].dstVal);
    return mem_ready;
}

int IF_stage (uint32_t* addr, SIM_cmd *dst){
    SIM_MemInstRead(*addr, dst);
    return *addr;
}

void ID_stage (SIM_coreState* state, PipeStageState* decode){  
    decode->cmd = state->pipeStageState[FETCH].cmd;
    decode->src1Val = state->regFile[state->pipeStageState[FETCH].cmd.src1];
    decode->src2Val = state->pipeStageState[FETCH].cmd.isSrc2Imm ? 0 : state->regFile[state->pipeStageState[FETCH].cmd.src2]; //TODO: think if default should be 0
    decode->dstVal  = (CMD_STORE == state->pipeStageState[FETCH].cmd.opcode) ? state->regFile[state->pipeStageState[FETCH].cmd.dst] : 0 ;
}

void EXE_stage(SIM_coreState* state, PipeStageState* execute){
    execute->cmd = state->pipeStageState[DECODE].cmd; 
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


void MEM_stage(SIM_coreState* state, SIM_coreState* next){
    next->pipeStageState[MEMORY].cmd = state->pipeStageState[EXECUTE].cmd;
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
        case CMD_BR:
	    case CMD_BREQ:
	    case CMD_BRNEQ:
            if (next->pipeStageState[MEMORY].br_valid){
	            next->pc = state->pc - 8 + next->regFile[next->pipeStageState[MEMORY].cmd.dst];
		        flush(next);
	        } else
	            next->pc = (next->pc) + 4;
	        break;
	     default: break;
	}
}

void WB_stage (SIM_coreState* state,SIM_coreState* next) {
    next->pipeStageState[WRITEBACK] = state->pipeStageState[MEMORY];
    switch (state->pipeStageState[MEMORY].cmd.opcode){
        case CMD_LOAD:
        case CMD_ADD :
        case CMD_SUB :
            next->regFile[next->pipeStageState[WRITEBACK].cmd.dst] = next->pipeStageState[WRITEBACK].dstVal;  
            break;
        default: break;
    }
}

int SIM_CoreReset(void) {

}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
    SIM_coreState next_state;
    if (check_memory(&state)){
    	next_state.pc = IF_stage (&state.pc,&next_state.pipeStageState[FETCH].cmd);
    	ID_stage(&state, &next_state.pipeStageState[DECODE]);
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
void SIM_CoreGetState(SIM_coreState *curState) {
}

































































































