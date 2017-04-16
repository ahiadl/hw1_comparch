/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"





/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/


/*****************************************************************************************************/
enum {
    READY = 0,
    NOT_READY = -1,
};

enum {
    NO_HAZARD  = 0,
    HAZARD     = 1,
};

enum {
    NOT_STALLED = 0,
    STALLED     = 1,
};


typedef struct {
    SIM_cmd cmd;      /// The processed command in each pipe stage
    int32_t src1Val;  /// Actual value of src1 (considering forwarding mux, etc.)
    int32_t src2Val;  /// Actual value of src2 (considering forwarding mux, etc.)
    int32_t dstVal;   /// Value to be written to dst register.
    bool    br_valid; /// Indicates if a branch should taken for this cmd
    int32_t pc_4;     /// pc (not pc+4) of the cmd
} PipeStageStateInt;

/* A structure to return information about the currect simulator state */
typedef struct {
    int32_t pc; /// Value of the current program counter (at instruction fetch stage)
    int32_t regFile[SIM_REGFILE_SIZE]; /// Values of each register in the register file
    PipeStageStateInt pipeStageState[SIM_PIPELINE_DEPTH];
} SIM_coreStateInt;

// Global variables
SIM_coreStateInt state;
int mem_ready;
int hazard_detected;

/*************************************************************************************************/

void insertNOP (PipeStageStateInt* stage){
    stage->cmd.opcode    = CMD_NOP;
    stage->cmd.src1      = 0;
    stage->cmd.src2      = 0;
    stage->cmd.isSrc2Imm = false;
    stage->cmd.dst       = 0;
    stage->src1Val       = 0;
    stage->src2Val       = 0;
    stage->dstVal        = 0;
    stage->br_valid      = false;
    stage->pc_4          = 0;   
}

void flush (SIM_coreStateInt* state_inst){
    insertNOP(&state_inst->pipeStageState[EXECUTE]);
    insertNOP(&state_inst->pipeStageState[DECODE]);
    insertNOP(&state_inst->pipeStageState[FETCH]);
}

void search_forwarding (SIM_coreStateInt* state_inst){
    int src1 = state_inst->pipeStageState[EXECUTE].cmd.src1;
    int src2 = state_inst->pipeStageState[EXECUTE].cmd.src2;
    int dst  = state_inst->pipeStageState[EXECUTE].cmd.dst;
    int* src1Val = &state_inst->pipeStageState[EXECUTE].src1Val;
    int* src2Val = &state_inst->pipeStageState[EXECUTE].src2Val;
    int* dstVal  = &state_inst->pipeStageState[EXECUTE].dstVal;
    
    switch (state_inst->pipeStageState[EXECUTE].cmd.opcode){
        case CMD_STORE: 
        case CMD_ADD:
        case CMD_SUB:
        case CMD_BREQ:
        case CMD_BRNEQ:
        case CMD_LOAD:
            *src1Val = (src1 == state_inst->pipeStageState[MEMORY].cmd.dst)    ? state_inst->pipeStageState[MEMORY].dstVal    :
                       (src1 == state_inst->pipeStageState[WRITEBACK].cmd.dst) ? state_inst->pipeStageState[WRITEBACK].dstVal : *src1Val;
            *src2Val = (src2 == state_inst->pipeStageState[MEMORY].cmd.dst)    ? state_inst->pipeStageState[MEMORY].dstVal    :
                       (src2 == state_inst->pipeStageState[WRITEBACK].cmd.dst) ? state_inst->pipeStageState[WRITEBACK].dstVal : *src2Val;    
        case CMD_BR:
            *dstVal  = (dst == state_inst->pipeStageState[MEMORY].cmd.dst)    ? state_inst->pipeStageState[MEMORY].dstVal    : 
                       (dst == state_inst->pipeStageState[WRITEBACK].cmd.dst) ? state_inst->pipeStageState[WRITEBACK].dstVal : *dstVal;
            break;
        default:
            break;
    }
}

void branch_exec (SIM_coreStateInt* state_inst){
     if (state_inst->pipeStageState[MEMORY].br_valid){
        hazard_detected = NO_HAZARD;
        mem_ready = READY;
        state_inst->pc = state_inst->pipeStageState[MEMORY].pc_4 + state_inst->regFile[state_inst->pipeStageState[MEMORY].cmd.dst];
        flush(state_inst);
     }
}

void write_to_regfile (SIM_coreStateInt* state_inst){
   switch (state_inst->pipeStageState[WRITEBACK].cmd.opcode){
    case CMD_LOAD:
    case CMD_ADD :
    case CMD_SUB :
        state_inst->regFile[state_inst->pipeStageState[WRITEBACK].cmd.dst] = state_inst->pipeStageState[WRITEBACK].dstVal;  
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

void reset_pipe_stage (PipeStageStateInt* pipe){
    reset_cmd (&pipe->cmd);
    pipe->src1Val  = 0;
    pipe->src2Val  = 0;
    pipe->dstVal   = 0;
    pipe->br_valid = 0;
    pipe->pc_4     = 0;
}

void reset_regFile (int32_t* regFile){
    int i;
    for (i=0; i<SIM_REGFILE_SIZE; i++) regFile[i]=0;
}

void reset_state (SIM_coreStateInt* state_inst){
    state_inst->pc =0;
    reset_regFile(state_inst->regFile);
    int i;
    for (i=0; i<SIM_PIPELINE_DEPTH ; i++) reset_pipe_stage (&state_inst->pipeStageState[i]);
}

void copy_regFile (int32_t* src, int32_t* dst){
    int i;
    for (i=0; i<SIM_REGFILE_SIZE; i++) dst[i]=src[i];
}

int check_depend_on_stage (SIM_coreStateInt* state_inst,int low_stage, int high_stage){
    int var1_vs_high_dst = 0;
    int var2_vs_high_dst = 0;
    int dst_vs_high_dst  = 0;
    int high_stage_cmd = state_inst->pipeStageState[high_stage].cmd.opcode;
    if (high_stage_cmd != CMD_LOAD && high_stage_cmd != CMD_ADD && high_stage_cmd != CMD_SUB) return NO_HAZARD;
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
  return (var1_vs_high_dst || var2_vs_high_dst || dst_vs_high_dst ) ? HAZARD : NO_HAZARD;
}


int check_decode_depend(SIM_coreStateInt* state_inst){
    if (forwarding) return NO_HAZARD;
    int decode_vs_execute   = check_depend_on_stage(state_inst, DECODE, EXECUTE);
    int decode_vs_memory    = check_depend_on_stage(state_inst, DECODE, MEMORY);
    int decode_vs_writeback = split_regfile ? 0 : check_depend_on_stage(state_inst, DECODE, WRITEBACK);
    return (decode_vs_memory || decode_vs_execute || decode_vs_writeback);
}

void shift_backwards_pipe(SIM_coreStateInt* state_inst){
    state_inst->pipeStageState[FETCH] = state_inst->pipeStageState[DECODE];
    state_inst->pipeStageState[DECODE] = state_inst->pipeStageState[EXECUTE];
    state_inst->pipeStageState[EXECUTE] = state_inst->pipeStageState[MEMORY];
    insertNOP(&state_inst->pipeStageState[MEMORY]);
}

int check_load_hazard (SIM_coreStateInt* state_inst){
    if (!(state_inst->pipeStageState[EXECUTE].cmd.opcode == CMD_LOAD)) return NO_HAZARD;
    int src1 = state_inst->pipeStageState[DECODE].cmd.src1;
    int src2 = state_inst->pipeStageState[DECODE].cmd.src2;
    int dst  = state_inst->pipeStageState[DECODE].cmd.dst;
    int dst_exe  = state_inst->pipeStageState[EXECUTE].cmd.dst;
    switch (state_inst->pipeStageState[DECODE].cmd.opcode){
        case CMD_STORE:
        case CMD_BREQ: 
        case CMD_BRNEQ: 
            return ((src1 == dst_exe) || (src2 == dst_exe) || (dst == dst_exe) ? HAZARD : NO_HAZARD);          
        case CMD_LOAD:
        case CMD_ADD:
        case CMD_SUB:
            return ((src1 == dst_exe) || (src2 == dst_exe) ? HAZARD : NO_HAZARD);          
        case CMD_BR:
            return ((dst == dst_exe) ? HAZARD : NO_HAZARD);          
        default:
            break;
    }
    return NO_HAZARD; 
}

/**********************************************************************************************/
/****************************Stages Implementation*********************************************/
/**********************************************************************************************/

int IF_stage (SIM_coreStateInt* state_inst){
    SIM_MemInstRead(state_inst->pc+4,&state_inst->pipeStageState[FETCH].cmd);
    state_inst->pipeStageState[FETCH].pc_4 = state_inst->pc+4;
    return state_inst->pc+4;
}

void ID_stage (SIM_coreStateInt* state_inst, SIM_coreStateInt* next ){  
    next->pipeStageState[DECODE] = state_inst->pipeStageState[FETCH];
    next->pipeStageState[DECODE].src1Val = next->regFile[state_inst->pipeStageState[FETCH].cmd.src1];
    next->pipeStageState[DECODE].src2Val = state_inst->pipeStageState[FETCH].cmd.isSrc2Imm ? state_inst->pipeStageState[FETCH].cmd.src2 : next->regFile[state_inst->pipeStageState[FETCH].cmd.src2];
    next->pipeStageState[DECODE].dstVal  = (CMD_STORE == state_inst->pipeStageState[FETCH].cmd.opcode) ? next->regFile[state_inst->pipeStageState[FETCH].cmd.dst] : 0 ;
}

void EXE_stage(SIM_coreStateInt* state_inst, SIM_coreStateInt* next ){
    next->pipeStageState[EXECUTE] = state_inst->pipeStageState[DECODE];
    if (forwarding) search_forwarding(next);
    switch(next->pipeStageState[EXECUTE].cmd.opcode){
        case CMD_ADD:
	        next->pipeStageState[EXECUTE].dstVal = next->pipeStageState[EXECUTE].src1Val + next->pipeStageState[EXECUTE].src2Val;
	        break;	
	    case CMD_SUB:
            next->pipeStageState[EXECUTE].dstVal = next->pipeStageState[EXECUTE].src1Val - next->pipeStageState[EXECUTE].src2Val;
            break;	
	    case CMD_BREQ:
	        next->pipeStageState[EXECUTE].br_valid = (next->pipeStageState[EXECUTE].src1Val == next->pipeStageState[EXECUTE].src2Val);
	        break;
	    case CMD_BRNEQ:
	        next->pipeStageState[EXECUTE].br_valid = (next->pipeStageState[EXECUTE].src1Val != next->pipeStageState[EXECUTE].src2Val);
	        break;
	    case CMD_BR:
	        next->pipeStageState[EXECUTE].br_valid = true;
            break;
	    default: break;
    }
}

void MEM_stage(SIM_coreStateInt* state_inst, SIM_coreStateInt* next){
    next->pipeStageState[MEMORY] = state_inst->pipeStageState[EXECUTE];
    int address;
    switch(next->pipeStageState[MEMORY].cmd.opcode){
        case CMD_LOAD:
            address = (next->pipeStageState[MEMORY].cmd.isSrc2Imm) ? next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].cmd.src2 :  
             		                                                 next->pipeStageState[MEMORY].src1Val + next->pipeStageState[MEMORY].src2Val;
            mem_ready = SIM_MemDataRead(address, &next->pipeStageState[MEMORY].dstVal);
	        break;
	    case CMD_STORE:
            address = (next->pipeStageState[MEMORY].cmd.isSrc2Imm) ? next->pipeStageState[MEMORY].dstVal + next->pipeStageState[MEMORY].cmd.src2 :  
                  		     							             next->pipeStageState[MEMORY].dstVal + next->pipeStageState[MEMORY].src2Val;
            SIM_MemDataWrite(address, next->pipeStageState[MEMORY].src1Val);
	        break;
	     default: 
            break;
	}
}

void WB_stage (SIM_coreStateInt* state_inst,SIM_coreStateInt* next) {
    next->pipeStageState[WRITEBACK] = state_inst->pipeStageState[MEMORY];
    if (split_regfile) write_to_regfile(next);
}


/**********************************************************************************************/
/******************************Interface Functions*********************************************/
/**********************************************************************************************/


int SIM_CoreReset(void) {
    mem_ready = READY;
    hazard_detected = NO_HAZARD;
    reset_state(&state);
    SIM_MemInstRead(state.pc, &state.pipeStageState[FETCH].cmd);
    return 0;
}

/*******************************************************************************************/

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
    SIM_coreStateInt next_state;                                                    ///////////////////////////////////
    reset_state(&next_state);                                                       ///
    if (!split_regfile) write_to_regfile (&state);                                  ///these lines prepare next_state for working 
    copy_regFile (state.regFile, next_state.regFile);                               ///
    branch_exec(&state);                                                            ///this line flushes the pipe in case of branch  
    next_state.pc = state.pc;                                                       ///////////////////////////////////

    int pipe_stalled = NOT_STALLED;
    if (mem_ready == NOT_READY) {
        shift_backwards_pipe(&state);
        next_state.pc -= 4;
        pipe_stalled = STALLED;
    }
    else if(hazard_detected == HAZARD){                                             // rewind decode stage in case it shouldn't be forwarded (data hazards)
        next_state.pc -= 4;
        state.pipeStageState[FETCH] = state.pipeStageState[DECODE]; 
    }

    WB_stage(&state, &next_state);                                                  ////////////////////////////////
    MEM_stage(&state, &next_state);                                                 ///
    EXE_stage(&state, &next_state);                                                 ///continue the pipe
    ID_stage(&state, &next_state);                                                  ///
    next_state.pc = IF_stage (&next_state);                                         ////////////////////////////////

    
    if((HAZARD == hazard_detected) && (NOT_STALLED == pipe_stalled))                /// if we have data hazard and no load blocking, stall IF and ID stages as well as NOP to EXE
        insertNOP(&next_state.pipeStageState[EXECUTE]);                       
    hazard_detected = (check_decode_depend(&next_state) || check_load_hazard(&next_state)) ? HAZARD : NO_HAZARD;   
    
    state = next_state; 
}


/****************************************************************************************/

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
    int stage = FETCH;
    for (stage ; stage<= WRITEBACK; stage++){
        curState->pipeStageState[stage].src1Val = state.pipeStageState[stage].src1Val;
        curState->pipeStageState[stage].src2Val = state.pipeStageState[stage].src2Val;
        curState->pipeStageState[stage].cmd = state.pipeStageState[stage].cmd;
    }
    curState->pc = state.pc;
    copy_regFile(state.regFile, curState->regFile);   
}


