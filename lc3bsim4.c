/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();
void evaluate();
void math(int iR);
void regFile(int iR);
void other(int iR);
int SEXT(int immediateLength, int num);
int isReadyInstruction(int n);
void latchRegs();
void latchNext(int state);
int decipherState(int j5, int j4, int j3, int j2, int j1, int j0);
void setCC(char cc);
int mask1(int bin);
int mask2(int bin);
int mask3(int bin);
int mask4(int bin);
int mask5(int bin);
int mask6(int bin);
int mask8(int bin);
int mask9(int bin);
int mask11(int bin);

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
/* MODIFY: you have to add all your new control signals */
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); }
int GetLSHF1(int *x)         { return(x[LSHF1]); }
/* MODIFY: you can add more Get functions for your new control signals */

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A
   There are two write enable signals, one for each byte. WE0 is used for
   the least significant byte of a word. WE1 is used for the most significant
   byte of a word. */

#define WORDS_IN_MEM    0x08000
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */

/* For lab 4 */
int INTV; /* Interrupt vector register */
int EXCV; /* Exception vector register */
int SSP; /* Initial value of system stack pointer */
/* MODIFY: You may add system latches that are required by your implementation */

} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {

  eval_micro_sequencer();
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {
    int address; /* this is a byte address */

    printf("\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%0.4x..0x%0.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%0.4x (%d) : 0x%0.2x%0.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {
    int k;

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%0.4x\n", BUS);
    printf("MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%0.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%0.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%0.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%0.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%0.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%0.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%0.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : init_memory                                     */
/*                                                             */
/* Purpose   : Zero out the memory array                       */
/*                                                             */
/***************************************************************/
void init_memory() {
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *argv[], int num_prog_files) {
    int i;
    init_control_store(argv[1]);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(argv[i + 2]);
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    CURRENT_LATCHES.SSP = 0x3000; /* Initial value of system stack pointer */

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv, argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code, except for the places indicated
   with a "MODIFY:" comment.

   Do not modify the rdump and mdump functions.

   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/

int nextInstruction = 0;
int SR1Out = 0;
int SR2Out = 0;
int SR2MuxOut = 0;
int SHFOut = 0;
int ALUOut = 0;
int ADDR1MuxOut = 0;
int ADDR2MuxOut = 0;
int PCOut = 0;
int PlusOut = 0;
int MARMuxOut = 0;
int MDRLogicOut = 0;
int MEMOut = 0;
int memCycle = 1;

void eval_micro_sequencer()
{
	if (CURRENT_LATCHES.MICROINSTRUCTION[IRD] == 1)
	{
		nextInstruction = mask4(CURRENT_LATCHES.IR >> 12);
		latchNext(nextInstruction);
	}
	else
	{
		int c0 = CURRENT_LATCHES.MICROINSTRUCTION[COND0];
		int c1 = CURRENT_LATCHES.MICROINSTRUCTION[COND1];
		int bE = CURRENT_LATCHES.BEN;
		int rB = CURRENT_LATCHES.READY;
		int aM = mask1(CURRENT_LATCHES.IR >> 11);
		int j5 = CURRENT_LATCHES.MICROINSTRUCTION[J5];
		int j4 = CURRENT_LATCHES.MICROINSTRUCTION[J4];
		int j3 = CURRENT_LATCHES.MICROINSTRUCTION[J3];
		int j2 = CURRENT_LATCHES.MICROINSTRUCTION[J2];
		int j1 = CURRENT_LATCHES.MICROINSTRUCTION[J1];
		int j0 = CURRENT_LATCHES.MICROINSTRUCTION[J0];
		int branch = ((~c0) & c1 & bE);
		int ready = (c0 & (~c1) & rB);
		int addressMode = (c0 & c1 & aM);
		j2 = j2 | branch;
		j1 = j1 | ready;
		j0 = j0 | addressMode;
		nextInstruction = decipherState(j5, j4, j3, j2, j1, j0);
		latchNext(nextInstruction);
	}
      int iR = CURRENT_LATCHES.IR;
      int n = CURRENT_LATCHES.N;
      int z = CURRENT_LATCHES.Z;
      int p = CURRENT_LATCHES.P;
      NEXT_LATCHES.BEN = ((mask1(iR >> 11) & n) | (mask1(iR >> 10) & z) | (mask1(iR >> 9) & p));
}


void cycle_memory()
{
      if (CURRENT_LATCHES.MICROINSTRUCTION[MIO_EN] == 1)
      {
	     if (memCycle == MEM_CYCLES - 1)
	     {
                 NEXT_LATCHES.READY = 1;
                 memCycle = 0;
	     }
		else
		{
			NEXT_LATCHES.READY = 0;
			memCycle++;
		}
	}
      if (CURRENT_LATCHES.READY == 1)
      {
            if (CURRENT_LATCHES.MICROINSTRUCTION[R_W] == 0)
            {
                  MEMOut = Low16bits(MEMORY[CURRENT_LATCHES.MAR/2][0]);
                  MEMOut |= Low16bits(MEMORY[CURRENT_LATCHES.MAR/2][1] << 8);
            }
            else
            {
                  if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0)
                  {
                        if (mask1(CURRENT_LATCHES.MAR) == 0)
                        {
                              MEMORY[CURRENT_LATCHES.MAR/2][0] = mask8(CURRENT_LATCHES.MDR);
                        }
                        else MEMORY[CURRENT_LATCHES.MAR/2][1] = mask8(CURRENT_LATCHES.MDR >> 8);
                  }
                  else
                  {
                        MEMORY[CURRENT_LATCHES.MAR/2][0] = mask8(CURRENT_LATCHES.MDR);
                        MEMORY[CURRENT_LATCHES.MAR/2][1] = mask8(CURRENT_LATCHES.MDR >> 8);
                  }
            }
      }
  /*
   * This function emulates memory and the WE logic.
   * Keep track of which cycle of MEMEN we are dealing with.
   * If fourth, we need to latch Ready bit at the end of
   * cycle to prepare microsequencer for the fifth cycle.
   */
}



void eval_bus_drivers()
{
	regFile(CURRENT_LATCHES.IR);
	math(CURRENT_LATCHES.IR);
	other(CURRENT_LATCHES.IR);
  /*
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers
   *     Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */
}


void drive_bus()
{
	if (GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION) == 1) BUS = Low16bits(MARMuxOut);
	else if (GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION) == 1) BUS = Low16bits(PCOut);
	else if (GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION) == 1) BUS = Low16bits(ALUOut);
	else if (GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION) == 1) BUS = Low16bits(SHFOut);
	else if (GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION) == 1) BUS = Low16bits(MDRLogicOut);
	else BUS = 0;
  /*
   * Datapath routine for driving the bus from one of the 5 possible
   * tristate drivers.
   */

}


void latch_datapath_values()
{
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_IR] == 1)
	{
		NEXT_LATCHES.IR = Low16bits(BUS);
	}
	else NEXT_LATCHES.IR = CURRENT_LATCHES.IR;

	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_REG] == 1)
	{
		int DRMuxSelect = CURRENT_LATCHES.MICROINSTRUCTION[DRMUX];
		if (DRMuxSelect == 0) NEXT_LATCHES.REGS[mask3(CURRENT_LATCHES.IR >> 9)] = Low16bits(BUS);
		else if (DRMuxSelect == 1) NEXT_LATCHES.REGS[7] = Low16bits(BUS);
	}
	else latchRegs();

  if (CURRENT_LATCHES.MICROINSTRUCTION[LD_PC] == 1)
  {
          int PCMuxSelect = GetPCMUX(CURRENT_LATCHES.MICROINSTRUCTION);
          if (PCMuxSelect == 0) NEXT_LATCHES.PC = Low16bits(CURRENT_LATCHES.PC + 2);
          else if (PCMuxSelect == 1) NEXT_LATCHES.PC = Low16bits(BUS);
          else if (PCMuxSelect == 2) NEXT_LATCHES.PC = Low16bits(PlusOut);
  }
  else NEXT_LATCHES.PC = CURRENT_LATCHES.PC;

  if (CURRENT_LATCHES.MICROINSTRUCTION[LD_CC] == 1)
  {
          if (Low16bits(mask1((BUS) >> 15)) == 1) setCC('N');
          else if (Low16bits(BUS) == 0) setCC('Z');
          else setCC('P');
  }
  else
  {
        NEXT_LATCHES.N = CURRENT_LATCHES.N;
        NEXT_LATCHES.Z = CURRENT_LATCHES.Z;
        NEXT_LATCHES.P = CURRENT_LATCHES.P;
  }

  if (CURRENT_LATCHES.MICROINSTRUCTION[LD_MAR] == 1)
  {
        NEXT_LATCHES.MAR = Low16bits(BUS);
  }
  else NEXT_LATCHES.MAR = CURRENT_LATCHES.MAR;

  if (CURRENT_LATCHES.MICROINSTRUCTION[LD_MDR] == 1)
  {
        if (CURRENT_LATCHES.MICROINSTRUCTION[MIO_EN] == 0)
        {
            if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0)
            {
                  NEXT_LATCHES.MDR = mask8(BUS) + (mask8(BUS) << 8);
            }
            else NEXT_LATCHES.MDR = Low16bits(BUS);
        }
        else NEXT_LATCHES.MDR = MEMOut;
  }
  else NEXT_LATCHES.MDR = CURRENT_LATCHES.MDR;
  /*
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come
   * after drive_bus.
   */

}

void regFile(int iR)
{
	int SR1MUXSelect = CURRENT_LATCHES.MICROINSTRUCTION[SR1MUX];
	if (SR1MUXSelect == 0)
	{
		SR1Out = NEXT_LATCHES.REGS[mask3(iR >> 9)];
	}
	else if (SR1MUXSelect == 1)
	{
		SR1Out = NEXT_LATCHES.REGS[mask3(iR >> 6)];
	}
	SR2Out = NEXT_LATCHES.REGS[mask3(iR)];
}

void math(int iR)
{
	if (mask1(iR >> 5) == 1) SR2MuxOut = Low16bits(SEXT(5, mask5(iR)));
	else if (mask1(iR >> 5) == 0) SR2MuxOut = SR2Out;

	int ALUSelect = GetALUK(CURRENT_LATCHES.MICROINSTRUCTION);
	if (ALUSelect == 0) ALUOut = Low16bits(SR2MuxOut + SR1Out);
	else if (ALUSelect == 1) ALUOut = SR2MuxOut & SR1Out;
	else if (ALUSelect == 2) ALUOut = SR2MuxOut ^ SR1Out;
	else if (ALUSelect == 3) ALUOut = SR1Out;

	if (mask2(iR >> 4) == 0) SHFOut = SR1Out << mask4(iR);
	else if (mask2(iR >> 4) == 1) SHFOut = SR1Out >> mask4(iR);
	else if (mask2(iR >> 4) == 3)
	{
		SHFOut = SR1Out;
		int amountToShift = mask4(iR);
		int signBit = (SR1Out & 0x8000);
          	while (amountToShift > 0)
          	{
            	SHFOut = SHFOut >> 1;
            	SHFOut = SHFOut | signBit;
            	amountToShift--;
          	}
	}
}

void other(int iR)
{
	int ADDR1MuxSelect = GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
	if (ADDR1MuxSelect == 0) ADDR1MuxOut = CURRENT_LATCHES.PC;
	else if (ADDR1MuxSelect == 1) ADDR1MuxOut = SR1Out;

	int ADDR2MuxSelect = GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION);
	if (ADDR2MuxSelect == 0) ADDR2MuxOut = 0;
	else if (ADDR2MuxSelect == 1) ADDR2MuxOut = Low16bits(SEXT(6, mask6(iR)));
	else if (ADDR2MuxSelect == 2) ADDR2MuxOut = Low16bits(SEXT(9, mask9(iR)));
	else if (ADDR2MuxSelect == 3) ADDR2MuxOut = Low16bits(SEXT(11, mask11(iR)));

	int LSHFSelect = GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION);
	if (LSHFSelect == 0) PlusOut = ADDR1MuxOut + ADDR2MuxOut;
	else if (LSHFSelect == 1) PlusOut = ADDR1MuxOut + (ADDR2MuxOut << 1);

	int MARMUXSelect = GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION);
	if (MARMUXSelect == 0) MARMuxOut = Low16bits(mask8(iR) << 1);
	else if (MARMUXSelect == 1) MARMuxOut = PlusOut;

	if (GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 0)
	{
		int MARZero = CURRENT_LATCHES.MAR % 2;
		if (MARZero == 0) MDRLogicOut = SEXT(8, mask8(CURRENT_LATCHES.MDR));
		else if (MARZero == 1) MDRLogicOut = SEXT(8, mask8(CURRENT_LATCHES.MDR >> 8));
	}
	else if (GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION) == 1) MDRLogicOut = CURRENT_LATCHES.MDR;
	PCOut = CURRENT_LATCHES.PC;
}

int SEXT(int immediateLength, int num)
{
  	int tempNum = num;
  	int amountToShift = 16 - immediateLength;
  	int check = (tempNum >> (immediateLength - 1)) & 0x0001;
  	if (check == 0x0000)
	  {
    		return num;
  	}
  	else
  	{
    		int starter = 0x8000;
    		while (amountToShift > 0)
    		{
      		num = num | starter;
      		amountToShift--;
      		starter = starter >> 1;
    		}
    	  return num;
  	}
}

void setCC(char cc)
{
  if (cc == 'N')
  {
    NEXT_LATCHES.N = 1;
    NEXT_LATCHES.Z = 0;
    NEXT_LATCHES.P = 0;
  }
  if (cc == 'Z')
  {
    NEXT_LATCHES.N = 0;
    NEXT_LATCHES.Z = 1;
    NEXT_LATCHES.P = 0;
  }
  if (cc == 'P')
  {
    NEXT_LATCHES.N = 0;
    NEXT_LATCHES.Z = 0;
    NEXT_LATCHES.P = 1;
  }
}
void latchRegs()
{
      memcpy(NEXT_LATCHES.REGS, CURRENT_LATCHES.REGS, sizeof(int)*LC_3b_REGS);
}
void latchNext(int state)
{
      NEXT_LATCHES.STATE_NUMBER = state;
	memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[state], sizeof(int)*CONTROL_STORE_BITS);
}

int decipherState(int j5, int j4, int j3, int j2, int j1, int j0)
{
	return (1 * j0) + (2 * j1) + (4 * j2) + (8 * j3) + (16 * j4) + (32 * j5);
}

int mask1(int bin)
{
	return (bin & 0x0001);
}

int mask2(int bin)
{
	return (bin & 0x0003);
}

int mask3(int bin)
{
	return (bin & 0x0007);
}

int mask4(int bin)
{
	return (bin & 0x000F);
}
int mask5(int bin)
{
	return (bin & 0x001F);
}
int mask6(int bin)
{
	return (bin & 0x003F);
}
int mask8(int bin)
{
	return (bin & 0x00FF);
}
int mask9(int bin)
{
	return (bin & 0x01FF);
}
int mask11(int bin)
{
	return (bin & 0x03FF);
}
