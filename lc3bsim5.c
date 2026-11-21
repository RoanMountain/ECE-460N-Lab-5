/*
    Name 1: Alex Vincent
    UTEID 1: ARV2599
*/

/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N - Lab 5                                           */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         pagetable    page table in LC-3b machine language   */
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
    DRMUX0,
    SR1MUX0,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
/* MODIFY: you have to add all your new control signals */
    /* Lab 4 signals. They are appended after the Lab 3 bits so the first
   35 columns of ucode4 line up with ucode3. */
    COND2,        /* COND is now 3 bits: COND2,COND1,COND0               */
    DRMUX1,       /* DRMUX is now 2 bits: 0=IR[11:9] 1=R7 2=R6           */
    SR1MUX1,      /* SR1MUX is now 2 bits: 0=IR[11:9] 1=IR[8:6] 2=R6     */
    LD_PSR,       /* load PSR (privilege bit and condition codes)        */
    PSRMUX,       /* 0: PSR <- BUS          1: PSR[15] <- 0              */
    GATE_PSR,     /* drive PSR onto the bus                              */
    GATE_SP,      /* drive SPMUX onto the bus                            */
    SPMUX1, SPMUX0, /* 0=SR1+2 1=SR1-2 2=Saved SSP 3=Saved USP           */
    LD_SSP,       /* Saved SSP <- SR1                                    */
    LD_USP,       /* Saved USP <- SR1                                    */
    GATE_PCM2,    /* drive PC-2 onto the bus                             */
    GATE_VECTOR,  /* drive x0200 + (EXCV << 1) onto the bus              */
    LD_EXCV,      /* load the vector register EXCV                       */
    EXCVMUX,      /* 0: exception logic output  1: INTV                  */
    CLR_INT,      /* clear the pending interrupt                         */
    /* Lab 5 signals. They are appended after the Lab 4 bits so the first
    51 columns of ucode 5 line up with ucode 4. */
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND2] << 2) + (x[COND1] << 1) + x[COND0]); }
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
int GetDRMUX(int *x)         { return((x[DRMUX1] << 1) + x[DRMUX0]); }
int GetSR1MUX(int *x)        { return((x[SR1MUX1] << 1) + x[SR1MUX0]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); }
int GetLSHF1(int *x)         { return(x[LSHF1]); }
/* MODIFY: you can add more Get functions for your new control signals */
int GetLD_PSR(int *x)        { return(x[LD_PSR]); }
int GetPSRMUX(int *x)        { return(x[PSRMUX]); }
int GetGATE_PSR(int *x)      { return(x[GATE_PSR]); }
int GetGATE_SP(int *x)       { return(x[GATE_SP]); }
int GetSPMUX(int *x)         { return((x[SPMUX1] << 1) + x[SPMUX0]); }
int GetLD_SSP(int *x)        { return(x[LD_SSP]); }
int GetLD_USP(int *x)        { return(x[LD_USP]); }
int GetGATE_PCM2(int *x)     { return(x[GATE_PCM2]); }
int GetGATE_VECTOR(int *x)   { return(x[GATE_VECTOR]); }
int GetLD_EXCV(int *x)       { return(x[LD_EXCV]); }
int GetEXCVMUX(int *x)       { return(x[EXCVMUX]); }
int GetCLR_INT(int *x)       { return(x[CLR_INT]); }
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

#define WORDS_IN_MEM    0x2000 /* 32 frames */
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
/* MODIFY: you should add here any other registers you need to implement interrupts and exceptions */
int USP;  /* Saved user stack pointer while in supervisor mode */
int PRIV; /* PSR[15]: 0 = supervisor, 1 = user. PSR[2:0] are N, Z, P above */
int INT;  /* Interrupt pending (set by the timer, cleared by CLR_INT) */

/* For lab 5 */
int PTBR; /* This is initialized when we load the page table */
int VA;   /* Temporary VA register */
/* MODIFY: you should add here any other registers you need to implement virtual memory */

} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/* For lab 5 */
#define PAGE_NUM_BITS 9
#define PTE_PFN_MASK 0x3E00
#define PTE_VALID_MASK 0x0004
#define PAGE_OFFSET_MASK 0x1FF

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
    /* Timer interrupt: raised once, at cycle 300, with vector x01. */
    if (CYCLE_COUNT == 300) {
      NEXT_LATCHES.INT = 1;
      NEXT_LATCHES.INTV = 0x01;
    }

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
void load_program(char *program_filename, int is_virtual_base) {
    FILE * prog;
    int ii, word, program_base, pte, virtual_pc;

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

    if (is_virtual_base) {
      if (CURRENT_LATCHES.PTBR == 0) {
	printf("Error: Page table base not loaded %s\n", program_filename);
	exit(-1);
      }

      /* convert virtual_base to physical_base */
      virtual_pc = program_base << 1;
      pte = (MEMORY[(CURRENT_LATCHES.PTBR + (((program_base << 1) >> PAGE_NUM_BITS) << 1)) >> 1][1] << 8) |
	     MEMORY[(CURRENT_LATCHES.PTBR + (((program_base << 1) >> PAGE_NUM_BITS) << 1)) >> 1][0];

      printf("virtual base of program: %04x\npte: %04x\n", program_base << 1, pte);
		if ((pte & PTE_VALID_MASK) == PTE_VALID_MASK) {
	      program_base = (pte & PTE_PFN_MASK) | ((program_base << 1) & PAGE_OFFSET_MASK);
   	   printf("physical base of program: %x\n\n", program_base);
	      program_base = program_base >> 1;
		} else {
   	   printf("attempting to load a program into an invalid (non-resident) page\n\n");
			exit(-1);
		}
    }
    else {
      /* is page table */
     CURRENT_LATCHES.PTBR = program_base << 1;
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
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0 && is_virtual_base)
      CURRENT_LATCHES.PC = virtual_pc;

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */
/*             and set up initial state of the machine         */
/*                                                             */
/***************************************************************/
void initialize(char *argv[], int num_prog_files) {
    int i;
    init_control_store(argv[1]);

    init_memory();
    load_program(argv[2],0);
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(argv[i + 3],1);
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);
    CURRENT_LATCHES.SSP = 0x3000; /* Initial value of system stack pointer */

/* MODIFY: you can add more initialization code HERE */

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
    if (argc < 4) {
	printf("Error: usage: %s <micro_code_file> <page table file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv, argc - 3);

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

/***************************************************************/
/* Helper functions and signals shared between the routines.   */
/***************************************************************/

/* Sign-extend the low 'bits' bits of x to a full int. */
int sext(int x, int bits) {
  int mask = 1 << (bits - 1);
  x &= (1 << bits) - 1;
  return (x ^ mask) - mask;
}

/* Memory model state. */
int MEM_CYCLE_COUNT = 0;   /* cycles the current memory access has taken */
int MEM_OUT = 0;           /* word read from memory this cycle (valid when R) */

/* Values at the inputs of the tristate drivers. */
int MARMUX_OUT, PC_OUT, ALU_OUT, SHF_OUT, MDR_OUT;
int PSR_OUT, SP_OUT, PCM2_OUT, VECTOR_OUT;

void eval_bus_drivers();
void drive_bus();

/* Output of the address adder (feeds MARMUX and PCMUX). */
int ADDER_OUT;

/* SR1 out: register selected by SR1MUX. */
int get_sr1(int *mi) {
  int ir = CURRENT_LATCHES.IR;
  int sr1;
  switch (GetSR1MUX(mi)) {
    case 0:  sr1 = (ir >> 9) & 0x7; break;
    case 1:  sr1 = (ir >> 6) & 0x7; break;
    default: sr1 = 6;               break;   /* stack pointer */
  }
  return CURRENT_LATCHES.REGS[sr1];
}

/* Microsequencer COND encodings. */
#define COND_UNCOND   0
#define COND_READY    1
#define COND_BRANCH   2
#define COND_ADDR     3
#define COND_INT      4   /* fetch: J[4] |= INT, plus word access check  */
#define COND_EXC_WORD 5   /* word access: protection or unaligned check  */
#define COND_EXC_BYTE 6   /* byte access: protection check only          */
#define COND_PRIV     7   /* J[3] |= PSR[15]                             */

/* State that starts the shared interrupt/exception initiation sequence. */
#define EXC_START_STATE 37

#define VEC_PROTECTION 0x02
#define VEC_UNALIGNED  0x03
#define VEC_UNKNOWN_OP 0x04

/*
 * Exception detection logic. Its inputs are the address being loaded
 * into MAR this cycle (the bus), PSR[15], IR[15:13] and COND.
 * Returns the exception vector, or 0 if there is no exception.
 * Protection has priority over unaligned access.
 *
 * The fetch states (COND_INT) check the PC as a word access. The opcode
 * check is skipped there, because the IR still holds the previous
 * instruction.
 */
int exception_logic(int *mi, int addr) {
  int cond = GetCOND(mi);
  int word = (cond == COND_EXC_WORD || cond == COND_INT);

  if (cond != COND_INT && ((CURRENT_LATCHES.IR >> 13) & 0x7) == 0x5)
    return VEC_UNKNOWN_OP;                              /* opcode 1010/1011 */
  if (!word && cond != COND_EXC_BYTE)
    return 0;
  if (CURRENT_LATCHES.PRIV && addr < 0x3000)
    return VEC_PROTECTION;
  if (word && (addr & 0x1))
    return VEC_UNALIGNED;
  return 0;
}


void eval_micro_sequencer() {

  /*
   * Evaluate the address of the next state according to the
   * micro sequencer logic. Latch the next microinstruction.
   */
  int *mi = CURRENT_LATCHES.MICROINSTRUCTION;
  int ir = CURRENT_LATCHES.IR;
  int next_state;

  if (GetIRD(mi)) {
    /* Decode: 0,0,IR[15:12] */
    next_state = (ir >> 12) & 0xF;
  } else {
    int cond = GetCOND(mi);
    int j = GetJ(mi);
    int branch = (cond == COND_BRANCH) && CURRENT_LATCHES.BEN;     /* J[2] */
    int ready  = (cond == COND_READY) && CURRENT_LATCHES.READY;    /* J[1] */
    int amode  = (cond == COND_ADDR) && ((ir >> 11) & 0x1);        /* J[0] */
    int intr   = (cond == COND_INT) && CURRENT_LATCHES.INT;        /* J[4] */
    int priv   = (cond == COND_PRIV) && CURRENT_LATCHES.PRIV;      /* J[3] */
    next_state = j | (intr << 4) | (priv << 3) | (branch << 2) |
                 (ready << 1) | amode;

    if (cond == COND_EXC_WORD || cond == COND_EXC_BYTE || cond == COND_INT) {
      /* The exception logic looks at the address going into MAR this
         cycle, so evaluate the (combinational) bus value first. If the
         access would fault, redirect to the initiation sequence instead
         of starting the memory access. In the fetch states this also
         wins over a pending interrupt, which is then taken at the
         handler's first fetch. */
      eval_bus_drivers();
      drive_bus();
      if (exception_logic(mi, BUS))
        next_state = EXC_START_STATE;
    }
  }

  NEXT_LATCHES.STATE_NUMBER = next_state;
  memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[next_state],
         sizeof(int) * CONTROL_STORE_BITS);
}


void cycle_memory() {

  /*
   * This function emulates memory and the WE logic.
   * Keep track of which cycle of MEMEN we are dealing with.
   * If fourth, we need to latch Ready bit at the end of
   * cycle to prepare microsequencer for the fifth cycle.
   */
  int *mi = CURRENT_LATCHES.MICROINSTRUCTION;
  int addr = CURRENT_LATCHES.MAR >> 1;   /* word address */
  int mar0 = CURRENT_LATCHES.MAR & 0x1;

  if (!GetMIO_EN(mi)) {
    MEM_CYCLE_COUNT = 0;
    NEXT_LATCHES.READY = 0;
    return;
  }

  if (CURRENT_LATCHES.READY) {
    /* Fifth cycle: the access completes this cycle. */
    if (GetR_W(mi)) {
      /* Write: WE logic based on DATA.SIZE and MAR[0]. */
      int mdr = CURRENT_LATCHES.MDR;
      if (GetDATA_SIZE(mi)) {
        MEMORY[addr][0] = mdr & 0xFF;             /* WE0 */
        MEMORY[addr][1] = (mdr >> 8) & 0xFF;      /* WE1 */
      } else if (mar0) {
        MEMORY[addr][1] = (mdr >> 8) & 0xFF;      /* WE1 */
      } else {
        MEMORY[addr][0] = mdr & 0xFF;             /* WE0 */
      }
    } else {
      /* Read: always a full word from M[MAR[15:1]'0]. */
      MEM_OUT = Low16bits((MEMORY[addr][1] << 8) | MEMORY[addr][0]);
    }
    MEM_CYCLE_COUNT = 0;
    NEXT_LATCHES.READY = 0;
  } else {
    MEM_CYCLE_COUNT++;
    /* Ready is asserted at the end of the fourth cycle. */
    NEXT_LATCHES.READY = (MEM_CYCLE_COUNT == MEM_CYCLES - 1);
  }
}



void eval_bus_drivers() {

  /*
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers
   *             Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */
  int *mi = CURRENT_LATCHES.MICROINSTRUCTION;
  int ir = CURRENT_LATCHES.IR;
  int sr1 = get_sr1(mi);
  int addr1, addr2, op2, amt;

  /* Address adder: ADDR1MUX + (optionally shifted) ADDR2MUX. */
  addr1 = GetADDR1MUX(mi) ? sr1 : CURRENT_LATCHES.PC;
  switch (GetADDR2MUX(mi)) {
    case 0:  addr2 = 0;                    break;
    case 1:  addr2 = sext(ir, 6);          break;
    case 2:  addr2 = sext(ir, 9);          break;
    default: addr2 = sext(ir, 11);         break;
  }
  if (GetLSHF1(mi)) addr2 <<= 1;
  ADDER_OUT = Low16bits(addr1 + addr2);

  /* MARMUX: LSHF(ZEXT[IR[7:0]],1) or adder output. */
  MARMUX_OUT = GetMARMUX(mi) ? ADDER_OUT : Low16bits((ir & 0xFF) << 1);

  /* PC */
  PC_OUT = CURRENT_LATCHES.PC;

  /* ALU: A = SR1, B = SR2MUX (IR[5] selects imm5 or SR2). */
  op2 = ((ir >> 5) & 0x1) ? sext(ir, 5) : CURRENT_LATCHES.REGS[ir & 0x7];
  switch (GetALUK(mi)) {
    case 0:  ALU_OUT = Low16bits(sr1 + op2); break;
    case 1:  ALU_OUT = Low16bits(sr1 & op2); break;
    case 2:  ALU_OUT = Low16bits(sr1 ^ op2); break;
    default: ALU_OUT = Low16bits(sr1);       break;
  }

  /* SHF: IR[5:4] selects LSHF / RSHFL / RSHFA, IR[3:0] is amount. */
  amt = ir & 0xF;
  switch ((ir >> 4) & 0x3) {
    case 0:  SHF_OUT = Low16bits(sr1 << amt);            break;
    case 1:  SHF_OUT = Low16bits(sr1 >> amt);            break;
    default: SHF_OUT = Low16bits(sext(sr1, 16) >> amt);  break;
  }

  /* MDR: whole word, or sign-extended byte selected by MAR[0]. */
  if (GetDATA_SIZE(mi)) {
    MDR_OUT = CURRENT_LATCHES.MDR;
  } else if (CURRENT_LATCHES.MAR & 0x1) {
    MDR_OUT = Low16bits(sext(CURRENT_LATCHES.MDR >> 8, 8));
  } else {
    MDR_OUT = Low16bits(sext(CURRENT_LATCHES.MDR, 8));
  }

  /* PSR: PSR[15] is the privilege bit, PSR[2:0] are N, Z, P. */
  PSR_OUT = (CURRENT_LATCHES.PRIV << 15) | (CURRENT_LATCHES.N << 2) |
            (CURRENT_LATCHES.Z << 1) | CURRENT_LATCHES.P;

  /* SPMUX: stack pointer increment/decrement or a saved stack pointer. */
  switch (GetSPMUX(mi)) {
    case 0:  SP_OUT = Low16bits(sr1 + 2);         break;
    case 1:  SP_OUT = Low16bits(sr1 - 2);         break;
    case 2:  SP_OUT = CURRENT_LATCHES.SSP;        break;
    default: SP_OUT = CURRENT_LATCHES.USP;        break;
  }

  /* PC-2: address of the instruction being interrupted / that faulted. */
  PCM2_OUT = Low16bits(CURRENT_LATCHES.PC - 2);

  /* Interrupt/exception vector table entry: x0200 + LSHF(EXCV, 1). */
  VECTOR_OUT = 0x0200 + ((CURRENT_LATCHES.EXCV & 0xFF) << 1);
}


void drive_bus() {

  /*
   * Datapath routine for driving the bus from one of the 5 possible
   * tristate drivers.
   */
  int *mi = CURRENT_LATCHES.MICROINSTRUCTION;

  if (GetGATE_MARMUX(mi))   BUS = MARMUX_OUT;
  else if (GetGATE_PC(mi))  BUS = PC_OUT;
  else if (GetGATE_ALU(mi)) BUS = ALU_OUT;
  else if (GetGATE_SHF(mi)) BUS = SHF_OUT;
  else if (GetGATE_MDR(mi)) BUS = MDR_OUT;
  else if (GetGATE_PSR(mi)) BUS = PSR_OUT;
  else if (GetGATE_SP(mi))  BUS = SP_OUT;
  else if (GetGATE_PCM2(mi)) BUS = PCM2_OUT;
  else if (GetGATE_VECTOR(mi)) BUS = VECTOR_OUT;
  else                      BUS = 0;   /* nothing driving the bus */
}


void latch_datapath_values() {

  /*
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come
   * after drive_bus.
   */
  int *mi = CURRENT_LATCHES.MICROINSTRUCTION;
  int ir = CURRENT_LATCHES.IR;

  if (GetLD_MAR(mi))
    NEXT_LATCHES.MAR = Low16bits(BUS);

  if (GetLD_MDR(mi)) {
    if (GetMIO_EN(mi)) {
      /* Only take memory data once the access has completed. */
      if (CURRENT_LATCHES.READY)
        NEXT_LATCHES.MDR = MEM_OUT;
    } else if (GetDATA_SIZE(mi)) {
      NEXT_LATCHES.MDR = Low16bits(BUS);
    } else {
      /* Byte store: replicate BUS[7:0] into both halves. */
      NEXT_LATCHES.MDR = ((BUS & 0xFF) << 8) | (BUS & 0xFF);
    }
  }

  if (GetLD_IR(mi))
    NEXT_LATCHES.IR = Low16bits(BUS);

  if (GetLD_BEN(mi))
    NEXT_LATCHES.BEN = (((ir >> 11) & 1) && CURRENT_LATCHES.N) ||
                       (((ir >> 10) & 1) && CURRENT_LATCHES.Z) ||
                       (((ir >> 9)  & 1) && CURRENT_LATCHES.P);

  if (GetLD_REG(mi)) {
    int dr;
    switch (GetDRMUX(mi)) {
      case 0:  dr = (ir >> 9) & 0x7; break;
      case 1:  dr = 7;               break;
      default: dr = 6;               break;   /* stack pointer */
    }
    NEXT_LATCHES.REGS[dr] = Low16bits(BUS);
  }

  if (GetLD_CC(mi)) {
    int val = Low16bits(BUS);
    NEXT_LATCHES.N = (val & 0x8000) ? 1 : 0;
    NEXT_LATCHES.Z = (val == 0) ? 1 : 0;
    NEXT_LATCHES.P = (val != 0 && !(val & 0x8000)) ? 1 : 0;
  }

  if (GetLD_PC(mi)) {
    switch (GetPCMUX(mi)) {
      case 0:  NEXT_LATCHES.PC = Low16bits(CURRENT_LATCHES.PC + 2); break;
      case 1:  NEXT_LATCHES.PC = Low16bits(BUS);                    break;
      default: NEXT_LATCHES.PC = ADDER_OUT;                         break;
    }
  }

  if (GetLD_PSR(mi)) {
    if (GetPSRMUX(mi)) {
      /* Enter supervisor mode; condition codes are unchanged. */
      NEXT_LATCHES.PRIV = 0;
    } else {
      /* Restore the whole PSR from the bus (RTI). */
      int val = Low16bits(BUS);
      NEXT_LATCHES.PRIV = (val >> 15) & 0x1;
      NEXT_LATCHES.N = (val >> 2) & 0x1;
      NEXT_LATCHES.Z = (val >> 1) & 0x1;
      NEXT_LATCHES.P = val & 0x1;
    }
  }

  /* Saved stack pointers are loaded straight from SR1 out (R6). */
  if (GetLD_SSP(mi))
    NEXT_LATCHES.SSP = get_sr1(mi);
  if (GetLD_USP(mi))
    NEXT_LATCHES.USP = get_sr1(mi);

  if (GetLD_EXCV(mi))
    NEXT_LATCHES.EXCV = GetEXCVMUX(mi) ? (CURRENT_LATCHES.INTV & 0xFF)
                                       : exception_logic(mi, BUS);

  if (GetCLR_INT(mi))
    NEXT_LATCHES.INT = 0;
}
