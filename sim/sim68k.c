/*
 * sun-2 emulator
 *
 * 10/2014  Brad Parker <brad@heeltoe.com>
 *
 */

#define _LARGEFILE64_SOURCE


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <signal.h>
#define __USE_GNU
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/time.h>
 
#include "sim68k.h"
#include "m68k.h"
#include "sim.h"


/* Read/write macros */
#define READ_BYTE(BASE, ADDR) (BASE)[ADDR]

#define READ_WORD(BASE, ADDR) (((BASE)[(ADDR)+0]<<8) |	\
			       (BASE)[(ADDR)+1])

#define READ_WORD_LE(BASE, ADDR) (((BASE)[(ADDR)+1]<<8) |	\
				  (BASE)[(ADDR)+0])

#define READ_LONG(BASE, ADDR) (((BASE)[ADDR]<<24) |	\
			       ((BASE)[(ADDR)+1]<<16) |	\
			       ((BASE)[(ADDR)+2]<<8) |	\
			       (BASE)[(ADDR)+3])

#define READ_LONG_LE(BASE, ADDR) (((BASE)[(ADDR)+3]<<24) |	\
			       ((BASE)[(ADDR)+2]<<16) |	\
			       ((BASE)[(ADDR)+1]<<8) |	\
			       (BASE)[(ADDR)+0])

#define WRITE_BYTE(BASE, ADDR, VAL) (BASE)[ADDR] = (VAL)&0xff

#define WRITE_WORD(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>8) & 0xff;	\
                                    (BASE)[(ADDR)+1] = (VAL)&0xff

#define WRITE_LONG(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>24) & 0xff;	\
                                    (BASE)[(ADDR)+1] = ((VAL)>>16)&0xff; \
                                    (BASE)[(ADDR)+2] = ((VAL)>>8)&0xff; \
                                    (BASE)[(ADDR)+3] = (VAL)&0xff


/* Prototypes */
void trace_all();
void exit_error(char* fmt, ...);
int osd_get_char(void);

void trace_file_mem_rd(unsigned int ea, unsigned int pa, int fc, int size, unsigned int pte,
		       int mtype, int fault, unsigned int value, unsigned int pc);
void trace_file_mem_wr(unsigned int ea, unsigned int pa, int fc, int size, unsigned int pte,
		       int mtype, int fault, unsigned int value, unsigned int pc);
void trace_file_pte_set(unsigned int address, unsigned int va, int bus_type, int boot, unsigned int pte);

unsigned int cpu_read_byte(unsigned int address);
unsigned int cpu_read_word(unsigned int address);
unsigned int cpu_read_long(unsigned int address);
void cpu_write_byte(unsigned int address, unsigned int value);
void cpu_write_word(unsigned int address, unsigned int value);
void cpu_write_long(unsigned int address, unsigned int value);
void cpu_pulse_reset(void);
void cpu_set_fc(unsigned int fc);
int cpu_irq_ack(int level);

void nmi_device_reset(void);
//void nmi_device_update(void);
int nmi_device_ack(void);
void get_user_input(void);
void pending_buserr(void);


/* Data */
unsigned int g_quit = 0;			/* 1 if we want to quit */
unsigned int g_nmi = 0;				/* 1 if nmi pending */
unsigned int g_buserr = 0;
unsigned int g_buserr_pc = 0;
int g_trace = 0;
unsigned long g_isn_count;

int trace_cpu_io = 0;
int trace_cpu_rw = 0;
int trace_cpu_isn = 0;
int trace_cpu_bin;
int trace_mmu_bin;
int trace_mmu;
int trace_mmu_rw;
int trace_mem_bin;
int trace_sc = 1;
int trace_scsi = 1;
int trace_armed;
int trace_irq;

extern int quiet;

unsigned int g_int_controller_pending = 0;	/* list of pending interrupts */
unsigned int g_int_controller_highest_int = 0;	/* Highest pending interrupt */

#ifdef TEK4404

unsigned int tekscsi;
unsigned int tekscsidelay;

unsigned short tekmap[4096];

enum {STS_RXR=1, STS_TXR=4, STS_TXE=8, STS_OER=16,STS_PER=32,STS_FER=64};
enum {CMD_ERX=1, CMD_DRX=2, CMD_ETX=4, CMD_DTX=8};
enum {ISTS_TAI=1, ISTS_RAI=2, ISTS_TBI=16, ISTS_RBI=32, ISTS_IPC=128};
enum {KEYBOARD_INT=4, TX_INT=16, RX_INT=32};
int duartbaud[][16] = {
	{50,110,134,200,300,600,1200,1060,2400,4800,7200,9600,38400, 0,0,0},
	{75,110,134,150,300,600,1200,2000,2400,4800,1800,9600,19200, 0,0,0}
};

struct DuartPort {
	unsigned char mode[2],stat,rx,tx,mode_index;
	unsigned char rx_wi,rx_ri, rx_queue[16];
	unsigned char tx_wi,tx_ri, tx_queue[16];
};

int empty_input(struct DuartPort *port)
{
	return port->rx_wi == port->rx_ri;
}

void push_input(struct DuartPort *port, int scancode)
{
	port->rx_queue[port->rx_wi & 15] = scancode;
	port->rx_wi++;
}

int pop_input(struct DuartPort *port)
{
	int c = 0;
	
	if (!empty_input(port))
	{
		c =	port->rx_queue[port->rx_ri & 15];
		port->rx_ri++;
	}
	
	return c;
}

struct Duart {
	struct DuartPort ports[2];
	unsigned char acr,ipcr, istat,imr,ivec;
};

struct Duart duart;

void tek4404_keydown(int c)
{

	duart.ports[0].stat |= STS_RXR;
	duart.istat |= ISTS_RAI;
	duart.ivec |= RX_INT;
	push_input(&duart.ports[0], c);

	duart.ports[0].rx = 0x05;
	
}

void tek4404_keyup(int c)
{
	duart.ports[0].stat |= STS_RXR;
	duart.istat |= ISTS_RAI;
	duart.ivec |= RX_INT;
	push_input(&duart.ports[0], c);
}



unsigned char acia[8];
char aciabuffer[128];
int acialen = 0;
int aciabaud[] = {0,50,75,110,134,150,300,600,1200,1800,2400,3600,4800,7200,9600,19200};
char aciafakeinput[] = "A7400\n";
int aciafakelen = 0;

unsigned char g_debugram[4096];		// 0x760000
#endif
unsigned char g_rom[MAX_ROM+1];					/* ROM */
unsigned char g_ram[MAX_RAM+1];					/* RAM */

#ifdef RAMDISK
unsigned char g_ramdisk[RAMDISK_SIZE];
#endif

unsigned int g_fc;       /* Current function code from CPU */

void memdump(int start, int end)
{
  int i = 0;
  while(start < end)
    {
      if((i++ & 0x0f) == 0)
	fprintf(stderr, "\r\n%08x:",start);
      fprintf(stderr, "%02x ", g_ram[start++]);
    }
}


void termination_handler(int signum)
{
  exit(0);
}

/* Exit with an error message.  Use printf syntax. */
void exit_error(char* fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);
  fprintf(stderr, "\n");

  exit(EXIT_FAILURE);
}

/* ------------------------------------------------ */

unsigned int io_read(int size, unsigned int va, unsigned int pa)
{
  unsigned int value;
	int port;
	
  value = 0xffff;
//  value = 0x0;

  switch (size) {
  case 1: value = 0xff; break;
  case 2: value = 0xffff; break;
  }

  /* for eprom, hardware bypasses mapping with cpu va */
#ifdef TEK4404
  if (pa < 0x10  && g_fc == 6) {
#else
  if (pa < 0x800 || pa >= 0xef0000) {
#endif
//  prom:
    pa = va & 0x7fff;
    switch (size) {
    case 1: value = READ_BYTE(g_rom, pa); break;
    case 2: value = READ_WORD(g_rom, pa); break;
    default:
    case 4: value = READ_LONG(g_rom, pa); break;
    }

    return value;
  }

#ifdef TEK4404
	// nothing here
	if (pa > MAX_RAM && pa < 0x600000) {
		trace_cpu_rw = trace_cpu_isn = trace_mmu = trace_sc = trace_scsi = 1;

		// used to probe how much memory is installed;  SFC/DFC alternatvely set to userdata,sysdata..
		pending_buserr();

		return value;
	}

  if (pa >= 0x600000 && pa < 0x620000) {
    return sun2_video_read(pa, size);
  }

  if (pa >= 0x800000 && pa < 0x1000000) {

			printf("tekmap[%d] <= %d\n", ((pa & 0xfffff) >> 12) & 0x1fff, value);
    return 0;
  }

  switch (pa & 0xfff000) {

  default:
		switch (size) {
		case 1: value = cpu_read_byte(pa); break;
		case 2: value = cpu_read_word(pa); break;
		default:
		case 4:
			value = cpu_read_long(pa); break;
		}
		
		break;
  
	case 0x740000:			// ROM_START
	case 0x741000:
	case 0x742000:
	case 0x743000:
	case 0x744000:
	case 0x745000:
	case 0x746000:
	case 0x747000:
	case 0x748000:
	case 0x749000:
	case 0x74a000:
	case 0x74b000:
	case 0x74c000:
	case 0x74d000:
	case 0x74e000:
	case 0x74f000:
    pa = va & 0x7fff;
    switch (size) {
    case 1: value = READ_BYTE(g_rom, pa); break;
    case 2: value = READ_WORD(g_rom, pa); break;
    default:
    case 4: value = READ_LONG(g_rom, pa); break;
    }
  	break;
	
	case 0x760000:			// DEBUG_RAM_START
    pa = va & 0xfff;
    switch (size) {
    case 1: value = READ_BYTE(g_debugram, pa); break;
    case 2: value = READ_WORD(g_debugram, pa); break;
    default:
    case 4: value = READ_LONG(g_debugram, pa); break;
    }
		break;

// Processor Board I/O

	case 0x780000:			// MMU_START
  	value = 0;
  	break;
	
	case 0x788000:			// SOUND_START
  	printf("SOUND(%08x) read %d\n", pa, size);
  	value = 0;
  	break;
	
  case 0x78a000:			// FPU_START
  	value = 0;
  	break;

  case 0x78c000:			// ACIA_START
  	value = acia[pa & 7];
  	if ((pa & 7) == 0)	// Data
  	{
  		// generate some fake input for monitor
			value = aciafakeinput[aciafakelen++];
			if (value == '\n')
			{
				// pretend we've dropped the connection
				acia[2] = 0x18 | 0x60;
			}
			if (aciafakelen > 6)
			{
				value = 0;
			}
  	}
  	if ((pa & 7) == 2)	// Status
  	{
  		value |= 0x08;		// recv empty
  		value |= 0x10;		// xmit empty
  		
  		// to provoke generating a prompt we pretend to be connected
			//value |= 0x60;		// DCD/DSR high (not connected)
  	}
  	break;

// Peripheral Board I/O

  case 0x7b0000:			// DIAG_START
  	value = 0;
  	break;
  	
  case 0x7b4000:			// DUART_START
  	// READ
		port = (pa>>4) & 1;
  	if ((pa & 15) == 0)		// MR12A
  	{
  		if (size == 2)
  		{
				value = (duart.ports[port].mode[0] << 8) | duart.ports[port].mode[1];
  		}
  		else
  		{
				value = duart.ports[port].mode[duart.ports[port].mode_index & 1];
				duart.ports[port].mode_index++;
			}
  	}
  	if ((pa & 15) == 2)		// CSRA
  	{
  		value = duart.ports[port].stat;

  		// fake process transmit
  		if ((duart.ports[port].stat & (STS_TXR | STS_TXE)))
  		{
				duart.istat = port ? ISTS_TBI : ISTS_TAI;
				
				if (duart.ports[port].mode[1] & 128)	// loopback
				{
					duart.ports[port].stat |= STS_RXR;
					duart.ports[port].rx = duart.ports[port].tx;
					duart.ivec |= RX_INT;
				}
  		}
  		
  	}
  	if ((pa & 15) == 6)		// Receive Buffer A
  	{
  		// this makes self-test fail..
  		//duart.ports[port].rx = pop_input(&duart.ports[port]);
  		
  		duart.ports[port].stat &= ~STS_RXR;
  		duart.istat &= ~(port ? ISTS_RBI : ISTS_RAI);
			duart.ivec &= ~(port ? KEYBOARD_INT : RX_INT);

  		value = duart.ports[port].rx;
  	}
  	
  	if ((pa & 31) == 8)		// IPCR_ACR
  	{
  		value = duart.ipcr;
	value = 0x40;
  		duart.ipcr &= 0xf0;
  		duart.ivec = 0;
			duart.istat &= ~ISTS_IPC;
  	}
  	if ((pa & 31) == 10)		// ISR_MASK
  	{
  		value = duart.istat;
  	}
  	if ((pa & 31) == 26)		// IP_OPCR
  	{
			value = 0x10;	// IP4 high (RxRDYAI)
  	}
  	
  	printf("DUART(%08x) %d => %02x\n", pa, size, value);

  	break;
  	
  case 0x7b6000:			// MOUSE_START
  	printf("MOUSE(%08x) read %d\n", pa, size);
  	value = 0;
  	break;
  	
  case 0x7b8000:			// TIMER_START
  	value = am9513_read(pa, size);
  	break;
  	
  case 0x7ba000:			// CAL_START
  	printf("CAL(%08x) read %d\n", pa, size);
  	value = 0;
  	break;
  	
  case 0x7bc000:			// SCSI_BUS_ADDRESS_REGISTER_START
		printf("scsi_bus_address(%08x) read %d\n", pa, size);
    break;

  case 0x7be000:			// SCSI_START
	  printf("scsi(%08x) read %d\n", pa, size);

		// reading 2 16-bit values..
    if (pa == 0x7be008)
    {
    	value = 0;
    }
    if (pa == 0x7be00c)
    {
    	value = 0;
    }

    if (pa == 0x7be012)
    {
    	switch(tekscsi)
    	{
				case 0x00:
					value = 128;
					break;
	    	case 0x01:
					break;
				case 0xff:
					break;
    	}
    }
    break;
	}

#else

  switch (pa & 0xff00) {
  case 0x2000:
  case 0x2002:
  case 0x2004:
  case 0x2006:
    value = scc_read(pa, size);
    break;

  case 0x2800:
    value = am9513_read(pa, size);
    break;

  case 0x1800:
pending_buserr();
    break;

  case 0x3800:
  case 0x3802:
  case 0x3804:
  case 0x3806:
  case 0x3808:
  case 0x380a:
  case 0x380c:
  case 0x380e:
  case 0x3828:
    value = mm58167_read(pa, size);
    break;

  case 0xe0000:
    value = e3c400_read(pa, size);
    break;

  default:
    printf("io: read %x -> %x (%d) pc %x\n", pa, value, size, m68k_get_reg(NULL, M68K_REG_PC));
    break;
  }
  
#endif


  if (trace_cpu_io)
    printf("io: read %x -> %x (%d) pc %x\n", pa, value, size, m68k_get_reg(NULL, M68K_REG_PC));

  return value;
}

void io_write(int size, unsigned int pa, unsigned int value)
{
	int port;

  if (trace_cpu_io)
    printf("io: write %x <- %x (%d)\n", pa, value, size);

#ifdef TEK4404
	// nothing here
	if (pa > MAX_RAM && pa < 0x600000) {
		trace_cpu_rw = trace_cpu_isn = trace_mmu = trace_sc = trace_scsi = 1;

		  pending_buserr();

		return;
	}

  if (pa >= 0x600000 && pa < 0x620000) {
    sun2_video_write(pa, size, value);
    return;
  }

  if (pa >= 0x800000 && pa < 0x1000000) {

			tekmap[((pa & 0xfffff) >> 12) & 0x1fff] = value;
			printf("tekmap[%d] <= %d\n", ((pa & 0xfffff) >> 12) & 0x1fff, value);
    return;
  }

  switch (pa & 0xfff000) {

  default:
		switch (size) {
		case 1: cpu_write_byte(pa, value); break;
		case 2: cpu_write_word(pa, value); break;
		default:
		case 4: cpu_write_long(pa, value); break;
		}
		break;
  
  case 0x740000:
		printf("LED %c%c%c%c\n", value & 8 ? '*' : '.', value & 4 ? '*' : '.', value & 2 ? '*' : '.', value & 1 ? '*' : '.');
		if (value == 1)
			trace_cpu_rw = trace_cpu_isn = 0;
		break;

	case 0x760000:			// DEBUG_RAM_START
    switch (size) {
    case 1: WRITE_BYTE(g_debugram, pa & 0xfff, value); break;
    case 2: WRITE_WORD(g_debugram, pa & 0xfff, value); break;
    default:
    case 4: WRITE_LONG(g_debugram, pa & 0xfff, value); break;
    }
		break;
		
// Processor Board I/O

	case 0x780000:			// MMU_START
		printf("MMU(%08x) <= %02x\n", pa, value);
		
		//trace_cpu_rw = trace_cpu_isn = trace_mmu = trace_sc = trace_scsi = 1;

		printf("VirtualMem:      %s\n", value & 0x10 ? "ENABLE" : "DISABLE");
		printf("Write Map Table: %s\n", value & 0x20 ? "ENABLE" : "DISABLE");
		printf("PTE pid:         %02d\n", value & 7);
  	break;
	
	case 0x782000:			// VIDEO_PAN
  	value = 0;
  	break;
	case 0x784000:			// VIDEO_CONT
  	value = 0;
  	break;

	case 0x788000:			// SOUND_START
		printf("SOUND(%08x) <= %02x\n", pa, value);
		
  	value = 0;
  	break;
	
  case 0x78a000:			// FPU_START
  	value = 0;
  	break;

  case 0x78c000:			// ACIA_START
  	if ((pa & 7) == 0)	// DATA
  	{
			aciabuffer[acialen++] = value;
  		if (value == '\n')
  		{
  			aciabuffer[acialen] = 0;
	  		printf("%s", aciabuffer);
				acialen = 0;
  		}
  		break;
  	}
  	if ((pa & 7) == 2)	// STAT
  	{
  		printf("ACIA STAT: %02x\n", value);
  		acia[0] = 0;
  		acialen = 0;
  	}
  	if ((pa & 7) == 4)	// CMD
  	{
	  	acia[4] = value;
  		printf("ACIA CMD: %02x  DTR(%d) IRQ(%d)\n", value, value & 1, (value & 2)>>1);
  	}
  	if ((pa & 7) == 6)	// CTRL
  	{
	  	acia[6] = value;
  		printf("ACIA CTRL: %02x Baud(%d) Bits(%d)\n", value, aciabaud[value & 15], 8-((value>>5)&3));
  		
  	}
  	break;

// Peripheral Board I/O

  case 0x7b0000:			// DIAG_START
  	value = 0;
  	break;
  	
  case 0x7b4000:			// DUART_START
		printf("DUART(%08x) <= %02x\n", pa, value);
		port = (pa>>4) & 1;
  	if ((pa & 15) == 0)		// MR12A
  	{
  		if (size == 2)
  		{
  			printf("ERROR\n");
  		}
  		duart.ports[port].mode[duart.ports[port].mode_index & 1] = value;
  		duart.ports[port].mode_index++;
  		
  		if ((duart.ports[port].mode_index & 1) == 0)
  		{
  			printf("%d: ",port);
				printf("%dbit ", 5 + (duart.ports[port].mode[0] & 3));
				if (duart.ports[port].mode[0] & 128)
					printf("rts ");
				if (duart.ports[port].mode[1] & 16)
					printf("cts ");
				if (duart.ports[port].mode[1] & 128)
					printf("loopback");
				putchar('\n');
			}
  	}
  	if ((pa & 15) == 2)		// CSRA (ClockSelect)
  	{
  		int set;
  		value = value;		// ignore BAUD rate selection
  		
  		set = !!(duart.acr & 128);
  		printf("%d: rcv baud %d xmit baud %d\n", port, duartbaud[set][(value>>4)&15], duartbaud[set][value&15]);
  	}
  	if ((pa & 15) == 4)		// CRA	(Command)
  	{
  		if (value & CMD_DTX)
  		{
  			printf("%d: Disable TX\n",port);
  			duart.ports[port].stat &= ~STS_TXR;
  			duart.ports[port].stat &= ~STS_TXE;
  			if (port == 0)
  			{
					duart.ivec &= ~TX_INT;
					duart.istat &= ~ISTS_TAI;
				}
  		}
  		if (value & CMD_ETX)
  		{
  			printf("%d: Enable TX\n",port);
  			duart.ports[port].stat |= STS_TXR;
  			duart.ports[port].stat |= STS_TXE;
  			if (port == 0)
  			{
					duart.ivec |= TX_INT;
					duart.istat |= ISTS_TAI;
				}
  		}
  		if (value & CMD_DRX)
  		{
  			printf("%d: Disable RX\n", port);
  			duart.ports[port].stat &= ~STS_RXR;
  			duart.ivec &= ~(port ? KEYBOARD_INT : RX_INT);
  			duart.istat &= ~(port ? ISTS_RBI : ISTS_RAI);
			}
  		if (value & CMD_ERX)
  		{
  			printf("%d: Enable RX\n", port);
  			duart.ports[port].stat |= STS_RXR;
  		}
  		switch((value>>4)&7)
  		{
  			case 1:
  				duart.ports[port].mode_index = 0;
  				break;
  			case 2:
  				duart.ports[port].stat |= STS_RXR;
  				break;
  			case 3:
  				duart.ports[port].stat |= STS_TXR;
  				duart.ports[port].stat |= STS_TXE;
  				break;
  			case 4:
  				duart.ports[port].stat &= ~(STS_FER | STS_PER | STS_OER);
  				break;
  		}
  	}
  	if ((pa & 15) == 6)		// Transmit Buffer A
  	{
			duart.ports[port].tx = value;
			duart.ports[port].stat &= ~(STS_TXE | STS_TXR);
			duart.istat &= ~(port ? ISTS_TBI : ISTS_TAI);
			duart.ivec &= ~TX_INT;
			
			if (duart.ports[port].mode[1] & 128)	// loopback
			{
				duart.ports[port].stat |= STS_RXR;
				duart.ports[port].rx = value;
				duart.ivec |= RX_INT;
			}
  	}
  	
  	if ((pa & 31) == 8)		// IPCR_ACR
  	{
  		duart.acr = value;
  		printf("IP0 IRQ %d\n", !!(value & 1));
  		printf("IP1 IRQ %d\n", !!(value & 2));
  		printf("IP2 IRQ %d\n", !!(value & 4));
  		printf("IP3 IRQ %d\n", !!(value & 8));
  	}
  	if ((pa & 31) == 10)		// ISR_MASK
  	{
			duart.imr = value;
  	}
  	if ((pa & 31) == 28)		// OPBITS_SET
  	{
  		// ignore
  	}
  	if ((pa & 31) == 30)		// OPBITS_RESET
  	{
			if ((value & 8) == 0)
			{
				printf("%d: Keyboard reset\n", port);
				duart.ports[0].stat |= STS_RXR;
				duart.ports[0].rx = 0xf0; // Reset  What does this mean?

				duart.ports[0].rx_ri = 0;
				duart.ports[0].rx_wi = 0;
				duart.ports[0].tx_ri = 0;
				duart.ports[0].tx_wi = 0;
				
//				trace_cpu_rw = trace_cpu_isn = 1;
			}
  	}
  	break;
  	
  case 0x7b6000:			// MOUSE_START
  	value = 0;
  	break;
  	
  case 0x7b8000:			// TIMER_START
		printf("TIMER(%08x) <= %02x\n", pa, value);
		am9513_write(pa, value, size);
  	break;
  	
  case 0x7ba000:			// CAL_START
  	value = 0;
  	break;
  	
  case 0x7bc000:			// SCSI_BUS_ADDRESS_REGISTER_START
		printf("scsi_bus_address(%08x) <= %02x\n", pa, value);
    break;

  case 0x7be000:			// SCSI_START
		printf("scsi(%08x) <= %02x\n", pa, value);

	  trace_cpu_rw = trace_cpu_isn = 1;
	  
	  tekscsi = value;
	  switch(tekscsi)
	  {
	  	case 0x00:
	  		break;
			case 0x01:
				break;
			case 0x0b:
				break;
			case 0xff:
				break;
	  }
	  
    break;
	}

#else

  switch (pa & 0xff00) {

  case 0x2000:
  case 0x2002:
  case 0x2004:
  case 0x2006:
    scc_write(pa, value, size);
    break;

  case 0x2800:
    am9513_write(pa, value, size);
    break;

  case 0x3800: // National MM58167
  case 0x3802:
  case 0x3804:
  case 0x3806:
  case 0x3808:
  case 0x380a:
  case 0x380c:
  case 0x380e:
  case 0x3828:
    mm58167_write(pa, value, size);
    break;

  case 0xe0000:
    value = e3c400_read(pa, size);		// AB: this is a typo, right?  (should be writing)
    break;
	
  default:
    printf("io: write %x <- %x (%d)\n", pa, value, size);
    break;
  }
#endif
}

/* ------------------------------------------------ */

#define sun2_pgmap_reg		0x000
#define sun2_segmap_reg		0x004
#define sun2_context_reg	0x006
#define sun2_idprom_reg		0x008
#define sun2_diag_reg		0x00a
#define sun2_buserr_reg		0x00c
#define sun2_sysenable_reg	0x00e

#define SUN2_SYSENABLE_EN_PAR	 0x01
#define SUN2_SYSENABLE_EN_INT1	 0x02
#define SUN2_SYSENABLE_EN_INT2	 0x04
#define SUN2_SYSENABLE_EN_INT3	 0x08
#define SUN2_SYSENABLE_EN_PARERR 0x10
#define SUN2_SYSENABLE_EN_DVMA	 0x20
#define SUN2_SYSENABLE_EN_INT	 0x40
#define SUN2_SYSENABLE_EN_BOOTN	 0x80

#define SUN2_BUSERROR_PARERR_L	0x01
#define SUN2_BUSERROR_PARERR_U	0x02
#define SUN2_BUSERROR_TIMEOUT	0x04
#define SUN2_BUSERROR_PROTERR	0x08
#define SUN2_BUSERROR_VMEBUSERR	0x40
#define SUN2_BUSERROR_VALID	0x80

#if 1
#define INTS_ENABLED	(sysen_reg & SUN2_SYSENABLE_EN_INT)
#else
#define INTS_ENABLED	(1)
#endif

unsigned int buserr_reg;
unsigned int sysen_reg;
unsigned char context_user_reg;
unsigned char context_sys_reg;
unsigned char diag_reg;
unsigned char id_prom[32] = {
#if 0
  0x01, 0x02, 0x08, 0x00, 0x20, 0x01, 0x75, 0xeb, 0x1e, 0xf8, 0x85, 0x52, 0x00, 0x18, 0x6a, 0xf7,
#endif
#if 1
  0x01, 0x01, 0x08, 0x00, 0x20, 0x01, 0x06, 0xe0, 0x1a, 0xe4, 0x23, 0x3b, 0x00, 0x0d, 0x72, 0x56,
#endif
  0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff
};
unsigned int pgmap[4096];
unsigned char segmap[4096];

#define PTE_PGTYPE	 0x00c00000
#define PTE_PGTYPE_SHIFT (22)
#define PTE_PGFRAME	 0x00000fff
#define PAGE_SIZE_LOG2	 (11)
#define PAGE_SIZE	 (1 << PAGE_SIZE_LOG2)

enum {
  PGTYPE_OBMEM = 0,
  PGTYPE_OBIO  = 1,
  PGTYPE_MBMEM = 2,
  PGTYPE_VME0  = 3,
  PGTYPE_MBIO  = 3,
  PGTYPE_VME8  = 3
};

void pgmap_write(unsigned int address, unsigned int value, int size)
{
  unsigned int pa, pgtype, index;

  if (trace_mmu_rw)
  printf("mmu: pgmap write %x <- %x (%d)\n", address, value, size);

  unsigned int segindex = (((address >> 15) & 0x1ff) << 3) | context_user_reg;
  unsigned int pmeg = segmap[segindex];
  unsigned int pgmapindex = (pmeg << 4) | ((address >> 11) & 0xf);
  segindex = segmap[segindex];
segindex = ((segindex << 1) & 0xfe) | (segindex & 0x80 ? 0x01 : 0x00);
  index = (segindex << 4) | ((address >> 11) & 0xf);

  pa = (value & PTE_PGFRAME) << PAGE_SIZE_LOG2;
  pgtype = (value & PTE_PGTYPE) >> PTE_PGTYPE_SHIFT;

  if (trace_mmu_rw)
  printf("pgmap: write pgmap[0x%x] <- %x;  address %x pa %x, pgtype %d; pc %x\n",
	 index, value, address, pa, pgtype, m68k_get_reg(NULL, M68K_REG_PC));

#if 0
  printf("pgmap[%d:%08x] <- %x size %d index %x pc %x isn_count %lu\n",
	 0, address, value, size, index, m68k_get_reg(NULL, M68K_REG_PC), g_isn_count);
#endif

  if (trace_mmu_bin) {
    trace_file_pte_set(address, pa, pgtype, (sysen_reg & SUN2_SYSENABLE_EN_BOOTN) ? 1 : 0, value);
  }

  switch (pgtype) {
  case PGTYPE_OBMEM:
  case PGTYPE_OBIO:
  case PGTYPE_MBMEM:
  case PGTYPE_MBIO:
    break;
  }

  pgmap[index] = value;
}

unsigned int pgmap_read(unsigned int address, int size)
{
  unsigned int index, value;

  unsigned int segindex = (((address >> 15) & 0x1ff) << 3) | context_user_reg;
  unsigned int pmeg = segmap[segindex];
  unsigned int pgmapindex = (pmeg << 4) | ((address >> 11) & 0xf);
  segindex = segmap[segindex];
segindex = ((segindex << 1) & 0xfe) | (segindex & 0x80 ? 0x01 : 0x00);
  index = (segindex << 4) | ((address >> 11) & 0xf);

  value = pgmap[index];

  if (trace_mmu_rw)
  printf("pgmap: read address %x value %x; index %x\n",
	 address, value, index);

  return value;
}

void segmap_write(unsigned int address, unsigned int value, int size)
{
  unsigned int index;

  index = (((address >> 15) & 0x1ff) << 3) | context_user_reg;
  segmap[index] = value;

  //printf("mmu: segmap write %x <- %x (%d)\n", address, value, size);

  if (trace_mmu_rw)
  printf("segmap: write address %x segmap[%x] <- %x (%d)\n",
	 address, index, value, size);
}

unsigned int segmap_read(unsigned int address, int size)
{
  unsigned int index, value;

  index = (((address >> 15) & 0x1ff) << 3) | context_user_reg;
  value = segmap[index];

  //printf("mmu: segmap read %x -> %x (%d)\n", address, value, size);

  if (trace_mmu_rw)
  printf("segmap: read address %x segmap[%x] -> %x (%d)\n",
	 address, index, value, size);

  return value;
}

void context_write(unsigned int address, unsigned int value, int size)
{
  if (trace_mmu) printf("mmu: context write %x <- %x (%d)\n", address, value, size);

  switch (address) {
  case sun2_context_reg:
    if (size == 2) {
      context_sys_reg = (value >> 8) & 0x7;
      context_user_reg = value & 0x7;
    }
    if (size == 1) {
      context_sys_reg = value & 0x7;
      if (trace_mmu) printf("mmu: write sys context <- %x\n", value & 0x7);
    }
    break;
  case sun2_context_reg+1:
    if (size == 1) {
      context_user_reg = value & 0x7;
      if (trace_mmu) printf("mmu: write user context <- %x\n", value & 0x7);
    }
    break;
  }
}

unsigned int context_read(unsigned int address, int size)
{
  unsigned int value;

  value = 0;
  switch (address) {
  case sun2_context_reg:
    if (size == 2)
      value = (context_sys_reg << 8) | context_user_reg;
    if (size == 1)
      value = context_sys_reg;
    break;
    if (trace_mmu) printf("mmu: read user context (%d) -> %x\n", size, value);
  case sun2_context_reg+1:
    if (size == 1)
      value = context_user_reg;
    if (trace_mmu) printf("mmu: read user context+1 (%d) -> %x\n", size, value);
    break;
  }

  if (trace_mmu) printf("mmu: context read %x -> %x (%d)\n", address, value, size);

  return value;
}

unsigned int diag_read(unsigned int address, int size)
{
  unsigned int value;
  value = diag_reg;
  if (trace_mmu) printf("mmu: diag read %x -> %x (%d)\n", address, value, size);
  return value;
}

void diag_write(unsigned int address, unsigned int value, int size)
{
  if (trace_mmu) printf("mmu: diag write %x <- %x (%d)\n", address, value, size);
  switch (size) {
  case 1:
    if (address == sun2_diag_reg)
      diag_reg = (value & 0xff00) | (diag_reg & 0x00ff); /* hi */
    else
      diag_reg = (value & 0x00ff) | (diag_reg & 0xff00); /* lo */
    break;
  case 2:
    diag_reg = value;
    break;
  }
}


void sysenable_read(unsigned int address, unsigned int *pvalue, int size)
{
  unsigned int value;
  value = sysen_reg;
  if (trace_mmu || trace_irq) printf("mmu: sysen read %x -> %x (%d)\n", address, value, size);
  *pvalue = value;
}

void sw_int_throw(int intr);

void sysenable_write(unsigned int address, unsigned int value, int size)
{
  int replay = 0;
  if (trace_mmu || trace_irq) printf("mmu: sysen write %x <- %x (%d)\n", address, value, size);
  switch (size) {
  case 2:
    if ((sysen_reg & SUN2_SYSENABLE_EN_INT) && !(value & SUN2_SYSENABLE_EN_INT)) {
      printf("sim68k: sysen_reg ints off; pending 0x%x, highest %d\n",
	     g_int_controller_pending, g_int_controller_highest_int);
    }
    if (!(sysen_reg & SUN2_SYSENABLE_EN_INT) && (value & SUN2_SYSENABLE_EN_INT)) {
      printf("sim68k: sysen_reg ints on; pending 0x%x, highest %d\n",
	     g_int_controller_pending, g_int_controller_highest_int);
      replay = 1;
    }

    sysen_reg = value;
    if (trace_mmu) printf("mmu: sysen %x\n", sysen_reg);
#if 0
    if (value > 0xff)
      enable_trace(1);
#endif

    if (replay) {
      if (g_int_controller_highest_int != 0)
	m68k_set_irq(g_int_controller_highest_int);
    }
    break;
  }

  if (sysen_reg & SUN2_SYSENABLE_EN_INT1)
    sw_int_throw(1);
  if (sysen_reg & SUN2_SYSENABLE_EN_INT2)
    sw_int_throw(2);
  if (sysen_reg & SUN2_SYSENABLE_EN_INT3)
    sw_int_throw(3);
}

unsigned int map_idprom_address(unsigned int address)
{
  return address >> 11;
}

unsigned int idprom_read(unsigned int address, int size)
{
  unsigned int value, offset;

  value = 0;
  offset = map_idprom_address(address);

  switch (size) {
  case 1: value = READ_BYTE(id_prom, offset); break;
  case 2:
  case 4:
    printf("idprom read size %d\n", size);
    exit(1);
  }

  if (0) printf("IDPROM: %x %x (%d) -> %x\n", address, offset, size, value);

  return value;
}

void mmu_write(unsigned int address, unsigned int value, int size)
{
  if (trace_mmu) printf("mmu: write %x <- %x (%d)\n", address, value, size);

  switch (address & 0x000f) {
  case sun2_pgmap_reg:
  case sun2_pgmap_reg+1:
    pgmap_write(address, value, size);
    break;
  case sun2_segmap_reg:
  case sun2_segmap_reg+1:
    segmap_write(address, value, size);
    break;
  case sun2_context_reg:
  case sun2_context_reg+1:
    context_write(address, value, size);
    break;
  case sun2_diag_reg:
  case sun2_diag_reg+1:
    diag_write(address, value, size);
    break;
  case sun2_buserr_reg:
  case sun2_buserr_reg+1:
    buserr_reg = 0;
    break;
  case sun2_sysenable_reg:
  case sun2_sysenable_reg+1:
    sysenable_write(address, value, size);
    break;
  }
}

unsigned int mmu_read(unsigned int address, int size)
{
  unsigned int value;

  value = 0;

  switch (address & 0x000f) {
  case sun2_pgmap_reg:
  case sun2_pgmap_reg+1:
    value = pgmap_read(address, size);
    break;
  case sun2_segmap_reg:
  case sun2_segmap_reg+1:
    value = segmap_read(address, size);
    break;
  case sun2_context_reg:
  case sun2_context_reg+1:
    value = context_read(address, size);
    break;
  case sun2_idprom_reg:
  case sun2_idprom_reg+1:
    value = idprom_read(address, size);
    break;
  case sun2_diag_reg:
  case sun2_diag_reg+1:
    value = diag_read(address, size);
    break;
  case sun2_buserr_reg:
  case sun2_buserr_reg+1:
      value = buserr_reg;
      buserr_reg = 0;
      break;
  case sun2_sysenable_reg:
  case sun2_sysenable_reg+1:
    sysenable_read(address, &value, size);
    break;
  }

  if (trace_mmu) printf("mmu: read %x -> %x (%d)\n", address, value, size);

  return value;
}

unsigned int cpu_read_mbio(unsigned int address, int size)
{
  unsigned int value;
  value = 0xffffffff;
  if (1) printf("cpu_read_mbio address %x (%d) -> %x\n", address, size, value);
  pending_buserr();
  return value;
}

void cpu_write_mbio(unsigned int address, int size, unsigned int value)
{
  if (1) printf("cpu_write_mbio address %x (%d) <- %x\n", address, size, value);
  pending_buserr();
}

unsigned int cpu_read_mbmem(unsigned int address, int size)
{
  unsigned int value;
  value = 0xffffffff;
  if (address >= 0x80000 && address <= 0x8000e) {
    value = sc_read(address, size);
  // 3c400 board takes up 8k from e0000 to e2000, the address is dip settable but this is afaik the default and sufficient for our needs
  } else if(address >= 0xe0000 && address < 0xe2000) {
    value = e3c400_read(address, size);
  } else
    switch (address) {
#if 0
  case 0xc0000:
    enable_trace(2);
    break;
#endif
  default:
    pending_buserr();
    break;
  }
  if (0) printf("cpu_read_mbmem address %x (%d) -> %x ; pc %x isn_count %lu\n",
		address, size, value,
		m68k_get_reg(NULL, M68K_REG_PC), g_isn_count);

  return value;
}

void cpu_write_mbmem(unsigned int address, int size, unsigned int value)
{
  if (trace_cpu_rw)
    printf("cpu_write_mbmem address %x (%d) <- %x\n", address, size, value);

  if (address >= 0x80000 && address <= 0x8000f) {
    sc_write(address, size, value);
  } else if(address >= 0xe0000 && address <= 0xe2000) {
    e3c400_write(address, size, value);
  } else
  switch (address) {
  default:
    pending_buserr();
    break;
  }
}

unsigned int cpu_read_obmem(unsigned int address, int size)
{
  if (0) printf("cpu_read_obmem address %x (%d)\n", address, size);

#ifdef TEK4404
  if (address >= 0x600000 && address < 0x620000) {
    return sun2_video_read(address, size);
  }
  if (address >= 0x760000 && address < 0x770000) {
			return READ_WORD(g_debugram, address & 0xfff);
  }
#endif

  if (address >= 0x700000 && address < 0x780000) {
    return sun2_video_read(address, size);
  }

  if (address >= 0x780000 && address < 0x780100) {
    return sun2_kbm_read(address, size);
  }

#ifdef TEK4404
  if (address >= 0x782000 && address < 0x786000) {
    return sun2_video_ctl_read(address, size);
  }
#else
  if (address >= 0x781800 && address < 0x781900) {
    return sun2_video_ctl_read(address, size);
  }
#endif

  return 0xffffffff;
}

void cpu_write_obmem(unsigned int address, int size, unsigned int value)
{
  if (0) printf("cpu_write_obmem address %x (%d) <- %x\n", address, size, value);

#ifdef TEK4404
  if (address >= 0x600000 && address < 0x620000) {
    sun2_video_write(address, size, value);
    return;
  }
  if (address >= 0x760000 && address < 0x770000) {
		WRITE_WORD(g_debugram, address & 0xfff, value);
		return;
  }
#else
  if (address >= 0x700000 && address < 0x780000) {
    sun2_video_write(address, size, value);
    return;
  }
#endif

  if (address >= 0x780000 && address < 0x780100) {
    sun2_kbm_write(address, size, value);
  }

#ifdef TEK4404
  if (address >= 0x782000 && address < 0x786000) {
    sun2_video_ctl_write(address, size, value);
  }
#else
  if (address >= 0x781800 && address < 0x781900) {
    sun2_video_ctl_write(address, size, value);
  }
#endif
}

/* Read data from RAM */
unsigned int cpu_read_byte(unsigned int address)
{
  unsigned int value;

  value = READ_BYTE(g_ram, address);
  if (trace_cpu_rw)
    printf("cpu_read_byte fc=%x %x -> %x\n", g_fc, address, value);

#if 0
  if (address > 0xf00000)
  printf("XXX: cpu_read_byte(0x%x) -> 0x%x\n", address, READ_BYTE(g_ram, address));
#endif

  return value;
}

unsigned int cpu_read_word(unsigned int address)
{
  unsigned int value;

  value = READ_WORD(g_ram, address);
  if (trace_cpu_rw)
    printf("cpu_read_word fc=%x %x -> %x\n", g_fc, address, value);

#if 0
  if (address > 0xf00000)
  printf("XXX: cpu_read_word(0x%x) -> 0x%x\n", address, READ_WORD(g_ram, address));
#endif

  return value;
}

unsigned int cpu_read_long(unsigned int address)
{
  unsigned int value;

  value = READ_LONG(g_ram, address);
  if (trace_cpu_rw)
    printf("cpu_read_long fc=%x %x -> %x\n", g_fc, address, value);

  return value;
}


// debug - check if r/w address is mapped into context 3 (the next user proc)
void _check_write(unsigned pa, unsigned b, int size)
{
  int i, j;

  if (!trace_armed)
    return;

  if ((m68k_get_reg(NULL, M68K_REG_PC) & 0xff0000) == 0xef0000)
    return;

  for (i = 0; i < 0x20; i++) {
    unsigned int segindex, pmeg_number, pgmapindex, pte, mapped_pa;
    segindex = (i << 3) | 3;
    pmeg_number = segmap[segindex];
    pmeg_number = ((pmeg_number << 1) & 0xfe) | (pmeg_number & 0x80 ? 0x01 : 0x00);

    if (pmeg_number == 0)
      continue;

    for (j = 0; j < 16; j++) {
      pgmapindex = (pmeg_number << 4) | j;
      pte = pgmap[pgmapindex];
      if ((pte & 0x80000000) == 0)
	continue;
      mapped_pa = (pte & 0x00fff) << 11;
      mapped_pa &= 0x00ffffff;
      if (mapped_pa == (pa & ~0x7ff)) {
	unsigned va;
	va =  (i << 15) | (j << 11);
	printf("write to mapped context3 space; pa %08x, v %02x, s %d (va %06x); sr %04x, pc %06x\n",
	       pa, b, size, va, 
	       m68k_get_reg(NULL, M68K_REG_SR), m68k_get_reg(NULL, M68K_REG_PC));
	break;
      }
    }
  }
}

/* Write data to RAM or a device */
void cpu_write_byte(unsigned int address, unsigned int value)
{
  if (trace_cpu_rw)
    printf("cpu_write_byte fc=%x %x <- %x @ %x\n", g_fc, address, value, m68k_get_reg(NULL, M68K_REG_PC));

  { extern uint m68ki_fault_pending; if (m68ki_fault_pending) printf("PENDING FAULT! cpu_write_byte %08x\n", address); }
  //_check_write(address, value, 1);

  WRITE_BYTE(g_ram, address, value);
}

void cpu_write_word(unsigned int address, unsigned int value)
{
  if (trace_cpu_rw)
    printf("cpu_write_word fc=%x %x <- %x @ %x\n", g_fc, address, value, m68k_get_reg(NULL, M68K_REG_PC));

  { extern uint m68ki_fault_pending; if (m68ki_fault_pending) printf("PENDING FAULT! cpu_write_word %08x\n", address); }
  //_check_write(address, value, 2);

  WRITE_WORD(g_ram, address, value);
}

void cpu_write_long(unsigned int address, unsigned int value)
{
  if (trace_cpu_rw)
    printf("cpu_write_long fc=%x %x <- %x @ %x\n", g_fc, address, value, m68k_get_reg(NULL, M68K_REG_PC));

  { extern uint m68ki_fault_pending; if (m68ki_fault_pending) printf("PENDING FAULT! cpu_write_long %08x\n", address); }
  //_check_write(address, value, 4);

  if (address < MAX_RAM) {
    WRITE_LONG(g_ram, address, value);
  }
}

/* Called when the CPU pulses the RESET line */
void cpu_pulse_reset(void)
{
  nmi_device_reset();
}

/* Called when the CPU changes the function code pins */
void cpu_set_fc(unsigned int fc)
{
  g_fc = fc;
}

/* Called when the CPU acknowledges an interrupt */
int cpu_irq_ack(int level)
{
  if (level != 7 && trace_irq) printf("cpu_irq_ack(%d)\n", level);
  switch(level)
    {
#ifdef TEK4404
		case IRQ_9513_TIMER1:
			return am9513_device_ack(1);
		case IRQ_DMA:
			/* DMA */
			break;
		case IRQ_SCSI:
			return scc_device_ack(1);
		case IRQ_UART:
			/* UART */
			break;
		case IRQ_VSYNC:
			/* VSYNC */
			break;
		case IRQ_DEBUG:
			/* DEBUG */
			break;
#else
//    case IRQ_NMI_DEVICE:
//      return nmi_device_ack();
    case IRQ_SC:
      return sc_device_ack();
    case IRQ_9513_TIMER1:
      return am9513_device_ack(1);
    case IRQ_9513_TIMER2:
      return am9513_device_ack(2);
    case IRQ_SCC:
      return scc_device_ack(1);
    case IRQ_SW_INT1:
      return sw_int_ack(1);
//    case IRQ_SW_INT2:
//      return sw_int_ack(2);
    case IRQ_SW_INT3:
      return e3c400_device_ack();
#endif
    }
  return M68K_INT_ACK_SPURIOUS;
}


int sw_int_ack(int intr)
{
  if (trace_irq) printf("sysen: sw int ack\n");
  switch (intr) {
  case 1:
    int_controller_clear(IRQ_SW_INT1);
    break;
  case 2:
    int_controller_clear(IRQ_SW_INT2);
    break;
  case 3:
    int_controller_clear(IRQ_SW_INT3);
    break;
  }
  return M68K_INT_ACK_AUTOVECTOR;
}


void sw_int_throw(int intr)
{
  if (trace_irq) printf("sysen: sw int throw\n");
  switch (intr) {
  case 1:
    int_controller_set(IRQ_SW_INT1);
    break;
  case 2:
    int_controller_set(IRQ_SW_INT2);
    break;
  case 3:
    int_controller_set(IRQ_SW_INT3);
    break;
  }
}

/* Implementation for the NMI device */
void nmi_device_reset(void)
{
  g_nmi = 0;
}

//void nmi_device_update(void)
//{
//  if(g_nmi)
//    {
//      g_nmi = 0;
//      int_controller_set(IRQ_NMI_DEVICE);
//    }
//}

//int nmi_device_ack(void)
//{
//  printf("\nNMI\n");fflush(stdout);
//  int_controller_clear(IRQ_NMI_DEVICE);
//  return M68K_INT_ACK_AUTOVECTOR;
//}

static unsigned int sdl_poll_delay;

void io_update(void)
{

/* here, do your time-consuming job */
	
	// tek scsi
	if (tekscsi == 0xff)
	{
		tekscsi = 0;
		tekscsidelay = 5000;
	}

	// wait for 'a bit' before reporting we're done?
	if(--tekscsidelay == 0)
	{
		int_controller_set(IRQ_SCSI);
	}
	
  am9513_update();
  mm58167_update();
  scc_update();
  e3c400_update();

  if (sdl_poll_delay++ == 10000) {
    sdl_poll_delay = 0;
    sdl_poll();
	}
    
}

void io_init(void)
{
  e3c400_init();
  sun2_init();
}

/* Implementation for the interrupt controller */
void int_controller_set(unsigned int value)
{
  unsigned int old_pending = g_int_controller_pending;

  if (value != 7 && trace_irq)
    printf("sim68k: int_controller_set(%d) old_pending %d, INTS_ENABLED %d\n", value, old_pending, INTS_ENABLED);

  g_int_controller_pending |= (1<<value);

  if(old_pending != g_int_controller_pending && value > g_int_controller_highest_int)
    {
      g_int_controller_highest_int = value;
      if (INTS_ENABLED) {
	m68k_set_irq(g_int_controller_highest_int);
      } else
	printf("sim68k: ints not enabled; set %d\n", value);
    }
  else printf("sim68k: tried to set irq %d (old_pending 0x%x, controller_pending 0x%x. highest_int %d)\n",
	      value, old_pending, g_int_controller_pending, g_int_controller_highest_int);
}

void int_controller_clear(unsigned int value)
{
  if (value != 7 && trace_irq)
    printf("sim68k: int_controller_clear(%d)\n", value);

  g_int_controller_pending &= ~(1<<value);

  for(g_int_controller_highest_int = 7;g_int_controller_highest_int > 0;g_int_controller_highest_int--)
    if(g_int_controller_pending & (1<<g_int_controller_highest_int))
      break;

  if (INTS_ENABLED) {
    if (trace_irq)
      printf("sim68k: int_controller_clear(%d) asserting lower int %d (sr %04x)\n",
	     value, g_int_controller_highest_int, m68k_get_reg(NULL, M68K_REG_SR));

    m68k_set_irq(g_int_controller_highest_int);
  }
}

unsigned int m68k_read_disassembler_16(unsigned int address)
{
  if ((m68k_get_reg(NULL, M68K_REG_SR) & 0x2000) == 0) {
    unsigned int mtype, fault;
    unsigned int pa, pte;
    pa = cpu_map_address(address, 1, 0, &mtype, &fault, &pte);
    if (fault) return 0;
    return READ_WORD(g_ram, pa);
  }

  /* hack */
#ifdef TEK4404
  if ((address & 0x00ff0000) == 0x00740000)
    return READ_WORD(g_rom, address & 0xffff);
#else
  if ((address & 0x00ff0000) == 0x00ef0000)
    return READ_WORD(g_rom, address & 0xffff);
#endif

  return READ_WORD(g_ram, address);
//  return cpu_read_word(address);
}

unsigned int m68k_read_disassembler_32(unsigned int address)
{
  if ((m68k_get_reg(NULL, M68K_REG_SR) & 0x2000) == 0) {
    unsigned int mtype, fault;
    unsigned int pa, pte;
    pa = cpu_map_address(address, 1, 0, &mtype, &fault, &pte);
    if (fault) return 0;
    return READ_LONG(g_ram, pa);
  }

  /* hack */
#ifdef TEK4404
  if ((address & 0x00ff0000) == 0x00740000)
    return READ_LONG(g_rom, address & 0xffff);
#else
  if ((address & 0x00ff0000) == 0x00ef0000)
    return READ_LONG(g_rom, address & 0xffff);
#endif

  return READ_LONG(g_ram, address);
//  return cpu_read_long(address);
}

void pending_buserr(void)
{
  g_buserr = 1;
  g_buserr_pc = m68k_get_reg(NULL, M68K_REG_PPC);
  m68k_mark_buserr();

#if 0
  {
    extern unsigned int m68ki_access_pc;
    extern unsigned int m68ki_access_address;
    extern unsigned char m68ki_access_fc;
    extern unsigned char m68ki_access_write;
    extern unsigned char m68ki_access_size;
    printf("pending_buserr: access pc=%x address=%x fc=%d write=%d size=%d\n",
	   m68ki_access_pc, m68ki_access_address, m68ki_access_fc, m68ki_access_write, m68ki_access_size);
  }
#endif
}

unsigned int cpu_map_address(unsigned int address, unsigned int fc, int m, unsigned int *mtype, unsigned int *pfault, unsigned int *ppte)
{
  unsigned int context, segindex, pageindex, offset, pmeg_number, pgmapindex;
  unsigned int pte, pa, prot, proterr;

  *pfault = 0;
  *mtype = 0;

  if (fc == 3)
    return address;

	// AB:  I think this is sort of like U171 flipflop on 4404 but we need fc==5 or fc==6
	
  if ((sysen_reg & SUN2_SYSENABLE_EN_BOOTN) == 0 && fc >= 5) {
    /* boot */
    //printf("map: %x -> %x (boot)\n", address, address);
    *mtype = PGTYPE_OBIO;
    return address;
  }

  /* normal */
  context = fc < 4 ? context_user_reg : context_sys_reg;

  segindex = (((address >> 15) & 0x1ff) << 3) | context;
  pmeg_number = segmap[segindex];
pmeg_number = ((pmeg_number << 1) & 0xfe) | (pmeg_number & 0x80 ? 0x01 : 0x00);
  pageindex = (address >> 11) & 0xf;
  pgmapindex = (pmeg_number << 4) | pageindex;
  pte = pgmap[pgmapindex];

  pgmap[pgmapindex] |= (1<<21) | (m ? 1<<20 : 0);

  /* sun-2/120 implements 12 bits yielding 23 bit pa */
  *ppte = pte & 0xfff00fff;
  *mtype = (pte >> 22) & 0x7;
  prot = (pte >> 25) & 0x3f;

  /*
   * rwxrwx
   * 100000 0x20
   * 010000 0x10
   * 000100 0x04
   * 000010 0x02
   */
  proterr = 0;
  if (m == 0)
    switch (fc) {
    case 1: /* u+r */
      if ((prot & 0x04) == 0) proterr = 1; break;
    case 2: /* u+x */
      if ((prot & 0x01) == 0) proterr = 1; break;
    case 5: /* s+r */
      if ((prot & 0x20) == 0) proterr = 1; break;
    case 6: /* s+x */
      if ((prot & 0x08) == 0) proterr = 1; break;
    }
  else
    switch (fc) {
    case 1: /* u+w */
      if ((prot & 0x02) == 0) proterr = 1; break;
    case 5: /* s+w */
      if ((prot & 0x10) == 0) proterr = 1; break;
    }

  if (proterr) {
    buserr_reg = SUN2_BUSERROR_VALID | SUN2_BUSERROR_PROTERR;
    *pfault = 1;
#if 0
    trace_mmu = 1;
#endif
  }

  /* pte valid? */
  if ((pte & 0x80000000) == 0) {
    buserr_reg = 0;
    *pfault = 1;
  }

  int show = 0;
  if (trace_mmu > 1) show = 1;
  //if (buserr_reg && (fc == 5 || fc == 6)) show = 1;

  if (show) {
    printf("mmu: address %x segindex 0x%x pmeg_number 0x%x pageindex 0x%x pgmapindex 0x%x\n",
	   address, segindex, pmeg_number, pageindex, pgmapindex);
    printf("mmu: pgmap[%d:%08x] -> %x; mtype %d, prot %x;  buserr_reg %04x\n",
	   context, address, pte, *mtype, prot, buserr_reg);
  }

  /*
   * pa:
   *
   * 3322222222221111111111
   * 10987654321098765432109876543210
   *      |   pte page   | offset
   */
  pa = (pte & 0x00fff) << 11;
  offset = address & 0x7ff;

  /* prom is magic - it uses part of va */
  if (*mtype == PGTYPE_OBIO && pa == 0) {
#ifdef TEK4404
 	pa = 0x740000;
#else
   pa = 0xef0000 | (address & 0x00f800);
#endif
  }

  pa |= offset;

  // we're a 24 bit PA cpu 
  pa &= 0x00ffffff;

  if (trace_mmu) {
    printf("map: %x -> %x pte %x p%x m%d fc %d context %x segindex %x pageindex %x pgmapindex %x offset %x\n",
	   address, pa, pte, prot, *mtype, fc, context, segindex, pageindex, pgmapindex, offset);
  }

  return pa;
}

extern void m68k_mark_buserr_fixup(unsigned access_address, int access_size);

unsigned int cpu_read(int size, unsigned int address)
{
  unsigned int mtype, fault;
  unsigned int pa, value, pte;

  // check for page crossing on 32bit read
#ifdef TEK4404
  if (((address & 0xfff) + size-1) > 0xfff) {		// 4096 PAGSIZ
#else
  if (((address & 0x7ff) + size-1) > 0x7ff) {
#endif
    if (size == 4) {
      unsigned v1, v2;

	v1 = cpu_read(2, address);
	if (g_buserr == 0) {
	  v2 = cpu_read(2, address+2);
	  if (g_buserr)
	    m68k_mark_buserr_fixup(address+2, 2);
	}

	return (v1 << 16) | v2;
    }
  }

  pa = cpu_map_address(address, g_fc, 0, &mtype, &fault, &pte);

  if (trace_cpu_rw)
    printf("cpu_read: va %x pa %x fc %x size %d mtype %d fault %d\n", address, pa, g_fc, size, mtype, fault);

  if (g_fc == 3) {
    return mmu_read(address, size);
  }

  value = 0;

  if (fault) {
    if (trace_mmu) {
      printf("fault: read\n");
      trace_all();
    }
    if (sysen_reg & SUN2_SYSENABLE_EN_BOOTN) {
      if (!quiet) {
	printf("fault: bus error! read, pc %x sr %04x isn_count %lu\n",
	       m68k_get_reg(NULL, M68K_REG_PC), m68k_get_reg(NULL, M68K_REG_SR), g_isn_count);

	printf("cpu_read: va %x pa %x fc %x size %d mtype %d fault %d, pte %x\n",
	       address, pa, g_fc, size, mtype, fault, pte);
      }

      pending_buserr();
    }

    value = 0;
  } else {
    switch (mtype) {
    case PGTYPE_OBMEM:
#if 0
      if ((address >= 0x500000 && address < 0x5fffff) || address == 0x2850)
	printf("cpu_read; OBMEM va %x pa %x size %d -> %x (pte %x) @ %x\n",
	       address, pa, size, value, pte, m68k_get_reg(NULL, M68K_REG_PC));
#endif
      if (pa >= (4*1024*1024) && pa < 0x700000)
	value = 0xffffffff;
      else
	if (pa >= 0x700000)
	  value = cpu_read_obmem(pa, size);
      else {
	switch (size) {
	case 1: value = cpu_read_byte(pa); break;
	case 2: value = cpu_read_word(pa); break;
	default:
	case 4:
	  value = cpu_read_long(pa); break;
	}
      }
      break;
    case PGTYPE_OBIO:
      value = io_read(size, address, pa);
      break;
    case PGTYPE_MBMEM:
      if (0) printf("3C400 cpu_read: MBMEM; address %x pa %x size %d pte %x\n", address, pa, size, pte);
      value = cpu_read_mbmem(pa, size);
      break;

    case PGTYPE_MBIO:
      printf("cpu_read: MBIO; address %x pa %x size %d pte %x\n", address, pa, size, pte);
      value = cpu_read_mbio(pa, size);
      break;

    default:
      printf("cpu_read: mtype %d; address %x pa %x size %d pte %x\n", mtype, address, pa, size, pte);
    }
  }

  if (trace_mem_bin)
    trace_file_mem_rd(address, pa, g_fc, size, pte, mtype, fault, value, m68k_get_reg(NULL, M68K_REG_PC));

  return value;
}

void cpu_write(int size, unsigned int address, unsigned int value)
{
  unsigned int mtype, fault;
  unsigned int pa, pte;

  pa = cpu_map_address(address, g_fc, 1, &mtype, &fault, &pte);

  if (trace_cpu_rw)
    printf("cpu_write: va %x pa %x fc %x size %d mtype %d fault %d\n",
	 address, pa, g_fc, size, mtype, fault);

  if (trace_mem_bin)
    trace_file_mem_wr(address, pa, g_fc, size, pte, mtype, fault, value, m68k_get_reg(NULL, M68K_REG_PC));

  if (g_fc == 3) {
    mmu_write(address, value, size);
    return;
  }

  if (fault) {
    if (trace_mmu) {
      printf("fault: write; pc 0x%x, prev pc 0x%x\n",
	     m68k_get_reg(NULL, M68K_REG_PC), m68k_get_reg(NULL, M68K_REG_PPC));
      //trace_all();
    }
    if (sysen_reg & SUN2_SYSENABLE_EN_BOOTN) {
      if (!quiet) {
	printf("fault: bus error! write, pc %x sr %04x isn_count %lu\n",
	       m68k_get_reg(NULL, M68K_REG_PC),	m68k_get_reg(NULL, M68K_REG_SR), g_isn_count);

	printf("cpu_write: va %x pa %x fc %x size %d mtype %d fault %d, pte %x\n",
	       address, pa, g_fc, size, mtype, fault, pte);
      }

      pending_buserr();
    }
    return;
  }

  switch (mtype) {
  case PGTYPE_OBMEM:
#if 0
    if ((address >= 0x500000 && address < 0x5fffff) || address == 0x2850)
      printf("cpu_write; OBMEM va %x pa %x size %d <- %x (pte %x) @ %x\n",
	     address, pa, size, value, pte, m68k_get_reg(NULL, M68K_REG_PC));
#endif
//    if (address > 0xe00000 || pa > 0xe00000)
//      printf("cpu_write; OBMEM va %x pa %x size %d <- %x (pte %x)\n", address, pa, size, value, pte);

#ifdef TEK4404
    if (pa >= 0x600000)
      cpu_write_obmem(pa, size, value);
#else
    if (pa >= 0x700000)
      cpu_write_obmem(pa, size, value);
#endif
    else {
      // check for 32bit writes crossing a page
#ifdef TEK4404
  if (((address & 0xfff) + size-1) > 0xfff) {		// 4096 PAGSIZ
#else
	if (((address & 0x7ff) + size-1) > 0x7ff) {
#endif
	if (size == 4) {
	  cpu_write(2, address,   value >> 16);
	  if (g_buserr == 0) {
	    cpu_write(2, address+2, value);
	    if (g_buserr)
	      m68k_mark_buserr_fixup(address+2, 2);
	  }
	  return;
	}
      }

      switch (size) {
      case 1: cpu_write_byte(pa, value); break;
      case 2: cpu_write_word(pa, value); break;
      default:
      case 4: cpu_write_long(pa, value); break;
      }
    }
    break;
  case PGTYPE_OBIO:
    io_write(size, pa, value);
    break;
  case PGTYPE_MBMEM:
    if (trace_cpu_rw)
      printf("cpu_write: MBMEM; address %x pa %x size %d pte %x\n", address, pa, size, pte);
    cpu_write_mbmem(pa, size, value);
    break;

  case PGTYPE_MBIO:
    if (trace_cpu_rw)
      printf("cpu_write: MBIO; address %x pa %x size %d pte %x\n", address, pa, size, pte);
    cpu_write_mbio(pa, size, value);
#if 0
    if (pa == 0)
      enable_trace(1);
#endif
    break;

  default:
    printf("cpu_write: mtype %d; address %x pa %x size %d pte %x\n", mtype, address, pa, size, pte);
    break;
  }
}
    
/*
  Print some information on the instruction and state.
 */
void trace_short(void)
{
  unsigned int pc;
  char buf[256];

  pc = m68k_get_reg(NULL, M68K_REG_PC);
  m68k_disassemble(buf, pc, M68K_CPU_TYPE_68010); 

  if (pc == 0xef0aa1) {
    printf("PROM ERROR!\n");
    exit(1);
  }

  printf("\n");
  printf("%lu %06x:%s\tSR:%04x A0:%06x A1:%06x A4:%06x A5:%06x A6:%06x A7:%06x D0:%08x D1 %08x D5 %08x\n",
	 g_isn_count, pc, buf,
	 m68k_get_reg(NULL, M68K_REG_SR),
	 m68k_get_reg(NULL, M68K_REG_A0), m68k_get_reg(NULL, M68K_REG_A1),
	 m68k_get_reg(NULL, M68K_REG_A4), m68k_get_reg(NULL, M68K_REG_A5),
	 m68k_get_reg(NULL, M68K_REG_A6), m68k_get_reg(NULL, M68K_REG_A7),
	 m68k_get_reg(NULL, M68K_REG_D0), m68k_get_reg(NULL, M68K_REG_D1),
	 m68k_get_reg(NULL, M68K_REG_D5));
}

int trace_bin_fd;
int trace_bin_history;
unsigned int trace_bin_last[64][20];
unsigned int trace_bin_last_ptr;

void trace_file_history_dump(void)
{
  int i, p;

  printf("trace history:\n");

  p = trace_bin_last_ptr;
  for (i = 0; i < 64; i++) {
    unsigned int *data = &trace_bin_last[p][0];
    int what = data[0] >> 8;
    unsigned int pc, sr, size;

    switch (what) {
    case 1:
    case 2:
      printf("\n");
      pc = data[1];
      sr = data[2];

      printf("pc %06x sr %04x ", pc, sr);

      {
	char buf[256];
	m68k_disassemble(buf, pc, M68K_CPU_TYPE_68010); 
	printf("%s\n", buf);
      }

#if 1
      printf(" D0:%08x D1:%08x D2:%08x D3:%08x D4:%08x D5:%08x D6:%08x D7:%08x\n",
	     data[11], data[12], data[13], data[14], 
	     data[15], data[16], data[17], data[18]);
      printf(" A0:%08x A1:%08x A2:%08x A3:%08x A4:%08x A5:%08x A6:%08x A7:%08x\n",
	     data[3], data[4], data[5], data[6], 
	     data[7], data[8], data[9], data[10]);
#endif
      break;
    case 11:
      printf(" tlb fill; ");
      printf(" pc %06x address %08x va %08x bus_type %d boot %d pte %08x\n",
	     data[1], data[2], data[3], data[4], data[5], data[6]);
      break;
    case 12:
      printf(" pte set; ");
      printf(" pc %06x address %08x pa %08x bus_type %d context_user %d pte %08x\n",
	     data[1], data[2], data[3], data[4], data[5], data[6]);
      break;
    case 21:
      size = data[2] & 0xffff;
      printf(" mem write; ");
      printf(" pc %06x size %d fc %d ea %08x value %08x\n",
	     data[1], size, data[3], data[4], data[5]);

      {
	int mtype, fault;
	mtype = (data[2] >> 16) & 0xff;
	fault = (data[2] >> 24) & 0xff;
	printf(" pa %x mtype %d fault %d pte %08x\n", data[6], mtype, fault, data[7]);
      }
      break;
    case 22:
      size = data[2] & 0xffff;
      printf(" mem read; ");
      printf(" pc %06x size %d fc %d ea %08x value %08x\n",
	     data[1], size, data[3], data[4], data[5]);
      {
	int mtype, fault;
	mtype = (data[2] >> 16) & 0xff;
	fault = (data[2] >> 24) & 0xff;
	printf(" pa %x mtype %d fault %d pte %08x\n", data[6], mtype, fault, data[7]);
      }
      break;
    }

    if (++p == 64) p = 0;
  }
}

void trace_file_entry(int what, unsigned int *record, int size)
{
  int ret, bytes;

  if (trace_bin_history) {
    int i;
    for (i = 1; i < size; i++)
      trace_bin_last[trace_bin_last_ptr][i] = record[i];
    trace_bin_last[trace_bin_last_ptr][0] = (what << 8) | size;

    trace_bin_last_ptr++;
    if (trace_bin_last_ptr == 64)
      trace_bin_last_ptr = 0;
    return;
  }

  if (trace_bin_fd == 0) {
#ifdef __linux__
    int flags = O_CREAT | O_TRUNC | O_LARGEFILE | O_WRONLY;
#else
    int flags = O_CREAT | O_TRUNC | O_WRONLY;
#endif

    trace_bin_fd = open("trace.bin", flags, 0666);
  }
  if (trace_bin_fd != 0) {
    record[0] = (what << 8) | size;
    bytes = sizeof(unsigned int)*size;
    ret = write(trace_bin_fd, (char *)record, bytes);
    if (ret != bytes)
      ;
  }
}

void trace_file_mem_rd(unsigned int ea, unsigned int pa, int fc, int size, unsigned int pte,
		       int mtype, int fault, unsigned int value, unsigned int pc)
{
  unsigned int record[20];
  record[0] = (22 << 8) | 8;
  record[1] = pc;
  record[2] = (fault << 24) | (mtype << 16) | size;
  record[3] = fc;
  record[4] = ea;
  record[5] = value;
  record[6] = pa;
  record[7] = pte;

  trace_file_entry(22, record, 8);
}

void trace_file_mem_wr(unsigned int ea, unsigned int pa, int fc, int size, unsigned int pte,
		       int mtype, int fault, unsigned int value, unsigned int pc)
{
  unsigned int record[20];
  record[0] = (21 << 8) | 8;
  record[1] = pc;
  record[2] = (fault << 24) | (mtype << 16) | size;
  record[3] = fc;
  record[4] = ea;
  record[5] = value;
  record[6] = pa;
  record[7] = pte;

  trace_file_entry(21, record, 8);
}

void trace_file_pte_set(unsigned int address, unsigned int va, int bus_type, int boot, unsigned int pte)
{
  unsigned int record[20];
  record[0] = (12 << 8) | 8;
  record[1] = m68k_get_reg(NULL, M68K_REG_PC);
  record[2] = address;
  record[3] = va;
  record[4] = bus_type;
  record[5] = boot;
  record[6] = pte;
  record[7] = 0;
  trace_file_entry(12, record, 8);
}

void trace_file_cpu(void)
{
  int ret;
  unsigned int record[20];

  record[0] = (2 << 8) | 20;
  record[1] = m68k_get_reg(NULL, M68K_REG_PC);
  record[2] = m68k_get_reg(NULL, M68K_REG_SR);
  record[3] = m68k_get_reg(NULL, M68K_REG_A0);
  record[4] = m68k_get_reg(NULL, M68K_REG_A1);
  record[5] = m68k_get_reg(NULL, M68K_REG_A2);
  record[6] = m68k_get_reg(NULL, M68K_REG_A3);
  record[7] = m68k_get_reg(NULL, M68K_REG_A4);
  record[8] = m68k_get_reg(NULL, M68K_REG_A5);
  record[9] = m68k_get_reg(NULL, M68K_REG_A6);
  record[10] = m68k_get_reg(NULL, M68K_REG_A7);

  record[11] = m68k_get_reg(NULL, M68K_REG_D0);
  record[12] = m68k_get_reg(NULL, M68K_REG_D1);
  record[13] = m68k_get_reg(NULL, M68K_REG_D2);
  record[14] = m68k_get_reg(NULL, M68K_REG_D3);
  record[15] = m68k_get_reg(NULL, M68K_REG_D4);
  record[16] = m68k_get_reg(NULL, M68K_REG_D5);
  record[17] = m68k_get_reg(NULL, M68K_REG_D6);
  record[18] = m68k_get_reg(NULL, M68K_REG_D7);
  record[19] = 0;
  trace_file_entry(2, record, 20);
}

void trace_all()
{
  unsigned int pc;
  char buf[256];

  pc = m68k_get_reg(NULL, M68K_REG_PC);
  m68k_disassemble(buf, pc, M68K_CPU_TYPE_68010); 

  printf("\n");
  printf("PC %06x sr %04x usp %06x isp %06x sp %06x context %02x%02x\n",
	 m68k_get_reg(NULL, M68K_REG_PC),
	 m68k_get_reg(NULL, M68K_REG_SR),
	 m68k_get_reg(NULL, M68K_REG_USP),
	 m68k_get_reg(NULL, M68K_REG_ISP),
	 m68k_get_reg(NULL, M68K_REG_SP),
	 context_sys_reg, context_user_reg
	 /*m68k_get_reg(NULL, M68K_REG_VBR)*/);

  printf("%s\n", buf);

  printf("A0:%08x A1:%08x A2:%08x A3:%08x A4:%08x A5:%08x A6:%08x A7:%08x\n",
	 m68k_get_reg(NULL, M68K_REG_A0), m68k_get_reg(NULL, M68K_REG_A1),
	 m68k_get_reg(NULL, M68K_REG_A2), m68k_get_reg(NULL, M68K_REG_A3),
	 m68k_get_reg(NULL, M68K_REG_A4), m68k_get_reg(NULL, M68K_REG_A5),
	 m68k_get_reg(NULL, M68K_REG_A6), m68k_get_reg(NULL, M68K_REG_A7));

  printf("D0:%08x D1:%08x D2:%08x D3:%08x D4:%08x D5:%08x D6:%08x D7:%08x\n",
	  m68k_get_reg(NULL, M68K_REG_D0), m68k_get_reg(NULL, M68K_REG_D1),
	  m68k_get_reg(NULL, M68K_REG_D2), m68k_get_reg(NULL, M68K_REG_D3),
	  m68k_get_reg(NULL, M68K_REG_D4), m68k_get_reg(NULL, M68K_REG_D5),
	  m68k_get_reg(NULL, M68K_REG_D6), m68k_get_reg(NULL, M68K_REG_D7));
}

void enable_trace(int what)
{
  switch (what) {
  case 0:
    trace_cpu_io = 0;
    trace_cpu_rw = 0;
    trace_cpu_isn = 0;
    trace_mmu = 0;
    trace_mmu_rw = 0;
    break;

  case 1:
  default:
    trace_cpu_io = 1;
    trace_cpu_rw = 1;
    trace_cpu_isn = 1;
    break;

  case 2:
    trace_cpu_io = 1;
    trace_cpu_rw = 1;
    trace_cpu_isn = 1;
    trace_mmu = 1;
    trace_mmu_rw = 1;
    break;

  case 3:
#if 0
    trace_cpu_io = 1;
    trace_cpu_rw = 1;
    trace_cpu_isn = 1;

    trace_mmu    = 1;
    trace_mmu_rw = 1;
#endif
    break;
  }
}

void xxx(void)
{
  //trace_all();
  //enable_trace(3);
}

char cc[256];
int cc_n;

void collect_console(int ch)
{
  cc[cc_n++] = ch;
  if (ch == '\n') {
    cc[cc_n-1] = 0;
    printf("console: [%s]\n", cc);
    cc_n = 0;
  }
}


void sim68k(void)
{
  int c;
  unsigned long isn_count = 0;
  int quanta;

  // put in values for the stack and PC vectors
#ifdef TEK4404
  WRITE_LONG(g_ram, 0, 0x740000);   // SP
  WRITE_LONG(g_ram, 4, 0x740000);   // PC
#else
  WRITE_LONG(g_ram, 0, 0xfe0000);   // SP
  WRITE_LONG(g_ram, 4, 0xfe0000);   // PC
#endif

  /*
    Install a handler for various termination events so that we have the
    opportunity to write the simulated file systems back. Plus clean up
    anything else required.
   */
  if (signal (SIGINT, termination_handler) == SIG_IGN)
    signal (SIGINT, SIG_IGN);
  if (signal (SIGHUP, termination_handler) == SIG_IGN)
    signal (SIGHUP, SIG_IGN);
  if (signal (SIGTERM, termination_handler) == SIG_IGN)
    signal (SIGTERM, SIG_IGN);

g_trace = 1;

#ifndef M68K_V33
  m68k_init();
#endif
  m68k_set_cpu_type(M68K_CPU_TYPE_68010);
  m68k_pulse_reset();
//  m68k_set_reg(M68K_REG_VBR, 0x00ef0000);
//trace_all();

  nmi_device_reset();
  io_init();

  trace_cpu_bin = trace_mmu_bin = trace_mem_bin = 0;


	// need interrupts for SCSI and memory tests
	sysen_reg |= SUN2_SYSENABLE_EN_INT;



  while(1)
    {
      if (g_buserr) {
	g_buserr = 0;
	m68k_set_buserr(g_buserr_pc);
      }

      if(trace_cpu_isn) {
	//trace_short();
	trace_all();
      }
      if (trace_cpu_bin) {
	trace_file_cpu();
      }

      quanta = 1/*g_trace ? 1 : 10000*/;
      m68k_execute(quanta);

//      nmi_device_update();
      io_update();

      isn_count++;
      g_isn_count = isn_count;

      if (0) {
	if ((isn_count % 1000000) == 0) {
	  printf("--- %lu ---", isn_count);
	  trace_short();
	}
      }

#if 1
      if (m68k_get_reg(NULL, M68K_REG_PC) == 0x12516) {  // sunos 2.0 _putchar
	unsigned int d0;
	d0 = m68k_get_reg(NULL, M68K_REG_D0);
	collect_console(d0);
      }
#endif

#if 0
      if (m68k_get_reg(NULL, M68K_REG_PC) == 0x8000 &&
	  (m68k_get_reg(NULL, M68K_REG_SR) & 0x2000) == 0)
      {
	enable_trace(1);
      }
#endif
#if 0
      if ((m68k_get_reg(NULL, M68K_REG_SR) & 0x2000) == 0) {
	enable_trace(2);
      }
#endif

#if 0
      if ((m68k_get_reg(NULL, M68K_REG_SR) & 0x2000) == 0) {
	enable_trace(2);
      } else {
	enable_trace(0);
      }
#endif

#if 0
      if (trace_armed && (m68k_get_reg(NULL, M68K_REG_SR) & 0x2000) == 0) {
	enable_trace(2);
      }
#endif

#if 0
      if (m68k_get_reg(NULL, M68K_REG_SR) & 0xc000) {
	enable_trace(2);
      } else {
	enable_trace(0);
      }
#endif

#if 0
      if (isn_count >= (24*1000*1000)) {
	while (1) sdl_poll();
	break;
      }
#endif

    }

}

#ifndef M68K_V33
unsigned int  m68k_read_memory_8(unsigned int address)
{
  return cpu_read(1, address);
}

unsigned int  m68k_read_memory_16(unsigned int address)
{
  return cpu_read(2, address);
}

unsigned int  m68k_read_memory_32(unsigned int address)
{
  return cpu_read(4, address);
}


/* Write to anywhere */
void m68k_write_memory_8(unsigned int address, unsigned int value)
{
  cpu_write(1, address, value);
}

void m68k_write_memory_16(unsigned int address, unsigned int value)
{
  cpu_write(2, address, value);
}

void m68k_write_memory_32(unsigned int address, unsigned int value)
{
  cpu_write(4, address, value);
}


#endif

/* Local Variables:  */
/* mode: c           */
/* c-basic-offset: 2 */
/* End:              */
