#include <QCoreApplication>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>    	/* Error number definitions */
#include <termios.h>    /* POSIX terminal control definitions */
#include <byteswap.h>

//adding readline libraries - Hugh.
#include <readline/readline.h>
#include <readline/history.h>

//4/1 KSS these might be used in the motor and shutter control portions?
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string.h>

#include <QTcpSocket>
#include <QTcpServer>
//#include <QThread>
#include <QtConcurrent/QtConcurrent>
#define SOCKET_PORT 66623

//--------------------------------------------------------------------------------------------
//  1-16-2019  Kate - Edited set_default_biases to have appropriate default values
//  1-16-2019  Kate - Added loadTStripe
//  1-17-2019  John - Added lvds_en
//  1-18-2019  John - Added
//  1-25-2019  John - Adding more commands bank0_en, setbank0tp, setbank0pixel, Dbg_Tpatt_en
//  2-19-2019  Kate - Add loadTSarb for loading arbitrary test patterns for in-pixel test sources [NB this is deleted as of 4/8 because it's redundant with loadTSource]
//  4-1-2019   Kate - Add functions for motor and shutter control
//  4-8-2019   Kate - major reorganization - deleting some deprecated functions (loadTSource2, set_bias_gr, taken_clear, config_bias_gr, set_bin, program_exposure_data, set_biases_new,
//						loadTSarb) and reorganizing remaining functions into functional sections
//  4-9-2019   John - Had some merger problems that needed to be fixed.  It compiles correctly now. Hopefully it's good
//  4-9-2019   John - Put dtpatt_en back into declarations and string parser.
//  4-21-2019  Hugh - Converted to qt project. Added thread for TCOP connection on port 66623. Moved command parser to function called by both command line and TCP/IP threads.
//--------------------------------------------------------------------------------------------
int fd;
//char cmd[256];
char *cmd;
uint16_t TSdata[16]; //10/6 - was 286 elements for hdrpad
u_char packet[150];
u_char packet2[150];
u_char *FramePtr;

int tty_initialized=0;
int num, numexp;
long Itime, Ftime, Fcount;

//4/1 KSS add variables for motor control:
int motor_fd1;
int motor_initialized=0;

//--------------------------------------------------------------------------------------------
//	Function Prototypes
//--------------------------------------------------------------------------------------------
//--------- general low-level control:
int initialize_tty(void);
int set_bias(void);
int adc_spi(void);
int set_ccr(void);
int set_default_biases(void);
int lvds_en(void);
int initialize_pixel_th(void);
int write_uBReg(void);
int read_uBReg(void);

//--------- exposure control:
int set_exp(void);
int taken(void);
int von(void);
int voff(void);
int program_exposure_data_tweak(void);
int program_exposure_data_tweak2(void);

//--------- digital edge test pattern programming and control:
int set_detp(void);
int atpatt_en(void);
int dtpatt_en(void);

//--------- in-pixel test source programming and control:
int progTSource(void);
int loadTSource(void);
int TurnTSOff(void);
int TurnTSOn(void);
int loadTSmile1(void);
int loadTSpixel(void);
int loadTSX(void);
int loadTPixPat1(void);
int loadTStripe(void);

//--------- Bank 0 test pattern programming and control:
int set_bank0tp(void);
int set_bank0pixel(void);
int bank0tp_en(void);

//--------- motor and shutter control:
int initialize_motor(void);
float get_motor_vel(void);
void set_motor_vel(void);
float get_motor_pos(void);
void set_motor_pos(void);
int set_shutter(void);

//--------- custom exposure loops
int custloop(void);
int custloop2(void);
int custloop3(void);
int	custloop4(void);
int	custloop5(void);
int	smileloop(void);
int	smileloop_mini(void);
int	Xloop_mini(void);
int	Xloop_maxi(void);
int	Xloop_itime(void);
int	Xloop_pixpat_itime(void);
int	Xloop_pixpat_itime_loop(void);
int	Xloop_pixpat_itime_loop2(void);
int isweep(void);
int isweep2(void);

//--------- DEPRECATED - to be deleted:
int ClearCnt(void);
int test_bias(void);
int ceb_en(void);

//--------- not sure what the following are for - may be deprecated?:
int ana_phase(void);

void CUAPStcpThread(void);
int commandParser(char*);

FILE *fp;




int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    int cmdStat;

    tty_initialized=0;
        FramePtr = (u_char *) packet;
        initialize_tty();
        //FILE *fp;
        const char* readlineHist=".readlineHistory";

        // Adding history file opening.  Hugh.

        if( access( ".padCommandHistory", F_OK ) != -1 )
        {
                // file exists
                printf("loading old history file \n");
                fp = fopen(".padCommandHistory","a+");
                if( fp == NULL)
                {
                    printf("failes to open file...\n");
                }
        }
        else
        {
                // file doesn't exist
                printf("creating new history file \n");
                fp = fopen(".padCommandHistory","a+");
                if( fp == NULL)
                {
                    printf("failes to open file...\n");
                }
        }
        // end add history file

        read_history(readlineHist);

        QFuture<void> future1 = QtConcurrent::run(CUAPStcpThread);

        while(1)
        {

            cmd = readline("Command: ");

            tcflush(fd,TCIOFLUSH);
            fprintf(fp,"%s ",cmd); // print command to history file
            add_history(cmd);
            //printf(" %s \n",cmd);
            cmdStat=commandParser(cmd);

            if(cmdStat == -2)
            {
                write_history(readlineHist);
                fclose(fp);
                a.quit();
            }
            if(cmdStat == -1)
            {
                printf("\" %s \" confuses me.\n", cmd );
            }

            fprintf(fp,"\n");
            free(cmd);
        }

        write_history(readlineHist);
        fclose(fp);
        return a.exec();

}



//--------------------------------------------------------------------------------------------
//	Initialize
//--------------------------------------------------------------------------------------------
int initialize_tty(void)
{
    if(tty_initialized)
    {
        printf("tty already initialized\n");
        return 0;
    }
    printf("initializing tty interface\n");

    tty_initialized=1;
    //fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY); //note that program must be run as sudo to initialize tty
        fd = open("/dev/serial/by-id/usb-Silicon_Labs_CP2103_USB_to_UART_Bridge_Controller_0001-if00-port0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        printf("Unable to open /dev/serial/by-id/usb-Silicon_Labs_CP2103_USB_to_UART_Bridge_Controller_0001-if00-port0 = /dev/ttyUSB0 or ttyUSB1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
        tty_initialized=0;
    }
    else { fcntl(fd, F_SETFL, 0); }

    struct termios options;
    tcgetattr(fd, &options);						// Get the current options for the port
    cfsetispeed(&options, B115200);						// Set the baud rates to 19200
    cfsetospeed(&options, B115200);						// Set the baud rates to 19200
    options.c_cflag |= (CLOCAL | CREAD);				// Enable the receiver and set local mode
    options.c_cflag &= ~PARENB;							// No parity
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;        					// Mask the character size bits
    options.c_cflag |= CS8;								// Select 8 data bits
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	// raw input - characters are passed through exactly as they are received
    //test setting new option to get rid of new line echo - HP - Sept. 26, 2017
    //options.c_lflag &= ~ECHONL;
    options.c_oflag &= ~(OPOST);
    // end new options
    tcsetattr(fd, TCSANOW, &options);	// Set the new options for the port
    tcflush(fd,TCIOFLUSH);			// Flush both data received but not read, and data written but not transmitted
    usleep(20000);

    return (fd);
}


//================================================================================================
//======================================general low-level control=================================
//================================================================================================

//---------program biases supplied to ASIC by DACs on the PCB - usage: bias [bias #] [DAC value]
int set_bias(void)
{
    //syntax: bias [command ID in decimal] [DAC setting in decimal (16-bit -> between 0 and 16383)]
    //bit pattern sent on DIN_DAC1,2 should be 24 bits: 0000 [channel #, 4 bits] 11 [DAC setting, 12 bits] 00
    int d1;
    //long d2; in HDR-PAD code d2 was long = 4 bytes. here 2 bytes are needed.
    int d2;

    char* substr;
    int cnt=0;

    tcflush(fd,TCIOFLUSH);

//	scanf("%d %d", &d1, &d2);
        //

    //-------begin command parser-----------------------------------------------------------------
        /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.  In this case, looking for 3 values.
     * If number of arguments is not what is expected, it will return without action and a
     * message. Return value will be 1.
     * This basic template is used for all functions that take arguments.
     */

    if(strlen(cmd)<=5)
    {//
        free(cmd);
        cmd = readline("bias> ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        d1 = atoi(cmd);
        free(cmd);

        cmd = readline("value> ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        d2 = atoi(cmd);
        cnt=2;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                d1 = atoi(substr);
            }
            if(cnt==2)
            {
                d2 = atoi(substr);
            }
        }
    }
    //
    if(cnt!=2)
    {
        printf("syntax error: useage:  bias <bias to be set> <value> \n or simply: bias \n second form will prompt \n");
        return 1;
    }
    //-------end command parser-------------------------------------------------------------------

    //command:
    packet[0] = 0x16;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;
    //data 1:
    packet[4] = d1;

    //*(int *)(FramePtr +  5) = __bswap_32(d2*2);
    *(int *)(FramePtr + 5) = __bswap_16(d2);

    for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    printf("wrote the packet\n");
    usleep(20000);
    read(fd, packet2, 16);					// Read output state
    printf("read the packet\n");
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}

//---------set all bias DACs to default values as specified by defval below - usage: default_bias
int set_default_biases(void)
{
    //set default voltage and current biases.

    // VALUES IN ARRAYS have been changed to: (jtw 11-6-2018)
    // {ISS_BUFK, IDiffInBufBias, IDiffBias, IcmFBBuff, ISS_COMP, ISS_ABK, ISS_HDR_EDGE_BUF, ISS_BUF, ISS_OSCK, ISS1,
        //  VTHRK, VREFK, V_MOSRES_BK, VHIK, PMOS_TEST_VG, VLOK, VNB, V_GR, VHC_bright, VHC_dim, V_CM, VtestSig, VtestRef,
    //  VOCM1, VOCM2

    //command IDs:
    int comID[25] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    //default values (DAC code, 0 to 16383):
    int defval[25] = {10778, 10502, 16383, 16221, 13517, 3260, 4137, 8110, 13540, 0, 4533, 9051, 15018, 9051, 16275, 4533, 6560,
                          9076, 9051, 4531, 8164, 9066, 9061, 128, 197};


    int i;

    for(i=0; i < 25; i++)
    {
        //command:
        packet[0] = 0x16;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        //data 1:
        packet[4] = comID[i];

        //*(int *)(FramePtr +  5) = __bswap_32(d2*2);
        *(int *)(FramePtr + 5) = __bswap_16(defval[i]);

        for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';

        write(fd, packet, 16);
        usleep(25000);
        read(fd, packet2, 16);								// Read output state
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
            printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
            printf("\n");
        usleep(20000);
    }

    return 0;
}

// writes a 24 bit word to the adc_spi
// can put into test pattern modes
// adcspi 0xdata
int adc_spi(void)
{
   char str [20];
   unsigned int spiData;	// 24 bits of Data to write to ADC SPI in hex


   if(strlen(cmd)<=7)
   {
      printf("syntax error: useage: 'adcspi 0x1abcde \n");
      return 1;
   }
   else
   {
      //printf("Printing a copy of the command = %s \n", cmd);
      sscanf(cmd, "%s %x", str, &spiData);
      //printf("Printing sscanf results = %s %x \n", str, uB_Address);
   }


   packet[0] = 0x0C;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = (spiData & 0x00ff0000) >> 16;
   packet[5] = (spiData & 0x0000ff00) >> 8;
   packet[6] = (spiData & 0x000000ff);

   for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(20000);
   read(fd, packet2, 16);                        // Read output state
   usleep(20000);
   for(num=0;num<16;num++){printf("%02x:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02x:", packet2[num]);}
   printf("\n");

   return 0;
}

//---------program chip control register - as of 4/8/2019 the typical working state is 0x4991
int set_ccr(void)
{
   char str [20];
   unsigned int ccrData;	// 16 bits into ccr


   if(strlen(cmd)<=7)
   {
      printf("syntax error: useage: 'setccr 0x12bc \n");
      return 1;
   }
   else
   {
      sscanf(cmd, "%s %x", str, &ccrData);
      //printf("Printing sscanf results = %s %x \n", str, uB_Address);
   }

   packet[0] = 0x1D;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = (ccrData & 0x0000ff00) >> 8;
   packet[5] = (ccrData & 0x000000ff);

   for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(20000);
   read(fd, packet2, 16);                        // Read output state
   usleep(20000);
   for(num=0;num<16;num++){printf("%02x:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02x:", packet2[num]);}
   printf("\n");

   return 0;
}

//---------enable/disable LVDS drivers - usage: lvds_en [0/1]
int lvds_en(void)
{
    //set LVDS_EN  1 = lvds enabled and use lvds data in, 0 = lvds disabled and use single ended data in
   tcflush(fd,TCIOFLUSH);

   int lvds; //1 for enable, 0 for disable
   //scanf("%d", &ceb);
    char* substr;
    int cnt=0;
    //-------begin command parser-----------------------------------------------------------------
    /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.  In this case, looking for 3 values.
     * If number of arguments is not what is expected, it will return without action and a
     * message. Return value will be 1.
     * This basic template is used for all functions that take arguments.
     */


   if(strlen(cmd)<=8)
    {
        //
        free(cmd);
        cmd = readline("enable? ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        lvds = atoi(cmd);
        cnt = 1;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                lvds = atoi(substr);
            }
        }
    }

    if(cnt!=1)
    {
        printf("syntax error: useage: 'lvds_en <value>' or 'lvds_en' for prompt \n");
        return 1;
    }
    //-------end command parser-------------------------------------------------------------------

   packet[0] = 0x1C;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = lvds;

   for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(50000);
   read(fd, packet2, 16);                        // Read output state

   for(num=0;num<16;num++){printf("%02d:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
   printf("\n");

   return 0;
}

//---------initialize pixel track-and-hold
int initialize_pixel_th(void)
{
    //clear pixel counters
    tcflush(fd,TCIOFLUSH);

    //command:
    packet[0] = 0x07;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    for(num = 4; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(20000);
    read(fd, packet2, 16);								// Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}

//---------write to a specific microblaze regsiter
int	write_uBReg(void)
{
   char str [20];
   unsigned int uB_Address;	// Address to write
   unsigned int uB_Data;	// Data to write

   if(strlen(cmd)<=6)
   {
      printf("syntax error: useage: 'write 0xeb340000 0xabcd\n");
      return 1;
   }
   else
   {
      //printf("Printing a copy of the command = %s \n", cmd);
      sscanf(cmd, "%s %x %x", str, &uB_Address, &uB_Data);
      //printf("Printing sscanf results = %s %x \n", str, uB_Address);
   }


   packet[0] = 0x1A;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = (uB_Address & 0xff000000) >> 24;
   packet[5] = (uB_Address & 0x00ff0000) >> 16;
   packet[6] = (uB_Address & 0x0000ff00) >> 8;
   packet[7] = (uB_Address & 0x000000ff);
   packet[8] = (uB_Data & 0xff000000) >> 24;
   packet[9] = (uB_Data & 0x00ff0000) >> 16;
   packet[10] = (uB_Data & 0x0000ff00) >> 8;
   packet[11] = (uB_Data & 0x000000ff);

   for(num = 12; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(20000);
   read(fd, packet2, 16);                        // Read output state
   usleep(20000);
   for(num=0;num<16;num++){printf("%02x:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02x:", packet2[num]);}
   printf("\n");

   return 0;
}

//---------read from a specific microblaze register
int	read_uBReg(void)
{
   char str [20];
   unsigned int uB_Address;	// Address to read in

   if(strlen(cmd)<=5)
   {
      printf("syntax error: useage: read 0xeb340000\n");
      return 1;
   }
   else
   {
      //printf("Printing a copy of the command = %s \n", cmd);
      sscanf(cmd, "%s %x", str, &uB_Address);
      //printf("Printing sscanf results = %s %x \n", str, uB_Address);
   }

   // packet[0] = 0x20;
   packet[0] = 0x1F;		// fix by jtw 3/18/2019 - 31 decimal
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = (uB_Address & 0xff000000) >> 24;
   packet[5] = (uB_Address & 0x00ff0000) >> 16;
   packet[6] = (uB_Address & 0x0000ff00) >> 8;
   packet[7] = (uB_Address & 0x000000ff);

   for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(20000);
   read(fd, packet2, 16);                        // Read output state
   usleep(20000);
   for(num=0;num<16;num++){printf("%02x:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02x:", packet2[num]);}
   printf("\n");

   return 0;
}



int set_exp(void)
{
    int temp;
    tcflush(fd,TCIOFLUSH);
    char* substr;
    int numframes=0;
    int cnt=0;

    //-------begin command parser-----------------------------------------------------------------
    /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.  In this case, looking for 3 values.
     * If number of arguments is not what is expected, it will return without action and a
     * message. Return value will be 1.
     * This basic template is used for all functions that take arguments.
     */

    if(strlen(cmd)<=7)
    {//
        free(cmd);

        cnt = 1;
        cmd = readline("frame count> ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        numframes = atol(cmd);
        free(cmd);
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                numframes = atol(substr);
            }

        }

    }

    if(cnt!=1)
    {
        printf("count %d \n",cnt);
        printf("usage: setexp <numframes> \n     or setexp   --- gives you prompt. \n");
        return 1;
    }
    printf("Values set: numframes: %d \n", numframes);
    //-------end command parser-------------------------------------------------------------------

    // Sending number of frames
    packet[ 0] = 0x01;
    packet[ 1] = 15;
    packet[ 2] = 0x00;
    packet[ 3] = 0x00;

    //*(int *)(FramePtr + 4) = numframes;
    *(int *)(FramePtr + 4) = __bswap_32(numframes);

    for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(200000);
    read(fd, packet2, 16);								// Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    usleep(200000);


    return 0;
}


//---------take exposures - usage: taken
int taken(void)
{
    tcflush(fd,TCIOFLUSH);
    packet[0] = 0x02;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    for(num = 4; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(20000);
    read(fd, packet2, 16);								// Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}


//---------turn video mode on
int von(void)
{
    tcflush(fd,TCIOFLUSH);
    packet[0] = 0x03;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    for(num = 4; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(20000);
    read(fd, packet2, 16);								// Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}

//---------turn video mode off
int voff(void)
{
    tcflush(fd,TCIOFLUSH);
    packet[0] = 0x04;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    for(num = 4; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(20000);
    read(fd, packet2, 16);								// Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}

//---------program exposure data: transition times of control signals. This version may be deprecated but is used in some of the custom exposure loops...
//---------allows you to set the exposure time (in # of 10ns master clock cycles) and number of frames manually.
//---------usage: program_exposure_data_tweak [exposure time] [number of frames]
int program_exposure_data_tweak(void)
{
    //set transition times for CLB_IN, PRB_IN, EXT_RST,END_EXP, READ, BIN, and end of frame time
    //for frame setup
    //take integration time in clock cycles as an argument. this sets the delay between 1st transition of EXT_RSTB and 2nd transition of END_EXP

    int temp;
    tcflush(fd,TCIOFLUSH);
    char* substr;
    int expt=0;
    int end_exp_secondedge=0; //location of the second transition of END_EXP - determines end of integration - will be computed in this function
    int ext_rst_firstedge=6; //location of the first transition of EXT_RST - determines start of integration - hard-coded
    int ext_rst_secondedge=0; //location of the second transition of EXT_RST - will be computed in this function to be 1 clock cycle after end of integration
    int endt = 0; //location of the END signal - will be computed in this function to be 2 clock cycles after end of integration
    int cnt=0;
    int framecount=1;

    //-------begin command parser-----------------------------------------------------------------
    /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.  In this case, looking for 3 values.
     * If number of arguments is not what is expected, it will return without action and a
     * message. Return value will be 1.
     * This basic template is used for all functions that take arguments.
     */

    if(strlen(cmd)<=7)
    {//
        free(cmd);

        cnt = 1;
        cmd = readline("exposure time [number of clock cycles]> ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        expt = atol(cmd);
        free(cmd);
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                expt = atol(substr);
            }

            if(cnt==2)
            {
                framecount = atol(substr);
            }

        }

    }

    if((cnt!=1)&&(cnt!=2))
    {
        printf("count %d \n",cnt);
        printf("usage: program_exposure_data <exposure time> \n     or program_exposure_data   --- gives you prompt. \n");
        return 1;
    }
    printf("Values set: exposure time [number of clock cycles]: %d \n", expt);
    //-------end command parser-------------------------------------------------------------------

    //compute other edge locations that depend on expt:
    end_exp_secondedge = ext_rst_firstedge + expt;
    ext_rst_secondedge = end_exp_secondedge + 2;
    endt = end_exp_secondedge + 4;

    //command IDs:
    int comID[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    //default values (1-13: # of clock cycles from initial timestamp. clock period is 10ns
    //					14: frame delay - # of clock cycles [from what to what?]
    //					15: frame count
    //int defval[15] = {1, end_exp_secondedge+1, 4, 5, 2, 3, ext_rst_firstedge, ext_rst_secondedge, 1, end_exp_secondedge, 5, 6, endt, 3, framecount};
    int defval[15] = {1, end_exp_secondedge+1, 4, 5, 2, 3, ext_rst_firstedge, ext_rst_secondedge, 1, end_exp_secondedge, 5, 6, endt, 100, framecount};

    printf("\
            Read 	edge1: %d	edge2: %d \n \
            CLB	edge1: %d	edge2: %d \n \
            PRB  	edge1: %d	edge2: %d \n \
            ext_rst	edge1: %d	edge2: %d \n \
            end_exp	edge1: %d	edge2: %d \n \
            bin	edge1: %d	edge2: %d \n \
            end time:		%d \n \
            frame delay:	%d \n \
            frame count:	%d \n",
            defval[0]*10,defval[1]*10,defval[2]*10,defval[3]*10,
            defval[4]*10,defval[5]*10,defval[6]*10,defval[7]*10,
            defval[8]*10,defval[9]*10,defval[10]*10,defval[11]*10,
            defval[12]*10,defval[13]*10,defval[14]);

    int i;

    for(i=0; i < 15; i++)
    {
        //command:
        packet[0] = 0x01;
        packet[1] = comID[i];
        packet[2] = 0x00;
        packet[3] = 0x00;
        //data:
        *(int *)(FramePtr + 4) = __bswap_32(defval[i]);
        //*(int *)(FramePtr + 4) = defval[i];

        for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';

        write(fd, packet, 16);
        usleep(100000);
        read(fd, packet2, 16);
        printf("Hex output \n");								// Read output state
        for(num=0;num<16;num++){printf("%02X:", packet[num]);}//{printf("%02d:", packet[num]);}
            printf("\n");
        for(num=0;num<16;num++){printf("%02X:", packet2[num]);}//{printf("%02d:", packet2[num]);}
            printf("\n");
        usleep(100000);
    }

    return 0;
}

//---------a further tweak of the above, and the one we are using as of 4/8/2019
//---------the difference is the presence of variables startER and endER - what are these?
//---------usage: program_exposure_data_tweak2 [exposure time] [number of frames]
int program_exposure_data_tweak2(void)
{
    //set transition times for CLB_IN, PRB_IN, EXT_RST,END_EXP, READ, BIN, and end of frame time
    //for frame setup
    //take integration time in clock cycles as an argument. this sets the delay between 1st transition of EXT_RSTB and 2nd transition of END_EXP

    int temp;
    tcflush(fd,TCIOFLUSH);
    char* substr;
    int expt=0;
    int end_exp_secondedge=0; //location of the second transition of END_EXP - determines end of integration - will be computed in this function
    int ext_rst_firstedge=6; //location of the first transition of EXT_RST - determines start of integration - hard-coded
    int ext_rst_secondedge=0; //location of the second transition of EXT_RST - will be computed in this function to be 1 clock cycle after end of integration
    int endt = 0; //location of the END signal - will be computed in this function to be 2 clock cycles after end of integration
    int cnt=0;
    int framecount=1;
    int startER = 2;
    int endER;
    endER = startER+1;
    /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.  In this case, looking for 3 values.
     * If number of arguments is not what is expected, it will return without action and a
     * message. Return value will be 1.
     * This basic template is used for all functions that take arguments.
     */

    if(strlen(cmd)<=7)
    {//
        free(cmd);

        cnt = 1;
        cmd = readline("exposure time [number of clock cycles]> ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        expt = atol(cmd);
        free(cmd);
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                expt = atol(substr);
            }

            if(cnt==2)
            {
                framecount = atol(substr);
            }

        }

    }

    if((cnt!=1)&&(cnt!=2))
    {
        printf("count %d \n",cnt);
        printf("usage: program_exposure_data <exposure time> \n     or program_exposure_data   --- gives you prompt. \n");
        return 1;
    }
    printf("Values set: exposure time [number of clock cycles]: %d \n", expt);

    //compute other edge locations that depend on expt:
    end_exp_secondedge = ext_rst_firstedge + expt;
    ext_rst_secondedge = end_exp_secondedge + 2;
    endt = end_exp_secondedge + 4;

    //command IDs:
    int comID[17] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,16, 17};
    //default values (1-13: # of clock cycles from initial timestamp. clock period is 10ns
    //					14: frame delay - # of clock cycles [from what to what?]
    //					15: frame count
    //int defval[15] = {1, end_exp_secondedge+1, 4, 5, 2, 3, ext_rst_firstedge, ext_rst_secondedge, 1, end_exp_secondedge, 5, 6, endt, 3, framecount};
    int defval[17] = {1, end_exp_secondedge-10, 4, 5, 2, 3, ext_rst_firstedge, ext_rst_secondedge, 1, end_exp_secondedge, 5, 6, endt, 100, framecount, startER, endER};

    printf("\
            Read 	edge1: %d	edge2: %d \n \
            CLB	edge1: %d	edge2: %d \n \
            PRB  	edge1: %d	edge2: %d \n \
            ext_rst	edge1: %d	edge2: %d \n \
            end_exp	edge1: %d	edge2: %d \n \
            bin	edge1: %d	edge2: %d \n \
            end time:		%d \n \
            frame delay:	%d \n \
            frame count:	%d \n",
            defval[0]*10,defval[1]*10,defval[2]*10,defval[3]*10,
            defval[4]*10,defval[5]*10,defval[6]*10,defval[7]*10,
            defval[8]*10,defval[9]*10,defval[10]*10,defval[11]*10,
            defval[12]*10,defval[13]*10,defval[14]);

    int i;

    for(i=0; i < 17; i++)  // modifying - changing from 15 to 17 parameters
    {
        //command:
        packet[0] = 0x01;
        packet[1] = comID[i];
        packet[2] = 0x00;
        packet[3] = 0x00;
        //data:
        *(int *)(FramePtr + 4) = __bswap_32(defval[i]);
        //*(int *)(FramePtr + 4) = defval[i];

        for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';

        write(fd, packet, 16);
        usleep(100000);
        read(fd, packet2, 16);
        printf("Hex output \n");								// Read output state
        for(num=0;num<16;num++){printf("%02X:", packet[num]);}//{printf("%02d:", packet[num]);}
            printf("\n");
        for(num=0;num<16;num++){printf("%02X:", packet2[num]);}//{printf("%02d:", packet2[num]);}
            printf("\n");
        usleep(100000);
    }

    return 0;
}

//================================================================================================
//=========================digital edge test pattern programming and control======================
//================================================================================================

//---------program digital edge test pattern - usage: setdetp [bank # 0-3] [18 bit pattern]
int set_detp(void)
{
   char str [20];
   int regnum;
   unsigned int detpData;	// 18 bits of Data to write to registers 0,1,2,3 in hex


   if(strlen(cmd)<=8)
   {
      printf("syntax error: useage: 'setdetp 1 0x312bc \n");
      return 1;
   }
   else
   {
      sscanf(cmd, "%s %d %x", str, &regnum, &detpData);
   }

   packet[0] = 0x1E;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = regnum;
   packet[5] = (detpData & 0x00ff0000) >> 16;
   packet[6] = (detpData & 0x0000ff00) >> 8;
   packet[7] = (detpData & 0x000000ff);

   for(num = 8; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(20000);
   read(fd, packet2, 16);                        // Read output state
   usleep(20000);
   for(num=0;num<16;num++){printf("%02x:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02x:", packet2[num]);}
   printf("\n");

   return 0;
}


//================================================================================================
//=========================in-pixel test pattern programming and control==========================
//================================================================================================

int atpatt_en(void)
{
    // tell ASIC to enable bank 0 to send out analog data
   tcflush(fd,TCIOFLUSH);

   int ch_aout0_en; //1 for enable bank 0, 0 for disable
   //scanf("%d", &ceb);
    char* substr;
    int cnt=0;
        /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.
     */


   if(strlen(cmd)<=10)
    {
        //
        free(cmd);
        cmd = readline("enable? ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        ch_aout0_en = atoi(cmd);
        cnt = 1;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                ch_aout0_en = atoi(substr);
            }
        }
    }

    if(cnt!=1)
    {
        printf("syntax error: useage: 'atpatt_en <value>' or 'atpatt_en' for prompt \n");
        return 1;
    }

    //printf(" %ld %ld %ld \n", Itime,Ftime,Fcount);
    //


   packet[0] = 0x20;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = ch_aout0_en;

   for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(50000);
   read(fd, packet2, 16);                        // Read output state

   for(num=0;num<16;num++){printf("%02d:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
   printf("\n");

   return 0;
}

int dtpatt_en(void)
{
        // tell ASIC to enable digital test data on outputs
   tcflush(fd,TCIOFLUSH);

   int testd;	//1 for enable test data or 0 for disable test data
   //scanf("%d", &ceb);
    char* substr;
    int cnt=0;
        /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.
     */


   if(strlen(cmd)<=10)
    {
        //
        free(cmd);
        cmd = readline("enable? ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        testd = atoi(cmd);
        cnt = 1;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                testd = atoi(substr);
            }
        }
    }

    if(cnt!=1)
    {
        printf("syntax error: useage: 'dtpatt_en <value>' or 'dtpatt_en' for prompt \n");
        return 1;
    }

    //printf(" %ld %ld %ld \n", Itime,Ftime,Fcount);
    //


   packet[0] = 0x22;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = testd;

   for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(50000);
   read(fd, packet2, 16);                        // Read output state

   for(num=0;num<16;num++){printf("%02d:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
   printf("\n");

   return 0;
}



int progTSource(void)
{
    //updated - 10/2/2017
    tcflush(fd,TCIOFLUSH);
    packet[0] = 0x09;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    for(num = 4; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(20000);
    read(fd, packet2, 16);								// Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}

//---------load a unique in-pixel test source pattern
int loadTSource(void)
{
    //pixel addresses to turn on. row 1, col 1 is the top right corner of the chip
    //defining this on the fly for now

    //uint16_t TSdata[16] = {0,0,0,0,0,0x0C20,0x0240,0x0180,0x0240,0x0C20,0,0,0,0,0,0};
    //uint16_t TSdata[16] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF3DF,0xFDBF,0xFE7F,0xFDBF,0xF3DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};

    //         making a pattern
    //			 011000010
    //			 000100100
    //			 000011000
    //			 000100100
    //			 011000010
    //
    //low bit value turns on source, high bit value turns off source

    //some different patterns:
    //ideally would make this a command line argument but hard-code for now...
    //one row on - should be row 2 from top
    //uint16_t TSdata[16] = {0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD};
    //one col on - should be col 7
    //uint16_t TSdata[16] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    //uint16_t TSdata[16] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDDDD, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    uint16_t TSdata[16] = {0x7FFF, 0x3041, 0x5DF7, 0x3049, 0x7FFF, 0xF041, 0x17F5, 0x5041, 0x1FFF, 0xF47D, 0x1541, 0xF07D, 0x1FFF, 0xF041, 0xFDD5, 0xF07F};


    int index;
    tcflush(fd,TCIOFLUSH);

    usleep(50000);

    for(index=0;index<16;index++)
    {
        packet[0] = 0x0B;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        *(int *)(FramePtr + 4) = __bswap_32(index*4);
        //*(int *)(FramePtr + 4) = (index*4);
        //packet[4] = index*4; //addresses increment by 4
        //packet[5] = 0x00;
        //packet[6] = 0x00;
        //packet[7] = 0x00;
        //packet[8] = TSdata[index];
        packet[8] = 0x00;
        packet[9] = 0x00;
        packet[10] = u_char((0xFF)&(TSdata[index])>>8);
        packet[11] = u_char((0xFF)&(TSdata[index]));

        for(num =12; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';


        write(fd, packet, 16);
        //usleep(20000);
        usleep(200000);
        read(fd, packet2, 16);								// Read output state
        printf("%d",index);
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    }
    return 0;
}

//---------turn all in-pixel test sources off - usage: turnoffts
int TurnTSOff(void)
{
    tcflush(fd,TCIOFLUSH);

    packet[ 0] = 0x11;
    packet[ 1] = 0x00;
    packet[ 2] = 0x00;
    packet[ 3] = 0x00;
    packet[ 4] = 0x01; // ENB = 1 means test source off

    for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(50000);
    read(fd, packet2, 16);                        // Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");
    return 0;
}

//---------turn all test sources on - usage: turnonts
int TurnTSOn(void)
{
    long addrs, data;
    tcflush(fd,TCIOFLUSH);

    packet[ 0] = 0x11;
    packet[ 1] = 0x00;
    packet[ 2] = 0x00;
    packet[ 3] = 0x00;
    packet[ 4] = 0x00;

    for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(50000);
    read(fd, packet2, 16);                        // Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");
    return 0;
}

//---------load smiley face in-pixel test source pattern
int loadTSmile1(void)
{
    //pixel addresses to turn on. row 1, col 1 is the top right corner of the chip
    //defining this on the fly for now

    //uint16_t TSdata[16] = {0,0,0,0,0,0x0C20,0x0240,0x0180,0x0240,0x0C20,0,0,0,0,0,0};
    //uint16_t TSdata[16] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF3DF,0xFDBF,0xFE7F,0xFDBF,0xF3DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};

    //         making a pattern
    //			 011000010
    //			 000100100
    //			 000011000
    //			 000100100
    //			 011000010
    //
    //low bit value turns on source, high bit value turns off source

    // modified load a smile like pattern - across all banks.
    uint16_t TSdata[16] = {0xFFFF,0xFFDF,0xDBE7,0xFDFB,0xF3ED,0xE7D5,0xEEED,0xCCFF,0xCCFB,0xCCFF,0xEEED,0xE7D5,0xF3ED,0xFDFB,0xDBE7,0xFFDF};



    int index;
    tcflush(fd,TCIOFLUSH);

    usleep(50000);

    for(index=0;index<16;index++)
    {
        packet[0] = 0x0B;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        *(int *)(FramePtr + 4) = __bswap_32(index*4);
        //*(int *)(FramePtr + 4) = (index*4);
        //packet[4] = index*4; //addresses increment by 4
        //packet[5] = 0x00;
        //packet[6] = 0x00;
        //packet[7] = 0x00;
        //packet[8] = TSdata[index];
        packet[8] = 0x00;
        packet[9] = 0x00;
        packet[10] = u_char((0xFF)&(TSdata[index])>>8);
        packet[11] = u_char((0xFF)&(TSdata[index]));

        for(num =12; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';


        write(fd, packet, 16);
        //usleep(20000);
        usleep(200000);
        read(fd, packet2, 16);								// Read output state
        printf("%d",index);
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    }
    return 0;
}

//---------load an in-pixel test source pattern with isolated pixels turned on - not tested as of 4/9/2019
int loadTSpixel(void)
{
    //pixel addresses to turn on. row 1, col 1 is the top right corner of the chip
    //defining this on the fly for now
    //int pixon = 2; //number of pixels to turn on
    //char rowson[2] = {1, 1};
    //char colson[2] = {1, 3};

    //uint16_t TSdata[16] = {0,0,0,0,0,0x0C20,0x0240,0x0180,0x0240,0x0C20,0,0,0,0,0,0};
    //uint16_t TSdata[16] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF3DF,0xFDBF,0xFE7F,0xFDBF,0xF3DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};

    //         making a pattern
    //			 011000010
    //			 000100100
    //			 000011000
    //			 000100100
    //			 011000010
    //
    //low gain banks<-->higher gain banks
    //low bit value turns on source, high bit value turns off source

    // modified load a smile like pattern - across all banks.
    //uint16_t TSdata[16] = {0xFFFF,0xFFDF,0xDBE7,0xFDFB,0xF3ED,0xE7D5,0xEEED,0xCCFF,0xCCFB,0xCCFF,0xEEED,0xE7D5,0xF3ED,0xFDFB,0xDBE7,0xFFDF};
    uint16_t TSdata[16] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFF7, 0xFDFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};


    int index;
    tcflush(fd,TCIOFLUSH);

    //populate the TSdata array with '1' for all sources - i.e. all off:
    //now turn on the the pixels specified by rowson, colson:
    //for(index=0;index<pixon;index++){TSdata[(rowson[index]-1)*16+(colson[index]-1)]=0x00;}

    usleep(50000);

    for(index=0;index<16;index++)
    {
        packet[0] = 0x0B;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        *(int *)(FramePtr + 4) = __bswap_32(index*4);
        //*(int *)(FramePtr + 4) = (index*4);
        //packet[4] = index*4; //addresses increment by 4
        //packet[5] = 0x00;
        //packet[6] = 0x00;
        //packet[7] = 0x00;
        //packet[8] = TSdata[index];
        packet[8] = 0x00;
        packet[9] = 0x00;
        packet[10] = u_char((0xFF)&(TSdata[index])>>8);
        packet[11] = u_char((0xFF)&(TSdata[index]));

        for(num =12; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';


        write(fd, packet, 16);
        //usleep(20000);
        usleep(200000);
        read(fd, packet2, 16);								// Read output state
        printf("%d",index);
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    }
    return 0;
}

//---------load an "X" in-pixel test source pattern
int loadTSX(void)
{
    //pixel addresses to turn on. row 1, col 1 is the top right corner of the chip
    //defining this on the fly for now
    //int pixon = 2; //number of pixels to turn on
    //char rowson[2] = {1, 1};
    //char colson[2] = {1, 3};

    //uint16_t TSdata[16] = {0,0,0,0,0,0x0C20,0x0240,0x0180,0x0240,0x0C20,0,0,0,0,0,0};
    //uint16_t TSdata[16] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF3DF,0xFDBF,0xFE7F,0xFDBF,0xF3DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};

    //         making a pattern
    //			 011000010
    //			 000100100
    //			 000011000
    //			 000100100
    //			 011000010
    //
    //low gain banks<-->higher gain banks
    //low bit value turns on source, high bit value turns off source

    // modified load an X pattern
    uint16_t TSdata[16] = {0x7FFE, 0xBFFD, 0xDFFB, 0xEFF7, 0xF7EF, 0xFBDF, 0xFDBF, 0xFE7F, 0xFE7F, 0xFDBF, 0xFBDF, 0xF7EF, 0xEFF7, 0xDFFB, 0xBFFD, 0x7FFE};


    int index;
    tcflush(fd,TCIOFLUSH);

    //populate the TSdata array with '1' for all sources - i.e. all off:
    //now turn on the the pixels specified by rowson, colson:
    //for(index=0;index<pixon;index++){TSdata[(rowson[index]-1)*16+(colson[index]-1)]=0x00;}

    usleep(50000);

    for(index=0;index<16;index++)
    {
        packet[0] = 0x0B;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        *(int *)(FramePtr + 4) = __bswap_32(index*4);
        //*(int *)(FramePtr + 4) = (index*4);
        //packet[4] = index*4; //addresses increment by 4
        //packet[5] = 0x00;
        //packet[6] = 0x00;
        //packet[7] = 0x00;
        //packet[8] = TSdata[index];
        packet[8] = 0x00;
        packet[9] = 0x00;
        packet[10] = u_char((0xFF)&(TSdata[index])>>8);
        packet[11] = u_char((0xFF)&(TSdata[index]));

        for(num =12; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';


        write(fd, packet, 16);
        //usleep(20000);
        usleep(200000);
        read(fd, packet2, 16);								// Read output state
        printf("%d",index);
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    }
    return 0;
}

//---------a test pattern programming loop someone wrote
int loadTPixPat1(void)
{
    // some isolated groups of 1,2, and 3 pixels. So I can look at specific beavhiours of cross tak in isolation

    uint16_t TSdata[16] = {0xFFFF, 0xFFFF, 0xBBBB, 0xFFFF, 0xFFFF, 0xFFFF, 0xBBBB, 0xBBBB, 0xFFFF, 0xFFFF, 0xFFFF, 0xBBBB, 0xBBBB, 0xBBBB, 0xFFFF, 0xFFFF};


    int index;
    tcflush(fd,TCIOFLUSH);

    //populate the TSdata array with '1' for all sources - i.e. all off:
    //now turn on the the pixels specified by rowson, colson:
    //for(index=0;index<pixon;index++){TSdata[(rowson[index]-1)*16+(colson[index]-1)]=0x00;}

    usleep(50000);

    for(index=0;index<16;index++)
    {
        packet[0] = 0x0B;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        *(int *)(FramePtr + 4) = __bswap_32(index*4);
        //*(int *)(FramePtr + 4) = (index*4);
        //packet[4] = index*4; //addresses increment by 4
        //packet[5] = 0x00;
        //packet[6] = 0x00;
        //packet[7] = 0x00;
        //packet[8] = TSdata[index];
        packet[8] = 0x00;
        packet[9] = 0x00;
        packet[10] = u_char((0xFF)&(TSdata[index])>>8);
        packet[11] = u_char((0xFF)&(TSdata[index]));

        for(num =12; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';


        write(fd, packet, 16);
        //usleep(20000);
        usleep(200000);
        read(fd, packet2, 16);								// Read output state
        printf("%d",index);
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    }
    return 0;
}

//---------load a striped test source pattern
int loadTStripe(void)
{
    //KSS 1/14/2019 for CU-APS-PAD Sub1
    //program test sources in a striped pattern
    //pixel addresses to turn on. row 1, col 1 is the top right corner of the chip
    //low bit value turns on source, high bit value turns off source

    uint16_t TSdata[16] = {0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA};

    int index;
    tcflush(fd,TCIOFLUSH);

    usleep(50000);

    for(index=0;index<16;index++)
    {
        packet[0] = 0x0B;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        *(int *)(FramePtr + 4) = __bswap_32(index*4);
        //*(int *)(FramePtr + 4) = (index*4);
        //packet[4] = index*4; //addresses increment by 4
        //packet[5] = 0x00;
        //packet[6] = 0x00;
        //packet[7] = 0x00;
        //packet[8] = TSdata[index];
        packet[8] = 0x00;
        packet[9] = 0x00;
        packet[10] = u_char((0xFF)&(TSdata[index])>>8);
        packet[11] = u_char((0xFF)&(TSdata[index]));

        for(num =12; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';


        write(fd, packet, 16);
        //usleep(20000);
        usleep(200000);
        read(fd, packet2, 16);								// Read output state
        printf("%d",index);
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    }
    return 0;
}

//================================================================================================
//=========================Bank 0 test pattern programming and control============================
//================================================================================================

//---------program Bank 0 test pattern
int set_bank0tp(void)
{
   char str [20];
   int regnum;
   unsigned int detpData;	// 32 bits of Data to write to detp registers 0,1,2,3,4,5,6,7 in hex

   if(strlen(cmd)<=11)
   {
      printf("syntax error: useage: 'setbank0tp 1 0x12345678 \n");
      return 1;
   }
   else
   {
      sscanf(cmd, "%s %d %x", str, &regnum, &detpData);
   }

   packet[0] = 0x20;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = regnum;
   packet[5] = (detpData & 0xff000000) >> 24;
   packet[6] = (detpData & 0x00ff0000) >> 16;
   packet[7] = (detpData & 0x0000ff00) >> 8;
   packet[8] = (detpData & 0x000000ff);

   for(num = 9; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(20000);
   read(fd, packet2, 16);                        // Read output state
   usleep(20000);
   for(num=0;num<16;num++){printf("%02x:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02x:", packet2[num]);}
   printf("\n");

   return 0;
}

int set_bank0pixel(void)	// setbank0pixel bank pixel value	Easier way to write into bank 0 data?
                                // uBlaze needs to figure out what bits to change and remember old values
{
   char str [20];		// just the command
   int bank;			// 1..4
   int pixel;			// 1..16
   int pixelvalue;		// 0..0xF

   if(strlen(cmd)<=11)
   {
      printf("syntax error: useage: 'setbank0tp 1 0x12345678 \n");
      return 1;
   }
   else
   {
      sscanf(cmd, "%s %d %d %x", str, &bank, &pixel, &pixelvalue);
   }

   packet[0] = 0x21;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = bank;
   packet[5] = pixel;
   packet[6] = pixelvalue;

   for(num = 7; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(20000);
   read(fd, packet2, 16);                        // Read output state
   usleep(20000);
   for(num=0;num<16;num++){printf("%02x:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02x:", packet2[num]);}
   printf("\n");

   return 0;
}

//---------mux bank0 analog output into all 4 analog banks in FPGA
int bank0tp_en(void)
{
    // Tell the FPGA to use ADC bank 0 output as the analog data for all 4 adc outputs
   tcflush(fd,TCIOFLUSH);

   int bank0tp; //1 for enable, 0 for disable
   //scanf("%d", &ceb);
    char* substr;
    int cnt=0;
    //-------begin command parser-----------------------------------------------------------------
        /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.
     */


   if(strlen(cmd)<=11)
    {
        //
        free(cmd);
        cmd = readline("enable? ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        bank0tp = atoi(cmd);
        cnt = 1;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                bank0tp = atoi(substr);
            }
        }
    }

    if(cnt!=1)
    {
        printf("syntax error: useage: 'bank0tp_en <value>' or 'bank0tp_en' for prompt \n");
        return 1;
    }
    //-------end command parser-------------------------------------------------------------------

   packet[0] = 0x20;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = bank0tp;

   for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(50000);
   read(fd, packet2, 16);                        // Read output state

   for(num=0;num<16;num++){printf("%02d:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
   printf("\n");

   return 0;
}

//================================================================================================
//======================================Motor and shutter control=================================
//================================================================================================

//4/1 KSS add functions for motor control:
//======================================Initialize the motor=================================

int initialize_motor(void)
{
    motor_fd1 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (motor_fd1 == -1)
    {
        printf("Unable to open /dev/ttyUSB0");
        motor_fd1 = open("/dev/ttyS4", O_RDWR | O_NOCTTY | O_NDELAY);
        if (motor_fd1 == -1)
        {
            printf("Unable to open /dev/ttyS4 or /dev/ttyS0\n");
            return(-1);
        } else
        {
            printf("/dev/ttyS4 ");
        }
    } else
    {
        printf("/dev/ttyUSB0 ");
    }
    fcntl(motor_fd1, F_SETFL, 0);
    struct termios options;
    // Get the current options for the port
    tcgetattr(motor_fd1, &options);
    // Set the baud rates to 19200
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    // Enable the receiver and set local mode
    options.c_cflag |= (CLOCAL | CREAD);
    // Set - No parity (8N1)
    options.c_cflag &= ~PARENB;					/* No parity */
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;        			/* Mask the character size bits */
    options.c_cflag |= CS8;						/* Select 8 data bits */
    // raw input - characters are passed through exactly as they are received
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // Set the new options for the port
    tcsetattr(motor_fd1, TCSANOW, &options);
    write(motor_fd1, "1MO\r", 4);			// Turn Motor #1 ON
    write(motor_fd1, "2MO\r", 4);			// Turn Motor #2 ON
    write(motor_fd1, "3MO\r", 4);			// Turn Motor #3 ON
    write(motor_fd1, "BO 1H\r", 6);
    write(motor_fd1, "BO?\r", 4);

    char strng[200];
    char strng2[200];
    usleep(20000);
    read(motor_fd1, strng2, 200);
    printf(" I/O check %s\n", strng2);
    write(motor_fd1, "SB 0\r", 5);
    write(motor_fd1, "SB?\r", 4);
    usleep(20000);
    read(motor_fd1, strng2, 200);
    printf("Default Bit value check %s\n", strng2);

    //write(motor_fd1, strng, strlen(strng));

    printf("motor comm initialized\n");
    motor_initialized=1;

    return (motor_fd1);
}

int set_shutter(void)
{
    if(!motor_initialized) initialize_motor();
    //motor_fd1 = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    //int motor_fd1 = initialize_motor();
    int cnt=0;
    int bit_val;
    char* substr;
    //printf(" before breaking%s\n",cmd);
    strtok(cmd," ,");
    //printf("%s\n",strtok(cmd," ,"));
    while((substr=strtok(NULL," ,"))!=NULL)
    {
            cnt++;
            printf("%s\n",substr);

            if(cnt==1)
            {
                bit_val = atol(substr);
            }

    }
    printf("bit value from command %d\n",bit_val);
    char strng[200];
    char strng2[200];
    if(bit_val)
    {
        write(motor_fd1, "SB 1\r", 5);
        write(motor_fd1, "SB?\r", 4);
        usleep(20000);
        read(motor_fd1, strng2, 200);
        printf("Shutter is open %s\n", strng2);
    }
    else
    {
        write(motor_fd1, "SB 0\r", 5);
        write(motor_fd1, "SB?\r", 4);
        usleep(20000);
        read(motor_fd1, strng2, 200);
        printf("Shutter is closed %s\n", strng2);
    }
    return(bit_val);
}
//==============================Getting the motor velocity===================
float get_motor_vel(void) //int channel
{
    int cnt=0;
    int channel;
    char* substr;
    //printf(" before breaking%s\n",cmd);
    strtok(cmd," ,");
    //printf("%s\n",strtok(cmd," ,"));
    while((substr=strtok(NULL," ,"))!=NULL)
    {
            cnt++;
            printf("%s\n",substr);

            if(cnt==1)
            {
                channel = atol(substr);
            }

    }
    printf("channel %d\n",channel);
    //get_channel();
    //int channel = get_channel();
    char strng[200];
    char strng2[200];
    float val;
    if(!motor_initialized) initialize_motor();
    sprintf(strng,"%dVA?\r",channel);
    tcflush(motor_fd1,TCIOFLUSH);
    write(motor_fd1, strng, strlen(strng));
    usleep(20000);
    read(motor_fd1, strng2, 200);
    val=atof(strng2);
    printf("Motor #%d velocity at %f units/s \n",channel,val);
    return (val);
}

//==============================Setting Motor velocity==========================

void set_motor_vel(void)
{
    int cnt=0;
    int channel;
    float val;
    char* substr;
    //printf(" before breaking%s\n",cmd);
    strtok(cmd," ,");
    //printf("%s\n",strtok(cmd," ,"));
    while((substr=strtok(NULL," ,"))!=NULL)
    {
            cnt++;
            printf("%s\n",substr);

            if(cnt==1)
            {
                channel = atol(substr);
            }
            if(cnt==2)
            {
                val = atof(substr);
                //cmd.append(" ",substr);
            }
    }

    printf("channel %d\n",channel);
    printf("value %f\n",val);
    char strng[200];
    if(!motor_initialized) initialize_motor();
    sprintf(strng,"%iVA%f\r",channel,val);
    write(motor_fd1, strng, strlen(strng));
    printf("Attempting to set motor#%i velocity to %f\n",channel,val);
    return;
}

//=========get motor pos============================================

float get_motor_pos(void)
{

    int cnt=0;
    int channel;
    char* substr;
    //printf(" before breaking%s\n",cmd);
    strtok(cmd," ,");
    //printf("%s\n",strtok(cmd," ,"));
    while((substr=strtok(NULL," ,"))!=NULL)
    {
            cnt++;
            printf("%s\n",substr);

            if(cnt==1)
            {
                channel = atol(substr);
            }
    }
    char strng[200];
    char strng2[200];
    float val;
    if(!motor_initialized) initialize_motor();
    sprintf(strng,"%dTP\r",channel);
    tcflush(motor_fd1,TCIOFLUSH);
//	   read(fd, strng2, 200);
    write(motor_fd1, strng, strlen(strng));
    usleep(20000);
    read(motor_fd1, strng2, 200);
       //printf("\n");
    val=atof(strng2);
       //printf("Motor output %d \n",strlen(strng2));
    printf("Motor #%d at position %f mm \n",channel,val);
    return (val);
}

//======================================set motor position=========================================

void set_motor_pos(void)
{
    int cnt=0;
    int channel;
    float val;
    char* substr;
    int mode;
    //printf(" before breaking%s\n",cmd);
    strtok(cmd," ,");
    //printf("%s\n",strtok(cmd," ,"));
    while((substr=strtok(NULL," ,"))!=NULL)
    {
            cnt++;
            printf("%s\n",substr);

            if(cnt==1)
            {
                channel = atol(substr);
            }
            if(cnt==2)
            {
                val = atof(substr);
                //cmd.append(" ",substr);
            }
            if(cnt==3)
            {
                mode = atof(substr);
                //cmd.append(" ",substr);
            }
    }
    printf("channel %d\n",channel);
    printf("value %f\n",val);
    printf("value %d\n",mode);
    char strng[200];
    if(!motor_initialized) initialize_motor();
    if(mode)
    {
        sprintf(strng,"%dPR%f\r",channel,val);
        write(motor_fd1, strng, strlen(strng));
        printf("Move motor #%d %f mm from current position \n",channel,val);
    }
    else
    {
       sprintf(strng,"%dPA%f\r",channel,val);
       write(motor_fd1, strng, strlen(strng));
       printf("Move motor#%d to absolute position %f \n",channel,val);
    }
    return;
}

//================================================================================================
//====================================Custom exposure loops=======================================
//================================================================================================
//custloop, custloop2, custloop3, custloop4 - all sweeping integration time in different
// increments.

int	custloop(void)
{
    int i;
    //free(cmd); // changing order of free commands to prevent error upon return to command loop.
    for(i=0;i<101;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak %d 100", 2000+i*100);
        printf("program_exposure_data_tweak %d 100 \n",i*100);
        program_exposure_data_tweak();
        usleep(50000);
        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(5000000);
        //free(cmd);
    }

    for(i=1;i<2001;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak %d 100",i*200+12000);
        printf("program_exposure_data_tweak %d 100\n",i*200+10000);
        program_exposure_data_tweak();
        usleep(50000);
        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(6000000);
    }

}
int	custloop2(void)
{
    int i;
    //free(cmd);
    for(i=0;i<4001;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak %d 100", 2000+i*100);
        printf("program_exposure_data_tweak %d 100 \n",i*100);
        program_exposure_data_tweak();
        usleep(50000);
        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(5000000);
        //free(cmd);
    }


}

int	custloop3(void)
{
    int i;
    //free(cmd);
    for(i=0;i<10000;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak2 %d 100", 2000+i*75);
        printf("program_exposure_data_tweak %d 100 \n",2000+i*75);
        program_exposure_data_tweak();
        usleep(50000);
        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(5000000);
        //free(cmd);  double free causing errors at end of loops when going back to command interface. switch order in loop.
    }


}

int	custloop4(void)
{
    int i;
    //free(cmd);
    for(i=1;i<1000;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak2 %d 100", 2000+6000*75+i*200);
        printf("program_exposure_data_tweak %d 100 \n",2000+6000*75+i*200);
        program_exposure_data_tweak();
        usleep(50000);
        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(5000000);
        //free(cmd);  double free causing errors at end of loops when going back to command interface. switch order in loop.
    }


}


int	custloop5(void)
{
    int i;
    //free(cmd); // changing order of free commands to prevent error upon return to command loop.
    for(i=0;i<100;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak %d 100", 10+10*i);
        printf("program_exposure_data_tweak %d 100 \n",10+10*i);
        program_exposure_data_tweak();
        usleep(50000);
        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(500000);
        //free(cmd);
    }

    for(i=0;i<500;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak %d 100",1000+i*100);
        printf("program_exposure_data_tweak %d 100\n",1000+i*100);
        program_exposure_data_tweak();
        usleep(50000);
        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(500000);
    }

}

//--------- another custom exposure loop
int	smileloop(void)
{
    int i;
    //free(cmd);
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 10000 1000");
    program_exposure_data_tweak2();
    usleep(50000);

    free(cmd);
    asprintf(&cmd,"loadtsmile1");
    loadTSmile1();
    usleep(100000);


    free(cmd);
    asprintf(&cmd,"progts");
    progTSource();
    usleep(100000);


    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);



    for(i=0;i<18;i++)
    {
        free(cmd);
        asprintf(&cmd,"bias 13 %d",3700-100*i);
        set_bias();
        usleep(40000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        //free(cmd);
        //asprintf(&cmd,"program_exposure_data_tweak2 %d 100", 2000+6000*75+i*200);
        //printf("program_exposure_data_tweak %d 100 \n",2000+6000*75+i*200);
        //program_exposure_data_tweak();
        //usleep(50000);
        //free(cmd);
        //asprintf(&cmd,"taken");
        //taken();
        //usleep(5000000);
        //free(cmd);  double free causing errors at end of loops when going back to command interface. switch order in loop.
    }


}

//---------another experimental loop
int	smileloop_mini(void)
{
    int i;
    //free(cmd);
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 10000 1000");
    program_exposure_data_tweak2();
    usleep(10000000);

    // comment out the test source programming until it is debugged in the new bitfiles.

    free(cmd);
    asprintf(&cmd,"loadtsmile1");
    loadTSmile1();
    usleep(5000000);


    free(cmd);
    asprintf(&cmd,"progts");
    progTSource();
    usleep(1000000);


    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);



    for(i=0;i<7;i++)
    {
        free(cmd);
        asprintf(&cmd,"bias 13 %d",3500-200*i);
        set_bias();
        usleep(40000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        //free(cmd);
        //asprintf(&cmd,"program_exposure_data_tweak2 %d 100", 2000+6000*75+i*200);
        //printf("program_exposure_data_tweak %d 100 \n",2000+6000*75+i*200);
        //program_exposure_data_tweak();
        //usleep(50000);
        //free(cmd);
        //asprintf(&cmd,"taken");
        //taken();
        //usleep(5000000);
        //free(cmd);  double free causing errors at end of loops when going back to command interface. switch order in loop.
    }

    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);


}

int loadTSarb(void)
{
    //updating for CU-APS-PAD Sub1 - function for dumping in an arbitrary test pattern
    //one row on - should be row 2 from top
    //uint16_t TSdata[16] = {0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD, 0xFFFD};
    //one col on - should be col 7
    //uint16_t TSdata[16] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    uint16_t TSdata[16] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDDDD, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};


    int index;
    tcflush(fd,TCIOFLUSH);

    usleep(50000);

    for(index=0;index<16;index++)
    {
        packet[0] = 0x0B;
        packet[1] = 0x00;
        packet[2] = 0x00;
        packet[3] = 0x00;
        *(int *)(FramePtr + 4) = __bswap_32(index*4);
        packet[8] = 0x00;
        packet[9] = 0x00;
        packet[10] = u_char((0xFF)&(TSdata[index])>>8);
        packet[11] = u_char((0xFF)&(TSdata[index]));

        for(num =12; num < 16; num++ ){ packet[num] = 0x00; }
        packet[15] = '*';


        write(fd, packet, 16);
        usleep(200000);
        read(fd, packet2, 16);								// Read output state
        printf("%d",index);
        for(num=0;num<16;num++){printf("%02d:", packet[num]);}
        printf("\n");
        for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
        printf("\n");
    }
    return 0;
}

//---------another experimental loop
int	Xloop_mini(void)
{
    int i;
    //free(cmd);
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 10000 1000");
    program_exposure_data_tweak2();
    usleep(10000000);

    // comment out the test source programming until it is debugged in the new bitfiles.

    free(cmd);
    asprintf(&cmd,"loadtsx");
    loadTSX();
    usleep(5000000);


    free(cmd);
    asprintf(&cmd,"progts");
    progTSource();
    usleep(1000000);


    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);



    for(i=0;i<7;i++)
    {
        free(cmd);
        asprintf(&cmd,"bias 13 %d",3500-200*i);
        set_bias();
        usleep(40000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        //free(cmd);
        //asprintf(&cmd,"program_exposure_data_tweak2 %d 100", 2000+6000*75+i*200);
        //printf("program_exposure_data_tweak %d 100 \n",2000+6000*75+i*200);
        //program_exposure_data_tweak();
        //usleep(50000);
        //free(cmd);
        //asprintf(&cmd,"taken");
        //taken();
        //usleep(5000000);
        //free(cmd);  double free causing errors at end of loops when going back to command interface. switch order in loop.
    }

    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);


}

//---------another experimental loop
int	Xloop_maxi(void)
{
    int i;
    //free(cmd);
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 10000 1000");
    program_exposure_data_tweak2();
    usleep(10000000);

    // comment out the test source programming until it is debugged in the new bitfiles.

    free(cmd);
    asprintf(&cmd,"loadtsx");
    loadTSX();
    usleep(5000000);


    free(cmd);
    asprintf(&cmd,"progts");
    progTSource();
    usleep(1000000);


    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);



    for(i=0;i<25;i++)
    {
        free(cmd);
        asprintf(&cmd,"bias 13 %d",4000-100*i);
        set_bias();
        usleep(40000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        //free(cmd);
        //asprintf(&cmd,"program_exposure_data_tweak2 %d 100", 2000+6000*75+i*200);
        //printf("program_exposure_data_tweak %d 100 \n",2000+6000*75+i*200);
        //program_exposure_data_tweak();
        //usleep(50000);
        //free(cmd);
        //asprintf(&cmd,"taken");
        //taken();
        //usleep(5000000);
        //free(cmd);  double free causing errors at end of loops when going back to command interface. switch order in loop.
    }

    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);


}


//---------another experimental loop
int	Xloop_itime(void)
{
    int cnt =0;
    char* substr;
    int b13 =4095;


    if(strlen(cmd)<=10)
    {//
        //free(cmd);
        //printf("ana_phase <phase code> \n     000 -> 045 degree phase\n     001 -> 060 degree phase\n     010 -> 075 degree phase\n     011 -> 090 degree phase\n     100 -> 105 degree phase\n     101 -> 120 degree phase\n ");
        b13 = 3100;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                b13 = atol(substr);
            }

        }

    }

    if(cnt == 0)
    {
        printf("using the default bias 13 setting of 3100\n");
    }
    else if(cnt == 1)
    {
        printf("using the bias 13 setting of %d \n",b13);
    }
    else
    {
        printf("use:  xloop_itime <bias 13 setting> \n command cycles through integration times with an x-pattern");
        return 1;
    }



    int i;
    //free(cmd);
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 10000 1000");
    program_exposure_data_tweak2();
    usleep(10000000);

    // comment out the test source programming until it is debugged in the new bitfiles.

    free(cmd);
    asprintf(&cmd,"loadtsx");
    loadTSX();
    usleep(5000000);


    free(cmd);
    asprintf(&cmd,"progts");
    progTSource();
    usleep(1000000);


    //free(cmd);
    //asprintf(&cmd,"bias 13 4095");
    //set_bias();
    //usleep(40000);

    //free(cmd);
    //asprintf(&cmd,"taken");
    //taken();
    //usleep(35000000);

    //free(cmd);
    //asprintf(&cmd,"bias 13 %d",b13);
    //set_bias();
    //usleep(100000);


    for(i=0;i<31;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak2 %d 1000",10000+i*500);
        program_exposure_data_tweak2();
        usleep(10000000);

        free(cmd);
        asprintf(&cmd,"bias 13 4095");
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        free(cmd);
        asprintf(&cmd,"bias 13 %d",b13);
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);
    }

    for(i=0;i<10;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak2 %d 1000",30000+i*5000);
        program_exposure_data_tweak2();
        usleep(10000000);

        free(cmd);
        asprintf(&cmd,"bias 13 4095");
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        free(cmd);
        asprintf(&cmd,"bias 13 %d",b13);
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);
    }


    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);


}

//---------another experimental loop
int	Xloop_pixpat_itime(void)
{
    int cnt =0;
    char* substr;
    int b13 =4095;


    if(strlen(cmd)<=10)
    {//
        //free(cmd);
        //printf("ana_phase <phase code> \n     000 -> 045 degree phase\n     001 -> 060 degree phase\n     010 -> 075 degree phase\n     011 -> 090 degree phase\n     100 -> 105 degree phase\n     101 -> 120 degree phase\n ");
        b13 = 3100;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                b13 = atol(substr);
            }

        }

    }

    if(cnt == 0)
    {
        printf("using the default bias 13 setting of 3100\n");
    }
    else if(cnt == 1)
    {
        printf("using the bias 13 setting of %d \n",b13);
    }
    else
    {
        printf("use:  xloop_itime <bias 13 setting> \n command cycles through integration times with an x-pattern");
        return 1;
    }



    int i;
    //free(cmd);
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 10000 1000");
    program_exposure_data_tweak2();
    usleep(10000000);

    // comment out the test source programming until it is debugged in the new bitfiles.

    free(cmd);
    asprintf(&cmd,"loadtspixpat1");
    loadTPixPat1();
    usleep(5000000);


    free(cmd);
    asprintf(&cmd,"progts");
    progTSource();
    usleep(1000000);


    //free(cmd);
    //asprintf(&cmd,"bias 13 4095");
    //set_bias();
    //usleep(40000);

    //free(cmd);
    //asprintf(&cmd,"taken");
    //taken();
    //usleep(35000000);

    //free(cmd);
    //asprintf(&cmd,"bias 13 %d",b13);
    //set_bias();
    //usleep(100000);


    for(i=0;i<31;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak2 %d 1000",10000+i*500);
        program_exposure_data_tweak2();
        usleep(10000000);

        free(cmd);
        asprintf(&cmd,"bias 13 4095");
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        free(cmd);
        asprintf(&cmd,"bias 13 %d",b13);
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);
    }

    for(i=0;i<10;i++)
    {
        free(cmd);
        asprintf(&cmd,"program_exposure_data_tweak2 %d 1000",30000+i*5000);
        program_exposure_data_tweak2();
        usleep(10000000);

        free(cmd);
        asprintf(&cmd,"bias 13 4095");
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);

        free(cmd);
        asprintf(&cmd,"bias 13 %d",b13);
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);
    }


    free(cmd);
    asprintf(&cmd,"bias 13 4095");
    set_bias();
    usleep(40000);

    free(cmd);
    asprintf(&cmd,"taken");
    taken();
    usleep(35000000);

    return 0;

}

//---------another experimental loop
int	Xloop_pixpat_itime_loop(void)
{
    int j;

    for(int i=0;i<15;i++)
    {
        free(cmd);
        asprintf(&cmd,"xloop_pixpat_itime %d",2500-i*100);
        j=Xloop_pixpat_itime();
        usleep(10000000);
    }
    return j;
}

//---------another experimental loop
int isweep(void)
{
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 %d 1000",10000);
    program_exposure_data_tweak2();
    usleep(10000000);

    for(int i =0; i<15;i++)
    {
        free(cmd);
        asprintf(&cmd,"bias 13 %d",2500-i*100);
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);
    }

    return 0;
}

//---------another experimental loop
int isweep2(void)
{
    free(cmd);
    asprintf(&cmd,"program_exposure_data_tweak2 %d 1000",10000);
    program_exposure_data_tweak2();
    usleep(10000000);

    for(int i =0; i<24;i++)
    {
        free(cmd);
        asprintf(&cmd,"bias 13 %d",3800-i*100);
        set_bias();
        usleep(100000);

        free(cmd);
        asprintf(&cmd,"taken");
        taken();
        usleep(35000000);
    }

    return 0;
}

//---------another experimental loop
int	Xloop_pixpat_itime_loop2(void)
{
    int j;

    for(int i=0;i<10;i++)
    {
        free(cmd);
        asprintf(&cmd,"xloop_pixpat_itime %d",3500-i*100);
        j=Xloop_pixpat_itime();
        usleep(10000000);
    }
    return j;
}

//================================================================================================
//=========================Deprecated - should be deleted=========================================
//================================================================================================

//---------DEPRECATED - toggle electron vs. hole collecting mode. For CU-APS-PAD Sub1 this is handled through set_ccr.
int ceb_en(void)
{
    //set CEB. 1 = collect holes, 0 = collect electrons
   tcflush(fd,TCIOFLUSH);

   int ceb; //1 for enable, 0 for disable
   //scanf("%d", &ceb);
    char* substr;
    int cnt=0;
    //-------begin command parser-----------------------------------------------------------------
    /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.  In this case, looking for 3 values.
     * If number of arguments is not what is expected, it will return without action and a
     * message. Return value will be 1.
     * This basic template is used for all functions that take arguments.
     */


   if(strlen(cmd)<=7)
    {
        //
        free(cmd);
        cmd = readline("enable? ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        ceb = atoi(cmd);
        cnt = 1;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                ceb = atoi(substr);
            }
        }
    }

    if(cnt!=1)
    {
        printf("syntax error: useage: 'ceb_en <value>' or 'ceb_en' for prompt \n");
        return 1;
    }
   //-------end command parser-------------------------------------------------------------------

   packet[0] = 0x05;
   packet[1] = 0x00;
   packet[2] = 0x00;
   packet[3] = 0x00;
   packet[4] = ceb;

   for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
   packet[15] = '*';

   write(fd, packet, 16);
   usleep(50000);
   read(fd, packet2, 16);                        // Read output state

   for(num=0;num<16;num++){printf("%02d:", packet[num]);}
   printf("\n");
   for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
   printf("\n");

   return 0;
}

//---------deprecated?
int test_bias(void)
{
    //HDR-PAD only - not yet updated - not sure what this function does either
    int d1;
    long d2;
    tcflush(fd,TCIOFLUSH);
    char* substr;
    int cnt=0;

    //scanf("%d %ld", &d1, &d2);
    /*
     * New command parser - either prompts for values, or, if string size is longer, looks for
     * values on the command line.  In this case, looking for 3 values.
     * If number of arguments is not what is expected, it will return without action and a
     * message. Return value will be 1.
     * This basic template is used for all functions that take arguments.
     */

    //
    if(strlen(cmd)<=7)
    {
        free(cmd);
        cmd = readline("> ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        d1 = atoi(cmd);
        free(cmd);

        cmd = readline("> ");
        add_history(cmd);
        fprintf(fp,"%s ",cmd);
        d2 = atol(cmd);

        cnt=2;
    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                d1 = atoi(substr);
            }

            if(cnt==2)
            {
                d2 = atol(substr);
            }
        }
    }

    if(cnt!=2)
    {
        printf("syntax error: useage: 'bbias <bias> <value>' or 'bbias' for prompt \n");
        return 1;
    }

    //


    packet[0] = 40;
    packet[1] = d1;
    packet[2] = 0x00;
    packet[3] = 0x00;
    *(int *)(FramePtr +  4) = __bswap_32(d2);

    for(num = 8; num < 100; num++ ){ packet[num] = 0x00; }
    packet[29] = '*';

    write(fd, packet, 30);
    usleep(20000);
    read(fd, packet2, 30);                                // Read output state
    for(num=0;num<30;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<30;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}

//---------clear all pixel counters - deprecated?
int ClearCnt(void)
{
    //clear pixel counters
    tcflush(fd,TCIOFLUSH);
    //command:
    packet[0] = 0x08;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    for(num = 4; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);
    usleep(20000);
    read(fd, packet2, 16);								// Read output state
    for(num=0;num<16;num++){printf("%02d:", packet[num]);}
    printf("\n");
    for(num=0;num<16;num++){printf("%02d:", packet2[num]);}
    printf("\n");

    return 0;
}

//--------- what is this? - microblaze code for corresponding case in commands.c says "set ADC clock select"
int	ana_phase(void)
{
    long	phcode;
    int cnt=0;
    int num;
    char* substr;

    if(strlen(cmd)<=10)
    {//
        //free(cmd);
        //printf("ana_phase <phase code> \n     000 -> 045 degree phase\n     001 -> 060 degree phase\n     010 -> 075 degree phase\n     011 -> 090 degree phase\n     100 -> 105 degree phase\n     101 -> 120 degree phase\n ");

    }
    else
    {
        strtok(cmd," ,");
        while((substr=strtok(NULL," ,"))!=NULL)
        {
            cnt++;
            if(cnt==1)
            {
                phcode = atol(substr);
            }

        }

    }

    if(cnt!=1)
    {
        printf("ana_phase <phase code> \n     000 -> 045 degree phase\n     001 -> 060 degree phase\n     010 -> 075 degree phase\n     011 -> 090 degree phase\n     100 -> 105 degree phase\n     101 -> 120 degree phase\n ");
        return 1;
    }

    packet[0] = 0x0E;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    packet[4] = (u_char)(phcode & 0x07);

    for(num = 5; num < 16; num++ ){ packet[num] = 0x00; }
    packet[15] = '*';

    write(fd, packet, 16);

    for(num=0;num<16;num++){printf("%02X:", packet[num]);}//{printf("%02d:", packet[num]);}
            printf("\n");


}


void CUAPStcpThread(void)
{
    //QTcpServer *tcpServer;
    //tcpServer = new QTcpServer(tcpServer);

    int cmdStat;

    int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // defining type of port/socket
    if (sockfd < 0)
    {
        printf("error opening socket");
    }
    bzero((char *) &serv_addr, sizeof(serv_addr)); //zeroing out port buffer.
    portno = 66623; //defining port number
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno); //byte ordering

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) //trying to bind to server socket
    {
        printf("Ouch - that didn't work. Unable to bind to server socket \n");
    }

    while(1)
    {
        listen(sockfd,5);
        clilen = sizeof(cli_addr);
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0)
        {
            printf("ERROR socket on accept \n");
        }
        else
        {
            printf("We have a connection! \n");
        }
        bzero(buffer,256);
        while(1)
        {
           bzero(buffer,256);
           n = read(newsockfd,buffer,255);

           cmd = new char[255]; // OK - need to copy the buffer string to the globally defined cmd.
           //cmd = buffer; //but I don't want to pass the pointer. Just the value.
           strcpy(cmd,buffer);

           if(n < 0)
           {
                printf("ERROR reading from socket\n");
           }

           cmdStat=commandParser(buffer);
           if(cmdStat == 0)
           {
                printf("Command sent:  %s \n",buffer);
           }
           else if(cmdStat == -1)
           {
               printf("The command \" %s \" confuses me.\n", buffer);
           }
           else if(cmdStat == -2)
           {
               printf(" exit or quit received.");
               close(newsockfd);
               break;
           }

        }
    }

}


int commandParser(char* buffer)
{

    if	   (strncasecmp(buffer,"ttyinit",7)==0)		{ initialize_tty(); }
    else if (strncasecmp(buffer,"exit",4)==0)			{ return -2; }
    else if (strncasecmp(buffer,"quit",4)==0)			{ return -2; }
    else if (strncasecmp(buffer,"//",2)==0) {printf("comment added to history \n"); }

    //--------- general low-level control:
    else if (strncasecmp(buffer,"bias",4)==0)			{ set_bias(); }
    else if (strncasecmp(buffer,"adcspi",6)==0)		{ adc_spi(); }
    else if (strncasecmp(buffer,"setccr",6)==0)		{ set_ccr(); }
    else if (strncasecmp(buffer,"default_bias",12)==0)	{ set_default_biases(); }
    else if (strncasecmp(buffer,"lvds_en",7)==0)	{ lvds_en();		}
    else if (strncasecmp(buffer,"initialize_pixel_th",19)==0)	{ initialize_pixel_th(); }
    else if (strncasecmp(buffer,"write",5)==0)	{ write_uBReg(); }
    else if (strncasecmp(buffer,"read",4)==0)	{ read_uBReg(); }

    //--------- exposure control:
    else if (strncasecmp(buffer,"setexp",6)==0)		{set_exp(); }
    else if (strncasecmp(buffer,"taken",5)==0)			{ taken(); }
    else if (strncasecmp(buffer,"von",3)==0)			{ von(); }
    else if (strncasecmp(buffer,"voff",4)==0)			{ voff(); }
    else if (strncasecmp(buffer,"program_exposure_data_tweak2",28)==0) { program_exposure_data_tweak2(); }
    else if (strncasecmp(buffer,"program_exposure_data_tweak",27)==0) { program_exposure_data_tweak(); }

    //--------- digital edge test pattern programming and control:
    else if (strncasecmp(buffer,"setdetp",7)==0)		{ set_detp(); }
    else if (strncasecmp(buffer,"atpatt_en",9)==0)	{ atpatt_en();		}
    else if (strncasecmp(buffer,"dtpatt_en",9)==0)	{ dtpatt_en();		}

    //--------- in-pixel test source programming and control:
    else if (strncasecmp(buffer,"progts",6)==0)		{ progTSource(); }
    else if (strncasecmp(buffer,"loadTSource",11)==0)	{ loadTSource();	}
    else if (strncasecmp(buffer,"loadTSmile1",11)==0)	{ loadTSmile1();	}
    else if (strncasecmp(buffer,"loadTSpixpat1",13)==0)	{ loadTPixPat1();	}
    else if (strncasecmp(buffer,"loadTSpixel",11)==0)	{ loadTSpixel();	}
    else if (strncasecmp(buffer,"loadTSX",11)==0)	{ loadTSX();	}
    else if (strncasecmp(buffer,"turnoffts",9)==0)	{ TurnTSOff();	}
    else if (strncasecmp(buffer,"turnonts",9)==0)	{ TurnTSOn();	}
    else if (strncasecmp(buffer,"loadTStripe",11)==0) { loadTStripe(); }

    //--------- Bank 0 test pattern programming and control:
    else if (strncasecmp(buffer,"setbank0tp",10)==0)		{ set_bank0tp(); }
    else if (strncasecmp(buffer,"setbank0pixel",13)==0)	{ set_bank0pixel(); }
    else if (strncasecmp(buffer,"bank0tp_en",10)==0)	{ bank0tp_en();		}

    //--------- motor and shutter control:
    else if (strncasecmp(buffer,"init_motor",10)==0)		{ initialize_motor(); }
    else if (strncasecmp(buffer,"get_motor_vel",13)==0)	{ get_motor_vel(); }
    else if (strncasecmp(buffer,"set_motor_vel",13)==0)	{ set_motor_vel(); }
    else if (strncasecmp(buffer,"get_motor_pos",13)==0)	{ get_motor_pos(); }
    else if (strncasecmp(buffer,"set_motor_pos",13)==0)	{ set_motor_pos(); }
    else if (strncasecmp(buffer,"set_shutter",11)==0)		{ set_shutter(); }

    //--------- custom exposure loops
    else if (strncasecmp(buffer,"custloop1",9)==0) {	custloop();}
    else if (strncasecmp(buffer,"custloop2",9)==0) {	custloop2();}
    else if (strncasecmp(buffer,"custloop3",9)==0) {	custloop3();}
    else if (strncasecmp(buffer,"custloop4",9)==0) {	custloop4();}
    else if (strncasecmp(buffer,"custloop5",9)==0) {	custloop5();}
    else if (strncasecmp(buffer,"smileloop_mini",14)==0)	{ smileloop_mini(); }
    else if (strncasecmp(buffer,"smileloop",9)==0)	{ smileloop(); }
    else if (strncasecmp(buffer,"xloop_mini",9)==0)	{ Xloop_mini(); }
    else if (strncasecmp(buffer,"xloop_maxi",9)==0)	{ Xloop_maxi(); }
    else if (strncasecmp(buffer,"xloop_itime",10)==0)	{ Xloop_itime(); }
    else if (strncasecmp(buffer,"xloop_pixpat_itime_loop2",24)==0)	{ Xloop_pixpat_itime_loop2(); }
    else if (strncasecmp(buffer,"xloop_pixpat_itime_loop",23)==0)	{ Xloop_pixpat_itime_loop(); }
    else if (strncasecmp(buffer,"xloop_pixpat_itime",18)==0)	{ Xloop_pixpat_itime(); }
    else if (strncasecmp(buffer,"isweep2",7)==0)	{ isweep2(); }
    else if (strncasecmp(buffer,"isweep",6)==0)	{ isweep(); }

    //--------- DEPRECATED - to be deleted?:
    else if (strncasecmp(buffer,"bbias",5)==0)			{ test_bias(); }
    else if (strncasecmp(buffer,"ceb_en",6)==0)	{ ceb_en();		}
    else if (strncasecmp(buffer,"clear_counter",13)==0)	{ ClearCnt();	}

    //--------- not sure what the following are for - may be deprecated?:
    else if (strncasecmp(buffer,"ana_phase",9)==0) {	ana_phase();}

    else { printf("Command Not Found \n"); return -1;}

    return 0;
}




