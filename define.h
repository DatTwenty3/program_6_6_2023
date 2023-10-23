/*--------Output----------*/
#define EN1     PD_ODR_ODR2
#define EN2     PA_ODR_ODR2

#define OUT1    PD_ODR_ODR6
#define OUT2    PD_ODR_ODR5

#define SEG_A   PC_ODR_ODR3
#define SEG_B   PC_ODR_ODR7
#define SEG_C   PC_ODR_ODR6
#define SEG_D   PC_ODR_ODR4
#define SEG_E   PC_ODR_ODR5
#define SEG_F   PB_ODR_ODR5
#define SEG_G   PB_ODR_ODR4

/*--------Input----------*/

#define SW1     PD_IDR_IDR4
#define SW2     PD_IDR_IDR3
#define INPUT   PA_IDR_IDR3

#define ON      1
#define OFF     0

#define PressTime       75
#define TOTAL_NODE      240

unsigned char FlagSW1 = 0;
unsigned char FlagSW2 = 0;
unsigned char FlagRun = 0;
unsigned char FlagPause = 0;
unsigned char FlagEnd = 1;
unsigned char FlagFirst = 0;
unsigned char FlagDelaySec = 0;
unsigned int  CntDelaySec  = 0;
unsigned int cnt0 = 0;
unsigned int cnt1 = 0;
unsigned cntPress = 0;
unsigned char FlagDelay_Msec = 0;
unsigned int CntDelay_Msec = 0;
unsigned char StatusMain = 0;
unsigned char Led1 = 0;
unsigned char Led2 = 0;
unsigned char Led3 = 0;
unsigned char Led4 = 0;
unsigned char FlagSave = 0;

unsigned int Number = 1;
unsigned int temp = 0;
unsigned char IndexOfLed = 0;
unsigned char Blink = 0;
unsigned char cntBlink = 0;
unsigned char Table[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF};
unsigned char Table_Test[11] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x80, 0xFF};
unsigned char IndexOf7Segment = 0;
unsigned int CntNoteH,CntNoteL,NumNoteH,NumNoteL;

const unsigned int TableTimeH[5][240]=
{
//bai 1, Bam chuoi
//1       2       3       4       5       6       7       8       9       10      11      12      13      14      15
{400, 30,400, 30,150, 40, 50, 50, 60, 30,400, 30,400, 30,350, 40,160, 40, 50, 40, 80, 30,400, 30,160, 50, 50, 50, 80, 30,
//16     17      18      19      20      21      22      23      24      25      26      27      28      29      30
160, 50, 40, 50, 70, 30,140, 40, 50, 40, 60, 30,360, 30,160, 40, 50, 60, 80, 40,160, 40, 50, 50, 80, 30,140, 40, 50, 40,
//31     32      33      34      35      36      37      38      39      40      41      42      43      44      45
 80, 40,340, 30,150, 40, 40, 40, 80, 40,120, 40,440,120,250,130,240,130,220,140, 40, 30, 40, 40, 70,120,220,130, 40, 30,
//46     47      48      49      50      51      52      53      54      55      56      57      58      59      60
 50, 50, 60,110,220,130, 40, 30, 40, 40, 70,120, 60, 40, 60, 40, 40,120, 60, 40,310,140, 50, 40,160, 40,360, 30,320, 40,
//61    62      63      64      65      66      67      68      69      70      71      72      73       74      75
330, 30,160, 30,360, 30,160, 30,160, 30,180, 30,380, 30,180, 30,360, 30,160, 30,160, 30,160, 30,160, 30, 40, 40, 80, 30,
//76    77      78      79      80      81      82      83      84      85     86       87     88       89      90
180, 40, 40, 40, 80, 30,140, 40, 40, 40, 60, 30,360, 30,150, 40, 40, 50, 60, 30,150, 40, 40, 50, 70, 30,160, 40, 40, 50,
//91    92      93      94      95      96      97      98      99      100     101     102     103     104      105
 80, 30,340, 30,150, 30, 50, 50, 60, 30,160, 40, 50, 50, 60, 30,160, 30, 50, 50, 60, 30,160, 30,160, 30,140, 40,160
},

//bai 3                                                                                                                          
//1       2       3       4       5       6       7       8       9       10      11      12      13      14      15
{120, 30, 80, 40,250, 30,250,200, 50, 30,230,200, 80, 40,190,230, 50, 40,120, 30,120, 30,260, 30,220,200, 50, 40,200,180,
//16     17      18      19      20      21      22      23      24      25      26      27      28      29      30
 80, 40,200,100,240,150, 50, 40,160,120,200,120, 50, 30, 70, 30, 80,140,200,120, 50, 30, 60, 40, 60,100,240,110, 60, 40,
//31     32      33      34      35      36      37      38      39      40      41      42      43      44      45
 50, 40, 60,100, 60, 40, 60, 40, 40,110, 80, 40, 50, 40, 60,140,240,120, 50, 30,170, 40,340, 30,300, 30,320, 30,130, 30,
//46     47      48      49      50      51      52      53      54      55      56      57      58      59      60
340, 30,140, 30,140, 30,170, 30,150, 30, 50, 50, 60, 30,170, 30, 30, 60, 70, 30,170, 40, 30, 50, 80, 30,160, 40, 20, 60,
//61    62      63      64      65      66      67      68      69      70      71      72      73       74      75
 80, 30,160, 40, 20, 70, 60, 30,200, 40,140, 40,140, 40,150,250, 90, 40,130, 40,120, 70,100,130, 40, 40,120, 50,140,230,
//76    77      78      79      80      81      82      83      84      85     86       87     88       89      90
 90, 40,150, 40,110,110,220,110,210},
//91    92      93      94      95      96      97      98      99      100     101     102     103     104      105
// 80, 30,340, 30,150, 30, 50, 50, 60, 30,160, 40, 50, 50, 60, 30,160, 30, 50, 50, 60, 30,160, 30,160, 30,140, 40,160


//bai 4                                                                                                                          
//1       2       3       4       5       6       7       8       9       10      11      12      13      14      15
{112, 31, 64, 37,250, 30,251,197, 48, 31,229,199, 67, 32,190,231, 45, 35,122, 31,126, 30,255, 32,222,202, 48, 40,204,175,
//16     17      18      19      20      21      22      23      24      25      26      27      28      29      30
 67, 40,194,105,239,142, 43, 37,155,123,194,119, 45, 28, 69, 21, 76,134,204,123, 46, 28, 54, 34, 58, 96,234,108, 56, 36,
//31     32      33      34      35      36      37      38      39      40      41      42      43      44      45
 44, 42, 54,105, 63, 34, 54, 43, 33,110, 78, 36, 43, 44, 56,137,239,122, 45, 33,169, 35,332, 32,305, 32,316, 28,132, 32,
//46     47      48      49      50      51      52      53      54      55      56      57      58      59      60
340, 30,140, 35,142, 29,167, 26,149, 30, 44, 44, 61, 28,165, 29, 25, 53, 63, 26,165, 39, 22, 53, 76, 26,155, 37, 20, 58,
//61    62      63      64      65      66      67      68      69      70      71      72      73       74      75
 74, 23,157, 36, 16, 68, 58, 22,199, 36,139, 38,136, 42,143,250, 89, 34,130, 40,116, 68, 91,124, 37, 37,121, 45,133,229,
//76    77      78      79      80      81      82      83      84      85     86       87     88       89      90
 90, 35,144, 40,106,107,212,109,209},
//91    92      93      94      95      96      97      98      99      100     101     102     103     104      105
// 80, 30,340, 30,150, 30, 50, 50, 60, 30,160, 40, 50, 50, 60, 30,160, 30, 50, 50, 60, 30,160, 30,160, 30,140, 40,160

//bai 7  tich te tich te                                                                                                                         
//1       2       3       4       5       6       7       8       9       10      11      12      13      14      15
{374,185, 21, 30,169, 33,404, 29,368, 32,343,172, 25, 32,182, 33,161,165, 36, 45, 33, 45, 58,138, 68, 43,336,164, 16, 41,
//16     17      18      19      20      21      22      23      24      25      26      27      28      29      30
180, 43,378, 40,350, 41,330,167, 16, 41,161, 42,150, 44,154, 36,141, 43,180,360, 22, 42,145, 41,144, 80, 88,153, 44, 37,
//31     32      33      34      35      36      37      38      39      40      41      42      43      44      45
146, 40,146,245,143, 36,139, 44,106,101,112,101, 91, 96, 96, 95,121,319, 53, 54,158,331, 62, 47,146,157, 35, 41,136,171,
//46     47      48      49      50      51      52      53      54      55      56      57      58      59      60
 20, 42,147, 48,152,162, 26, 42,151, 52, 60, 27, 76,153, 67, 41, 31, 54, 57,141,127, 43,303,169, 22, 42,174, 38,167, 40,
//61    62      63      64      65      66      67      68      69      70      71      72      73       74      75
 33, 48, 79, 36,193, 42, 36, 59, 51, 31,156, 40, 38, 62, 57, 34,152, 35,163, 40,130, 46,159
//76    77      78      79      80      81      82      83      84      85     86       87     88       89      90
//680, 40, 40, 40, 80, 30,140, 40, 40, 40, 60, 30,360, 30,150, 40, 40, 50, 60, 30,150, 40, 40, 50, 70, 30,160, 40, 40, 50,
//91    92      93      94      95      96      97      98      99      100     101     102     103     104      105
//780, 30,340, 30,150, 30, 50, 50, 60, 30,160, 40, 50, 50, 60, 30,160, 30, 50, 50, 60, 30,160, 30,160, 30,140, 40,160
},

//bai 8    tich te te                                                                                                                      
//1       2       3       4       5       6       7       8       9       10      11      12      13      14      15
{115, 70,132,245,123, 41,136, 71,117,148,273, 41,119, 67,111, 76, 82,121, 55, 41,138, 41,159,121,549,142, 64, 37, 54, 40,
//16     17      18      19      20      21      22      23      24      25      26      27      28      29      30
 68,120,242,133, 30, 42, 30, 52, 67,145,217,128, 53, 35, 42, 47, 48,126, 76, 41, 30, 52, 45,130, 61, 34,128, 80, 98,112,
//31     32      33      34      35      36      37      38      39      40      41      42      43      44      45
 26, 52, 83, 26,157, 39,158, 26,151, 40, 32, 56, 65, 25,169, 36,152, 31,145, 43, 26, 62, 70, 29,167, 37,146, 41,133, 41,
//46     47      48      49      50      51      52      53      54      55      56      57      58      59      60
138, 43,125, 48,153,173,184,138, 42, 40, 32, 50, 51,116,216,137, 29, 40, 21, 57, 59,129, 47, 52, 10,114,  6,121, 68, 39,
//61    62      63      64      65      66      67      68      69      70      71      72      73       74      75
115, 98, 95,109,229
//76    77      78      79      80      81      82      83      84      85     86       87     88       89      90
//680, 40, 40, 40, 80, 30,140, 40, 40, 40, 60, 30,360, 30,150, 40, 40, 50, 60, 30,150, 40, 40, 50, 70, 30,160, 40, 40, 50,
//91    92      93      94      95      96      97      98      99      100     101     102     103     104      105
//780, 30,340, 30,150, 30, 50, 50, 60, 30,160, 40, 50, 50, 60, 30,160, 30, 50, 50, 60, 30,160, 30,160, 30,140, 40,160
},
};

const unsigned int TableTimeL[12][150]=
{
//bai 1
//1       2       3       4       5       6       7       8       9      10      11      12      13      14      15
{ 53,141, 55,141,252,137, 53,141, 55,141,253,137, 52,141, 55,141,253,137, 53,140, 56,141,252,137, 53,141, 55,141,253,136,
//16     17      18      19      20      21      22      23      24      25      26      27      28      29      30
 53,141, 55,141,253,137, 52,141, 56,141,252,137, 53,141, 55,141,253,137, 52,141, 55,141,253,137, 53,141, 55,141,252,137,
//31    32       33      34      35      36      37      38      39      40      41      42      43      44      45
 200,200,200,200,200,200,200,200,200,200,250,250,250,250,250,250,250,250,250,250,050,050,050,050,050,050,050,050,050,050,
 //46    47       48      49      50      51      52      53      54      55      56      57      58      59      60
 31, 35,136, 39,130,140, 29, 36,144, 39,121,135, 26, 34, 33, 45, 63,108, 63, 38,118,106, 98,168,177,228, 77, 38, 24, 48,
//61      62      63      64      65    
 250,250,050,050,050,050,050,050,050,050},
//bai 2 
//1       2       3       4       5       6       7       8       9      10      11      12      13      14      15
{123, 39,159, 40, 43, 39, 64,323, 64, 35,137, 38, 44, 39, 64,134, 40, 36, 39, 36, 64,115, 60, 35,138,306, 35, 37,132, 38,
//16     17      18      19      20      21      22      23      24      25      26      27      28      29      30
 140, 89, 93,152, 21, 40,132, 40,148,291, 42, 42,114, 42,129,116,218,120,204,280, 38, 48,137,323, 47, 45,120,153, 30, 44,
//31    32       33      34      35      36      37      38      39      40      41      42      43      44      45
 126,155, 19, 42,121, 45,119,132, 24, 43,135, 44,123,137, 01, 01, 15, 41, 26, 47, 60,124, 38, 44,116, 93,108,283, 43,112,
 //46    47       48      49      50      51      52      53      54      55      56      57      58      59      60
 59,127, 42, 45, 26, 44, 49,136, 56, 45, 82,134, 52, 41,130, 42,320, 38,311, 38,304, 35,137, 39,315, 36,128, 41,150, 38,
//61      62      63      64      65    
148, 38,317, 36,154, 33,322, 33,144, 35},
{0},
{0},
{0},
{0},
{0},
{0},
{0},
{0},
{0},
{0},
};

void Input_Process(void);
void MainProcess();
void ADC_Process();
void Main_Process();
void DelaySec(unsigned int Sec);
void DelayMiliSec(unsigned int m_sec);
void Protect();
void Display7Segment();
void DisplayNumber(unsigned int number);
void Vitual_Timer();