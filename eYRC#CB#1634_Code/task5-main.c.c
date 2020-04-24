/*
 * Team Id: 1634
 * Author List: Praveen Yadav,Praveen Dwivedi,Darshan Savaliya,Arun Kumar
 * Filename: task5_main.c
 *Theme: Construct-O-Bot – eYRC Specific
 * Functions: void picking(int material),placing(int house,int i),travel(int p1, int p2),wall_follower(void),is_white_Node(void),white_right_turn_wls(void),
 void white_left_turn_wls(void);
 void LCD_Command(unsigned char cmnd);
 void LCD_Char (unsigned char char_data);
 void LCD_Init (void);
 void LCD_String (char *str)	;
 void LCD_String_xy (char row, char pos, char *str);
 void LCD_Clear();
 void wls_print();
 void buzzer();
 void turn_180();
 void servo1(int initial,int final);
 void servo2(int initial,int final);
 void servo3(int initial,int final);
 void servo4(int initial,int final);
 void velocity (unsigned char left_motor, unsigned char right_motor);
 void read_wls(void);
 void stopmm();
 void forward();
 void backward();
 void turn_right();
 void turn_left();
 void soft_right();
 void soft_left();
 void pick_bot();
 void place_lr();
 void place_hr();
 void place_in(int d, int pi, int pl);
 void pick_from(int d, int a, int p);
 void papom( int pick, int place,int node);
 void stepper(int rot, int deg);
 void wall_follower();
 void ir();
 void forward_wls_zz(int node,int speed);
 void line_follower();
 void make_path();
 * Global Variables: direction,f,left,right,wls_left,wls_right,wls_middle,count,step,count1,ir_right,ir_left,max_distance,isSelected,CM_place_in_bot[4],high_or_low[5],mat_req_for_house[5],mat_remain,or_path[17],dup_path[12],house[5],
 
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
typedef int bool;
#define TRUE  1
#define FALSE 0
#define  F_CPU 14745600
#define stepc 1
#define stepac 0
#define clk 1
#define anclk 0
#define byte uint8_t
#include <avr/interrupt.h>
#include <util/delay.h>
int sensor_values[3];
////////////////////////////////////////////all about motor///////////////////////////////////////////////////////////////////////////
int left=0;
int right=0;
int wls_left;
int wls_middle;
int wls_right;
int count=0;

//LCD definition......................................///////////////////////////////////////
#define LCD_Data_Dir DDRK		/* Define LCD data port direction */
#define LCD_Command_Dir DDRC		/* Define LCD command port direction register */
#define LCD_Data_Port PORTK		/* Define LCD data port */
#define LCD_Command_Port PORTC		/* Define LCD data port */
#define RS PC0				/* Define Register Select (data/command reg.)pin */
#define RW PC1				/* Define Read/Write signal pin */
#define EN PC2				/* Define Enable signal pin */
///////////////////////////////////////////////////////////////////////////////////////////////



void left_encoder_pin_config (void) //Configure Interrupt 6
{
	DDRE = DDRE & 0x9F; //Set the direction of the PORTE 6 pin as input
	PORTE = PORTE | 0x40; //Enable internal pull-up for PORTE 6 pin
}

void right_encoder_pin_config (void) //Configure Interrupt 5
{
	DDRE = DDRE & 0xDF; //Set the direction of the PORTE 5 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 5 pin
}
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x20; // INT6 is set to trigger with falling edge
	EIMSK = EIMSK | 0x40; // Enable Interrupt INT6 for left position encoder
	sei(); // Enables the global interrupt
}
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for left position encoder
	sei(); // Enables the global interrupt
}
ISR(INT6_vect)
{
	left+=1;
}
ISR(INT5_vect)
{
	right+=1;
}


//graph definition......................................///////////////////////////////////////
#define INFINITY 99
#define MAX 16
void dijkstra(int G[MAX][MAX],int n,int startnode,int lastnode);
int path[15][15];
int step=0;
///////////////////////////////////////////////////////////////////////////////////////////////



//movement definitions..................................///////////////////////////////////////
#define clk 1
#define anclk 0
int direction=0;
int f=0;
///////////////////////////////////////////////////////////////////////////////////////////////



//functions definitions..................................//////////////////////////////////////
bool to_pick = TRUE;
void picking(int material);
void placing(int house,int i);
void travel(int p1, int p2);
void wall_follower(void);
bool is_white_Node(void);
void white_right_turn_wls(void);
void white_left_turn_wls(void);
void LCD_Command(unsigned char cmnd);
void LCD_Char (unsigned char char_data);
void LCD_Init (void);
void LCD_String (char *str)	;
void LCD_String_xy (char row, char pos, char *str);
void LCD_Clear();
void wls_print();
void buzzer();
void turn_180();
void servo1(int initial,int final);
void servo2(int initial,int final);
void servo3(int initial,int final);
void servo4(int initial,int final);
void velocity (unsigned char left_motor, unsigned char right_motor);
void read_wls(void);
void stopmm();
void forward();
void backward();
void turn_right();
void turn_left();
void soft_right();
void soft_left();
void pick_bot();
void place_lr();
void place_hr();
void place_in(int d, int pi, int pl);
void pick_from(int d, int a, int p);
void papom( int pick, int place,int node);
void stepper(int rot, int deg);
void wall_follower();
void ir();
void forward_wls_zz(int node,int speed);
void line_follower();
void make_path();


int wls_left, wls_right, wls_middle;
int ir_left,ir_right;
int count1=0;
unsigned char new_state='r';

/*
 * Function Name:<all_init>
 * Input: no
 * Output:no
 * Logic: avr pins are initialized
 * Example Call: all_init()
 */

void all_init()
{
	
	//stepper connections
	//DDRA=0x3F;
	//PORTA =0x20;
	
	//direction 0   anticlockwise
	//direction 1   clockwise
	
	
	
	//step A0			 1
	//direction A1		 2
	//MS1 A2			 4
	//MS2 A3			 8
	//MS3 A4			 10
	//Enable A5			 20
	
	//left motor pins 0 and 1
	//right motor pins 2 and 3
	DDRD=0x0f;
	PORTD =0x00;
	DDRL=0xFF;
	PORTL=0xFF;
	DDRH=0xFF;
	PORTH=0xFF;
	//timer setup.........................
	//left and right motors pwm speed registers()
	TCCR5A = 0xA9;
	TCCR5B = 0x0B;
	TCNT5H = 0xFF;
	TCNT5L = 0x00;
	OCR5AH = 0x00;//PL3
	OCR5AL = 0xFF;
	OCR5BH = 0x00;//PL4
	OCR5BL = 0xFF;
	
	
	//servo connections
	DDRB=0xFF;
	PORTB=0xff;
	DDRH=0xFF;
	PORTH=0xff;
	DDRE=0xFF;
	PORTE=0xff;
	
	//Servos pwm registers
	// 	TCCR1A = 0xA9;
	// 	TCCR1B = 0x0B;
	// 	TCNT1H = 0xFF;
	// 	TCNT1L = 0x00;
	// 	OCR1AH = 0x00;    //PB5  servo 4
	// 	OCR1AL = 0xFF;
	// 	OCR1BH = 0x00;   //PB6   servo 3
	// 	OCR1BL = 0xFF;
	// 	OCR1CH = 0x00;   //PB7   servo 2
	// 	OCR1CL = 0xFF;

	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)

	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	
	TCCR4A|=(1<<COM4A1)|(1<<COM4B1)|(1<<WGM41);        //NON Inverted PWM//ph3>>oc4a/ph4>>oc4b
	TCCR4B|=(1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<CS40); //PRESCALER=64 MODE 14(FAST PWM)

	ICR4=4999;  //fPWM=50Hz (Period = 20ms Standard).

	TCCR3A|=(1<<COM3A1)|(1<<COM3B1)|(1<<WGM31);        //NON Inverted PWM
	TCCR3B|=(1<<WGM33)|(1<<WGM32)|(1<<CS31)|(1<<CS30); //PRESCALER=64 MODE 14(FAST PWM)

	ICR3=4999;  //fPWM=50Hz (Period = 20ms Standard).
	
	
	
	//servo initial positions.............................................
	OCR3A=97;        //servo 00
	OCR3B=97;	//SERVO01
	OCR4A=365;    //SERVO1
	//Wait();
	OCR4B=450;    //SERVO2
	// Wait();
	OCR1B=316;		//SERVO3
	OCR1A=300;		//SERVO4
	LCD_Init();
	left_encoder_pin_config();
	right_encoder_pin_config();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	
}

//LCD functions ..................................///////////////////////////////////////////////////////////////////////////////////

/*
 * Function Name:LCD_Command
 * Input: cmnd
 * Output: no
 * Logic: lcd cursor position changes according to data port
 * Example Call: LCD_Command(unsigned char cmnd )
 */

void LCD_Command(unsigned char cmnd)
{
	LCD_Data_Port= cmnd;
	LCD_Command_Port &= ~(1<<RS);	/* RS=0 command reg. */
	LCD_Command_Port &= ~(1<<RW);	/* RW=0 Write operation */
	LCD_Command_Port |= (1<<EN);	/* Enable pulse */
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(3);
}

/*
 * Function Name:LCD_Char
 * Input: char_data
 * Output: prints a character on led 
 * Logic: character is printed with the help of data pins
 * Example Call: LCD_Char (unsigned char a)
*/
void LCD_Char (unsigned char char_data)	/* LCD data write function */
{
	LCD_Data_Port= char_data;
	LCD_Command_Port |= (1<<RS);	/* RS=1 Data reg. */
	LCD_Command_Port &= ~(1<<RW);	/* RW=0 write operation */
	LCD_Command_Port |= (1<<EN);	/* Enable Pulse */
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	_delay_ms(1);
}

/*
 * Function Name:<LCD_Init>
 * Input: no
 * Output:no
 * Logic: lcd pins are initialized
 * Example Call: LCD_Init ()
 */
void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Command_Dir = 0xFF;		/* Make LCD command port direction as o/p */
	LCD_Data_Dir = 0xFF;		/* Make LCD data port direction as o/p */
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	LCD_Command (0x38);		/* Initialization of 16X2 LCD in 8bit mode */
	LCD_Command (0x0C);		/* Display ON Cursor OFF */
	LCD_Command (0x06);		/* Auto Increment cursor */
	LCD_Command (0x01);		/* Clear display */
	LCD_Command (0x80);		/* Cursor at home position */
}

/*
 * Function Name:LCD_String
 * Input: string
 * Output:prints on led
 * Logic: string is printed with the help LCD_Char
 * Example Call: LCD_String (char *str)
 */
void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

/*
 * Function Name:LCD_String_xy
 * Input: row, position,string
 * Output:prints on led at specific position
 * Logic: string is printed with the help LCD_String and LCD_Command
 * Example Call: LCD_String_xy (1, 5, go)
 */
void LCD_String_xy (char row, char pos, char *str)/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);	/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);	/* Command of first row and required position<16 */
	LCD_String(str);		/* Call LCD string function */
}

/*
 * Function Name:LCD_Clear
 * Input: no
 * Output:no
 * Logic: lcd screen is cleared using LCD_Command
 * Example Call: LCD_Clear()
 */
void LCD_Clear()
{
	LCD_Command (0x01);		/* clear display */
	LCD_Command (0x80);		/* cursor at home position */
}

/*
 * Function Name:wls_print
 * Input:no
 * Output:print white line sensor values
 * Logic: prints sensor value using lcd
 * Example Call: wls_print()
 *
 */
void wls_print()
{
	unsigned char c[5];
	read_wls();
	itoa(wls_left, c, 10);
	LCD_String(c);
	
	itoa(wls_middle, c, 10);
	LCD_String_xy(0,5,c);
	
	itoa(wls_right, c, 10);
	LCD_String_xy(1,0,c);
	_delay_ms(10);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                             ALGO PART
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CM_place_in_bot[4]={0,0,0,0};

 int or_path[17]={0,0,0,0,11};
 int dup_path[12];
 
 int max_distance= 99;
 bool isSelected=FALSE;
 int mat_picked[]={2,2};
 int Dist[12][12]={  {0, 5, 2, 5, 4, 7, 1, 4, 3, 6, 7,  3},//1
 {5, 0, 5, 2, 7, 4, 4, 1, 6, 3, 7,  3},//2
 {2, 5, 0, 5, 2, 7, 1, 4, 1, 6, 5,  5},//3
 {5, 2, 5, 0, 7, 2, 4, 1, 6, 1, 5,  5},//4
 {4, 7, 2, 7, 0, 6, 3, 6, 1, 6, 3,  7},//5
 {7, 4, 7, 2, 6, 0, 6, 3, 6, 1, 3,  7},//6
 {1, 4, 1, 4, 3, 6, 0, 3, 2, 7, 6,  4},//h1 7
 {4, 1, 4, 1, 6, 3, 3, 0, 7, 2, 6,  4},//h2 8
 {3, 6, 1, 6, 1, 6, 2, 7, 0, 5, 4,  6},//h3 9
 {6, 3, 6, 1, 6, 1, 7, 2, 5, 0, 4,  6},//h4 10
 {7, 7, 5, 5, 3, 3, 6, 6, 4, 4, 0, 10},//h5 11
 {3, 3, 5, 5, 7, 7, 4, 4, 6, 6, 10, 0},//start
 };
 
 int house[5]={5,1,2,3,4};
	 
/////////////////////////////////////////////////////////////////
//             CONFIGURATION                                   //
/////////////////////////////////////////////////////////////////
int CM[10] = {0,0,0,0,0,0,0,0,0};
int high_or_low[5]={0,0,0,0,0};
int mat_req_for_house[5]={2,2,2,2,2};
int mat_remain[6]={1,1,1,1,1,1};
//////////////////////////////////////////////////////////////
 struct available_mat
 {
	 int mat;
	 struct available_mat *next;
 };


/*
 * Function Name:               calculate_dist
 * Input:                       mat1,mat2 -> node number of CM which can be placed in bot_mat
                                prev_H-> previous house from where bot will start
                                now_H ->house bot is going now
 * Output:                      return the total minimum distance from prev_H to now_H including the CMs in path 
 * Logic:                       first it calculates which CM is nearer to prev_H then second CM if it is there(mat value is 0 if no mat is there)
                                then it sum all the distance 
 * Example Call:                calculate_dist(2,4,4,5);
 *
 */ 

int calculate_dist(int mat1,int mat2,int prev_H,int now_H)
{
	if(mat1==0 || mat2==0)
	{
		if(mat1!=0)
		return Dist[prev_H+5][mat1-1]+Dist[mat1-1][now_H+5];
		else if(mat2!=0)
		return Dist[prev_H+5][mat2-1]+Dist[mat2-1][now_H+5];
		else
		return Dist[prev_H+5][now_H+5];
	}
	else
	{
		if(Dist[prev_H+5][mat1-1]<Dist[prev_H+5][mat2-1])
		return Dist[prev_H+5][mat1-1]+Dist[mat1-1][mat2-1]+Dist[mat2-1][now_H+5];
		else
		return Dist[mat2-1][prev_H+5]+Dist[mat1-1][mat2-1]+Dist[now_H+5][mat1-1];
	}
}

/*
 * Function Name:               decide_which_mat_take
 * Input:                       available_mat* prev-> linked list which contain list of available CM 
                                available_mat* now->  linked list which will contain possible CM which can be picked and put in bot
                                bot_mat->materials which are currently present in bot 
                                H->house node value where bot has to go next
                                *b ->what type of combination is done is stored in this address
 * Output:                      void 
 * Logic:                       it checks how many material required for next house are already present in bot, based on that it makes combination of material which can be put in bot 
                                during travel to next house
 * Example Call:                decide_which_mat_take(prev,now,bot_mat,3,&a);
 *
 */ 

 void decide_which_mat_take(struct available_mat* prev,struct available_mat* now,int bot_mat[],int H, int *b)
 {
	 int a=2;
	 
	 
	 struct available_mat *p,*q,*s1,*s2,*temp;
	 
	 bool CM1=TRUE;
	 bool CM2=TRUE;
	 s1=now;
	 s2=prev;
	 int mat_for_house_H[] = {CM[2*H-1],CM[2*H-2]};
	 
	 unsigned char c[10];
	 
	 //check bot_mat with mat req for given house
	 if(bot_mat[0]==mat_for_house_H[0] && CM1)
	 {
		 
		 CM1=FALSE;
		 a--;
	 }
	 else if(bot_mat[0]==mat_for_house_H[1] && CM2)
	 {
		 
		 CM2=FALSE;
		 a--;
	 }
	 if(bot_mat[1]==mat_for_house_H[0] && CM1)
	 {
		 
		 CM1=FALSE;
		 a--;
	 }
	 else if(bot_mat[1]==mat_for_house_H[1] && CM2)
	 {
		 
		 CM2=FALSE;
		 a--;
	 }
	 if(!CM1 && a==1)
	 {
		 q=malloc(sizeof(struct available_mat));
		 now->mat=mat_for_house_H[1];
		 
		 now->next=q;
		 now=q;
	 }
	 else if(!CM2 && a==1)
	 {
		 q=malloc(sizeof(struct available_mat));
		 now->mat=mat_for_house_H[0];
		 
		 now->next=q;
		 now=q;
	 }
	 bool ismat1=FALSE;
	 bool ismat2=FALSE;
	 bool for_H=FALSE;
	 *b=a;
	 int i=0;
	 while(prev!=NULL)
	 {
		 
		 //if mat in bot is mat req for house
		 if(a==0)
		 
		 {
			 
			 if((prev->mat!=mat_picked[0] || ismat1) && (prev->mat!=mat_picked[1] || ismat2) )
			 {
				 q=malloc(sizeof(struct available_mat));
				 now->mat=prev->mat;
				 
				 
				 now->next=q;
				 temp=now;
				 now=q;
			 }
			 else
			 {
				 if(prev->mat==mat_picked[0])
				 ismat1=TRUE;
				 else if(prev->mat==mat_picked[1])
				 ismat2=TRUE;
			 }
			 
			 prev=prev->next;
		 }
		 
		 else if(a==1)
		 {
			 if(!CM1)
			 {
				 
				 if(((prev->mat!=mat_picked[0] || ismat1) && (prev->mat!=mat_picked[1] || ismat2) )&& (prev->mat!=mat_for_house_H[1] || for_H) )
				 {
					 q=malloc(sizeof(struct available_mat));
					 now->mat=prev->mat;
					 
					 now->next=q;
					 temp=now;
					 now=q;
				 }
				 else
				 {
					 if(prev->mat==mat_picked[0])
					 ismat1=TRUE;
					 else if(prev->mat==mat_picked[1])
					 ismat2=TRUE;
					 else if(prev->mat==mat_for_house_H[1])
					 for_H=TRUE;
				 }
				 
				 prev=prev->next;
				 
			 }
			 else if(!CM2)
			 {
				 
				 if(((prev->mat!=mat_picked[0] || ismat1) && (prev->mat!=mat_picked[1] || ismat2) )&& (prev->mat!=mat_for_house_H[0] || for_H))
				 {
					 q=malloc(sizeof(struct available_mat));
					 now->mat=prev->mat;
					 
					 now->next=q;
					 temp=now;
					 now=q;
				 }
				 else
				 {
					 if(prev->mat==mat_picked[0])
					 ismat1=TRUE;
					 else if(prev->mat==mat_picked[1])
					 ismat2=TRUE;
					 else if(prev->mat==mat_for_house_H[0])
					 for_H=TRUE;
				 }
				 
				 prev=prev->next;
			 }
		 }
		 //mat req for house is not in bot hence take those material only
		 else if(a==2)
		 {
			 unsigned char c[10];
			 
			 prev=NULL;
			 q=malloc(sizeof(struct available_mat));
			 
			 
			 now->mat=mat_for_house_H[0];
			 
			 now->next=q;
			 now=q;
			 now->mat=mat_for_house_H[1];
			 
			 
		 }
	 }
	 if(a!=2)
	 {
		 now=temp;
		 free(q);
	 }
	 now->next=NULL;
	 now=s1;
	 prev=s2;
	 
	 
 }

/*
 * Function Name:               swap
 * Input:                       *x->address of first number
                                *y->address of second numder
 * Output:                      void 
 * Logic:                       it takes the address of both x and y and swap the address
 * Example Call:                swap(&a,&b);
 *
 */ 

 void swap(int *x, int *y)
 {
	 int temp;
	 temp = *x;
	 *x = *y;
	 *y = temp;
 }


 

 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              CM_pick_order
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
 * Function Name:               CM_pick_order
 * Input:                       step->number of houses it reached after H5 
                                comb_of_HOUSE->array which store combinatio of houses 
                                available_mat* prev-> linked list which contain list of available CM 
                                mat_in_bot-> CM which are in both
                                distance_upto_now->how much distance covered
 * Output:                      void 
 * Logic:                       it recursively makes combination of 2 materials for each house and calculate the distance 
                                it also keeps record of the distance covered upto now if it reaches max value of distance it stops the recursion and backtrack to next combination
                                
 * Example Call:                CM_pick_order(2,house,prev,mat_in_bot,44);
 *
 */ 

  void CM_pick_order(int step,int comb_of_HOUSE[5],struct available_mat* prev,int mat_in_bot[],int distance_upto_now)
  {

	  step++;
	  if(step==4)
	  {

		  int distance=distance_upto_now+Dist[comb_of_HOUSE[step-1]+6][comb_of_HOUSE[step]+6];
		  
		  if(distance<max_distance)
		  {
			  for(int i=0;i<9;i++)
			  or_path[i+5]=dup_path[i];
			  
			  
			  max_distance=distance;
			  or_path[3*(step-1)+5]=0;
			  or_path[3*(step-1)+1+5]=0;
			  or_path[3*(step-1)+2+5]=comb_of_HOUSE[step]+6;
			  
			  
		  }
	  }
	  else if(step!=4)
	  {
		  int prev_m1=mat_picked[0];
		  int prev_m2=mat_picked[1];
		  
		  struct available_mat *now,*dup_now,*availmat,*dup_availmat,*head,*comb1,*comb2,*q,*for_availmat,*for_prev,*for_c1,*for_c2;
		  int a;
		  now = malloc(sizeof(struct available_mat));
		  dup_now=now;
		  availmat=malloc(sizeof(struct available_mat));
		  dup_availmat=availmat;
		  
		  //see available combination of available_materials
		  decide_which_mat_take(prev, now, mat_in_bot, comb_of_HOUSE[step], &a);
		  
		  //this will be used for generating combination of materials
		  comb1=now;
		  comb2=now->next;
		  
		  //now combination of available materials
		  
		  head=availmat;
		  for_prev=prev;
		  bool ismat1=FALSE;
		  bool ismat2=FALSE;
		  bool CM1=FALSE;
		  bool CM2=FALSE;
		  bool isQ=FALSE;
		  for_availmat=availmat;
		  while(prev!=NULL)
		  {
			  //making list for next available material
			  if((prev->mat!=mat_picked[0] || ismat1) && (prev->mat!=mat_picked[1] || ismat2)  )
			  {
				  isQ=TRUE;
				  q=malloc(sizeof(struct available_mat));
				  availmat->mat=prev->mat;
				  for_availmat=availmat;
				  
				  availmat->next=q;
				  availmat=q;
			  }
			  else
			  {
				  if(prev->mat==mat_picked[0])
				  ismat1=TRUE;
				  else if(prev->mat=mat_picked[1])
				  ismat2=TRUE;
			  }
			  
			  prev=prev->next;
		  }
		  if(isQ)
		  free(q);
		  
		  availmat=for_availmat;
		  
		  availmat->next=NULL;
		  availmat=head;
		  prev=for_prev;
		  
		  int distance;
		  //begin the combination of available_materials
		  
		  if(a==0)
		  {
			  while(comb1->next!=NULL)
			  {
				  while(comb2!=NULL)
				  {
					  
					  distance=distance_upto_now+calculate_dist(comb2->mat,comb1->mat,comb_of_HOUSE[step-1],comb_of_HOUSE[step]);
					  if(distance<max_distance)
					  {
						  
						  mat_in_bot[0]=comb2->mat;
						  mat_in_bot[1]=comb1->mat;
						  if(Dist[comb_of_HOUSE[step-1]][comb1->mat]<Dist[comb_of_HOUSE[step-1]][comb2->mat])
						  {
							  dup_path[3*(step-1)]=comb1->mat;
							  dup_path[3*(step-1)+1]=comb2->mat;
						  }
						  else
						  {
							  dup_path[3*(step-1)]=comb2->mat;
							  dup_path[3*(step-1)+1]=comb1->mat;
						  }
						  dup_path[3*(step-1)+2]=comb_of_HOUSE[step]+6;
						  int M1=mat_picked[0];
						  int M2=mat_picked[1];
						  mat_picked[0]=comb1->mat;
						  mat_picked[1]=comb2->mat;
						  
						  CM_pick_order(step, comb_of_HOUSE, availmat, mat_in_bot, distance);
						  
						  mat_picked[0]=M1;
						  mat_picked[1]=M2;
						  
						  
					  }
					  
					  comb2=comb2->next;
				  }
				  
				  comb1=comb1->next;
				  comb2=comb1->next;
			  }
		  }
		  else if(a==1)
		  {
			  
			  while(comb2!=NULL)
			  {
				  
				  distance=distance_upto_now+calculate_dist(comb2->mat,comb1->mat,comb_of_HOUSE[step-1],comb_of_HOUSE[step]);
				  
				  if(distance<max_distance)
				  {
					  for(int i=0;i<2;i++)
					  {
						  if(mat_in_bot[i]==CM[2*comb_of_HOUSE[step]-1] ||  mat_in_bot[i]==CM[2*comb_of_HOUSE[step]-2] )
						  {
							  
							  mat_in_bot[i]=comb2->mat;
							  break;
						  }
					  }
					  
					  if(Dist[comb_of_HOUSE[step-1]][comb1->mat]<Dist[comb_of_HOUSE[step-1]][comb2->mat])
					  {
						  dup_path[3*(step-1)]=comb1->mat;
						  dup_path[3*(step-1)+1]=comb2->mat;
					  }
					  else
					  {
						  dup_path[3*(step-1)]=comb2->mat;
						  dup_path[3*(step-1)+1]=comb1->mat;
					  }
					  dup_path[3*(step-1)+2]=comb_of_HOUSE[step]+6;
					  int M1=mat_picked[0];
					  int M2=mat_picked[1];
					  mat_picked[0]=comb1->mat;
					  mat_picked[1]=comb2->mat;
					  
					  CM_pick_order(step, comb_of_HOUSE, availmat, mat_in_bot, distance);
					  mat_picked[0]=M1;
					  mat_picked[1]=M2;
					  
				  }
				  comb2=comb2->next;
			  }
			  
		  }
		  else if(a==2)
		  {
			  
			  
			  distance=distance_upto_now+calculate_dist(comb2->mat,comb1->mat,comb_of_HOUSE[step-1],comb_of_HOUSE[step]);
			  
			  if(distance<max_distance)
			  {
				  
				  
				  if(Dist[comb_of_HOUSE[step-1]][comb1->mat]<Dist[comb_of_HOUSE[step-1]][comb2->mat])
				  {
					  dup_path[3*(step-1)]=comb1->mat;
					  dup_path[3*(step-1)+1]=comb2->mat;
				  }
				  else
				  {
					  dup_path[3*(step-1)]=comb2->mat;
					  dup_path[3*(step-1)+1]=comb1->mat;
				  }
				  
				  dup_path[3*(step-1)+2]=comb_of_HOUSE[step]+6;
				  
				  int M1=mat_picked[0];
				  int M2=mat_picked[1];
				  mat_picked[0]=comb1->mat;
				  mat_picked[1]=comb2->mat;
				  
				  CM_pick_order(step, comb_of_HOUSE, availmat, mat_in_bot, distance);
				  mat_picked[0]=M1;
				  mat_picked[1]=M2;
				  
			  }
		  }
		  
		  delete_list(dup_now);
		  delete_list(dup_availmat);
		  
		  // free(&a);
		  mat_picked[0]=prev_m1;
		  mat_picked[1]=prev_m2;
		  //free(comb1);
		  //free(comb2);
	  }
	  
	  
  }

 void delete_list(struct available_mat *head)
 {
	 struct available_mat *temp;
	 while(head->next!=NULL)
	 {
		 temp=head;
		 head=head->next;
		 free(temp);
	 }
	 free(head);
 }
/*
 * Function Name:               permute
 * Input:                       *a->address of array for which permutation has to be done 
                                l->left index of array from which permutation has to start 
                                r->right index of array upto which permutation has to done
                                available_mat* prev-> linked list which contain list of available CM 
                                flag-> which array has to make combination
                                distance_upto_now->how much distance covered
 * Output:                      void 
 * Logic:                       it recusively goes to index of array and swap it with next next and then backtrack
 * Example Call:                permute(house,1,3,prev,1);
 *
 */ 

void permute(int *a, int l, int r,struct available_mat* prev,int flag)  
{
	int i;
	if (l == r)
	{
		if(flag==0)
		{
			int start = 11;
			int H5 = 10;

			int distance = Dist[start][a[0]-1]+Dist[a[0]-1][a[1]-1]+Dist[a[1]-1][a[2]-1]+Dist[a[2]-1][a[3]-1]+Dist[a[3]-1][H5];
			if(distance<max_distance)
			{
				
				for(i=0;i<4;i++)
					or_path[i]=a[i];
				
				
				isSelected=TRUE;
				max_distance=distance;
				
			}
			
		}
		else
		{
			
			int a = mat_picked[0];
			int b = mat_picked[1];
			int bot_mat[] = {mat_picked[0],mat_picked[1]};
			int distance=0;
			int step = 0;
			
			mat_picked[0]=bot_mat[0];
			mat_picked[1]=bot_mat[1];
			CM_pick_order(step,house,prev,bot_mat,distance);
			
			mat_picked[0]=a;
			mat_picked[1]=b;
		}
	}

	else
	{
		for (i = l; i <= r; i++)
		{
			swap((a+l), (a+i));
			permute(a, l+1, r,prev,flag);
			swap((a+l), (a+i)); //backtrack
		}
	}
}
/*
 * Function Name:               decide_CM_for_H5
 * Input:                       void
 * Output:                      void 
 * Logic:                       it makes combination of 4 material including 2 material reaquired for house H5 
                                in shortest distance
 * Example Call:                decide_CM_for_H5();
 *
 */ 

void decide_CM_for_H5()
{
	int H5_mat_array[4]={CM[8],CM[9],0,0};
	for(int i=0;i<6;i++)
	{
		if(CM[i]!=0)
		{
			for(int j=i+1; j<7;j++)
			{
				if(CM[j]!=0)
				{
					H5_mat_array[2]=CM[i];
					H5_mat_array[3]=CM[j];
					struct available_mat* dummy;
					permute(H5_mat_array,0,3,dummy,0);
					if(isSelected)
					{
						mat_picked[0]=CM[i];
						mat_picked[1]=CM[j];
					}
					isSelected=FALSE;
				}
			}
		}
	}
	max_distance=999;
}

/*
 * Function Name:               make_path
 * Input:                       void
 * Output:                      void 
 * Logic:                       it calls the function decide_CM_for_H5 and permute which store the node value of CM and houses in array or_path
 * Example Call:                make_path();
 *
 */ 
void make_path()
{
	struct available_mat *head,*p,*prev,*q;
	// copying CM from array to linked list
	head=malloc(sizeof(struct available_mat));
	head->mat=CM[0];
	head->next=NULL;
	p=head;
	prev=head;
	for(int i=1;i<8;i++)
	{
		q=malloc(sizeof(struct available_mat));
		p->next=q;
		q->mat=CM[i];
		
		if(i<8)
		p=q;
		else
		p->next=NULL;
	}
	decide_CM_for_H5();
	permute(house, 1, 4,prev,1);
	delete_list(head);
}

/*
 * Function Name: shortest_path
 * Input: u,l >>any two point as defined in graph
 * Output: bot moves from one point to another in shortest path
 * Logic: uses djikstra and gives an array of path containig connected nodes and uses travel to move from one point to another using array 
 * Example Call:shortest_path(2,10)
 */ 
void shortest_path(int u,int l)
{
	
		int i,j,n;
		n=16;
		int G[16][16]={{0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
		{2,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,1,0,1,0,0,0,0,0,0,0,0,4,0},
		{0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,1,0,1,0,0,0,0,5,0,0,0},
		{0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,1,0,2,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,2,0,2,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,2,0,1,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
		{0,0,0,0,0,0,5,0,0,0,0,1,0,1,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0},
		{0,0,0,0,4,0,0,0,0,0,0,0,0,1,0,1},
		{1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0}};
		

		
		if(u!=l)
		{
			dijkstra(G,n,u,l);
			
			for(int i=0;i<count1;i++)
			{
				travel(path[step][i],path[step][i+1]);
			}
		}
		return 0;
}

/*
 * Function Name:  dijkstra
 * Input: G[][]>>COST MATRIX
		n >> no. of nodes defined in graph
		startnode,lastnode>> nodes to start and end
 * Output: gives an array of nodes
 * Logic: uses dijkstra logic and gives shotest path 
 * Example Call:dijkstra(int G[MAX][MAX],16,2,10)
 */

void dijkstra(int G[MAX][MAX],int n,int startnode,int lastnode)
{
	int cost[MAX][MAX],distance[MAX],pred[MAX];
	int visited[MAX],count,mindistance,nextnode,i,j;
	//pred[] stores the predecessor of each node
	//count gives the number of nodes seen so far
	//create the cost matrix
	for(i=0;i<n;i++)
	for(j=0;j<n;j++)
	if(G[i][j]==0)
	cost[i][j]=INFINITY;
	else
	cost[i][j]=G[i][j];
	
	//initialize pred[],distance[] and visited[]
	for(i=0;i<n;i++)
	{
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}
	
	distance[startnode]=0;
	visited[startnode]=1;
	count=1;
	
	while(count<n-1)
	{
		mindistance=INFINITY;
		
		//nextnode gives the node at minimum distance
		for(i=0;i<n;i++)
		if(distance[i]<mindistance&&!visited[i])
		{
			mindistance=distance[i];
			nextnode=i;
		}
		
		//check if a better path exists through nextnode
		visited[nextnode]=1;
		for(i=0;i<n;i++)
		if(!visited[i])
		if(mindistance+cost[nextnode][i]<distance[i])
		{
			distance[i]=mindistance+cost[nextnode][i];
			pred[i]=nextnode;
		}
		count++;
	}
	
	
	i=lastnode;
	if(i!=startnode)
	{
		printf("\nDistance of node%d=%d",i,distance[i]);
		
		j=i;
		count1=0;
		do
		{
			j=pred[j];
			count1++;
		}while(j!=startnode);
		j=i;
		path[step][count1]=lastnode;
		for(int i=0;i<count1;i++)
		{
			path[step][count1-i-1]=pred[j];
			j=pred[j];
		}
		for(int i=0;i<count1+1;i++)
		{
			
			printf("\n%d ",path[step][i]);
			
		}
	}	
}

/*
*
* Function Name: travel
* Input: p1,p2
* Output: travel between two adjacent nodes
* Logic: uses graph logic to travel
* Example Call: travel(3,4)
*
*/

void travel(int p1, int p2)
{
	
	if (direction == clk)
	{
		if (( (p1==4 && p2==3)||(p1==6 && p2==5)||(p1==12 && p2==11)||(p1==14 && p2==13)  )&& (f==1))
		{
			right_turn_wls();
			f = 0;
			direction=clk;
		}
		else if (( (p1==4 && p2==5)||(p1==6 && p2==7)||(p1==12 && p2==13)||(p1==14 && p2==15)  )&&(f==1))
		{
			left_turn_wls();
			f = 0;
			direction=anclk;
		}
		else if (p1 == 10 || p1 == 8 || p1 == 2 || p1 == 0)
		{

			right_turn_wls();
		}
		else if(p2-p1==1)
		{
			direction=anclk;
			turn_180();
		}
		else if (abs(p1 - p2) > 1)
		{
			right_turn_wls();
			f=1;
			
		}
		
	}
	else if (direction == anclk)
	{
		if (( (p1==4 && p2==5)||(p1==6 && p2==7)||(p1==12 && p2==13)||(p1==14 && p2==15)  )&&(f==1))
		{
			left_turn_wls();
		
			f = 0;
			direction=anclk;
		}
		else if (( (p1==4 && p2==3)||(p1==6 && p2==5)||(p1==12 && p2==11)||(p1==14 && p2==13)  )&&(f==1))
		{
			right_turn_wls();

			f = 0;
			direction=clk;
		}
		else if (p1 == 10 || p1 == 8 || p1 == 2 || p1 == 0)
		{
			left_turn_wls();
		}
		else if(p1-p2==1)
		{
			direction=clk;
			turn_180();
		}
		else if (abs(p1 - p2) > 1 && p2!=0)
		{
			left_turn_wls();
			f = 1;
		}
		
	}
	if ((p1 == 14 && p2 == 4) || (p1 == 4 && p2 == 14))
	{
		wall_follower();
	}
	else if((p1 == 12 && p2 == 6) || (p1 == 6 && p2 == 12))
	{
		forward_wls_zz(1,150);
	}
	else if (( p1 == 8 && p2 == 9)||( p1 == 9 && p2 == 10)|| (p1 == 10 && p2 == 9)|| (p1 == 9 && p2 == 8))
	{
		follow_white_line(1,150,p1);
	}
	
	else if((p1==7 && p2==8) || (p1==10 && p2==11) || p2==0 || p2==2 )
	{
		forward_wls(1,120);
	}
	else
	{
		forward_wls(1, 150);
	}

}
/*
*
* Function Name: follow_white_line
* Input: nodes to be crossed ,speed
* Output: follow line in inverted region
* Logic: uses inverted logic of black line following
* Example Call: follow_white_line(1,150)
*
*/
bool isWhite_node()
{
	while(1)
	{
		read_wls();
		if(wls_left>100 && wls_middle<100 && wls_right>100)
			turn_right();
		else if(wls_left<100 && wls_middle<100 && wls_right<100)
			turn_left();
		else if(wls_left<100 && wls_middle<100 && wls_right<100)
			return TRUE;
		else
			return FALSE;
	}
	
}
/*
*
* Function Name: follow_white_line
* Input: nodes to be crossed ,speed
* Output: follow line in inverted region
* Logic: uses inverted logic of black line following
* Example Call: follow_white_line(1,150)
*
*/
void follow_white_line(int node,int speed,int point)
{
	
		if(point==9)
		{
			while (1)
			{
				read_wls();
				
				
				
				//NODE DETECTED
				//all white
				if (wls_middle < 150 && wls_left < 150 && wls_right < 150)
				{
					all_sensor_white();
					read_wls();
					
					forward_wls(1,120);
					break;
					
					
				}
				
				else if (wls_middle < 150 && (wls_left < 150 || wls_right < 150))
				{
					velocity(120,120);
					
					
					if(wls_left < 150)
					turn_right();
					else if (wls_right < 150)
					turn_left();
					_delay_ms(5);
					stopmm();
				}
				
				
				else if (wls_middle > 150 && (wls_left < 150 || wls_right< 150))
				{
					velocity(120,120);
					if(wls_middle>150 && wls_left<150 && wls_right<150)
					{
						forward_wls(1,120);
						break;
					}
					
					else if(wls_left < 150)
					{
						soft_left();
						_delay_ms(25);
					}
					else if(wls_right < 150)
					{
						soft_right();
						_delay_ms(25);
					}
				}
				else
				{
					velocity(150,132);
					forward();
				}
				
				

			}
		}
		//if point is 8 or 10
		
		else
		{
			
			stopmm();
			read_wls();
			wls_print();
			_delay_ms(100);
			stopmm();
			while(wls_middle>100)
			{
				read_wls();
				if(point==10)
				soft_right();
				else
				soft_left();
				if(wls_middle<150)
				{
					stopmm();
					break;
				}
			}
			stopmm();
			wls_print();
			_delay_ms(500);
			while (1)
			{
				read_wls();
				//NODE DETECTED
	
				if (wls_middle < 150 && (wls_left < 150 || wls_right < 150))
				{
					stopmm();
					break;
				}
				else if (wls_middle > 150 && (wls_left > 150 || wls_right> 150))
				{
					velocity(120,120);
					
					
					if(wls_left < 150)
					{
						soft_left();
						_delay_ms(25);
					}
					else if(wls_right < 150)
					{
						soft_right();
						_delay_ms(25);
					}
				}
				else
				{
					velocity(150,132);
					forward();
				}
			}
			
			left=0;
			right=0;
			while(left<120 && right<120)
			{
				velocity(150,132);
				forward();
			}
			stopmm();
			wls_print();
			_delay_ms(100);
			LCD_Clear();
			
			
			while(wls_middle>150)
			{
				velocity(100,100);
				if(wls_left <190 || wls_left< wls_right)
				turn_left();
				else if(wls_right<190 || wls_right< wls_left )
				turn_right();
				_delay_ms(20);
				read_wls();
			}
			stopmm();
		}

	}
/*
*
* Function Name: all_sensor_white
* Input: no
* Output: checks un necesaary all white condition
* Logic: check surrounding values with sensor
* Example Call: all_sensor_white
*
*/
void all_sensor_white()
{
	stopmm();
	_delay_ms(100);
	int time_passed = 0;
	int max_time = 1;
	int i = 0;
	velocity(110,110);



	while (wls_middle <50)
	{
		max_time *= 4;
		if (new_state == 'r')
		{
			while ((time_passed < max_time) && (wls_middle <50))
			{

				turn_left();
				_delay_ms(100);
				stopmm();
				_delay_ms(15);
				time_passed++;
				read_wls();
			}
			stopmm();
			_delay_ms(10);
			time_passed = 0;
			read_wls();
			if (wls_middle >100)
			{
				break;
			}
			new_state = 'l';
			continue;
		}


		if (new_state == 'l')
		{
			while ((time_passed < max_time) && (wls_middle <50))
			{
				

				turn_right();
				_delay_ms(100);
				stopmm();
				_delay_ms(15);
				time_passed++;
				read_wls();
			}
			stopmm();
			_delay_ms(10);
			time_passed = 0; 
			read_wls();

			if (wls_middle >100)
			{
				break;
			}
			new_state = 'r';

		}

	}

	stopmm();
	_delay_ms(50);
}

/*
*
* Function Name: forward_wls
* Input: nodes to be crossed ,speed
* Output: follow line
* Logic: move straight with the help of white line sensors
* Example Call: forward_wls(1,150)
*
*/

void forward_wls(int node,int speed)
{
	int i=0,node_covered=0;
	read_wls();
	while (wls_middle > 100 && wls_left > 100 && wls_right > 100)
	{
		velocity(speed,speed*135/150);
		forward();
		read_wls();
		_delay_ms(90);      
		
	}
	velocity(speed,speed*135/150);
	forward();
	while (1)
	{
		read_wls();
		//NODE DETECTED
		if (wls_middle > 100 && wls_left > 100 && wls_right > 100)
		{
			velocity(speed,speed*135/150);
			backward();
			_delay_ms(30);
			stopmm();
			_delay_ms(20);
			node_covered++;
			i=0;
			
			
			if (node_covered == node)
			{
				left=0;
				right=0;
				break;
			}
			
			
			while (wls_middle > 100 && wls_left > 100 && wls_right > 100)
			{
				velocity(speed,speed*135/150);
				forward();
				_delay_ms(30);
				read_wls();
			}
			
			stopmm();
			_delay_ms(250);
		}
		else if (wls_middle > 100 && (wls_left > 100 || wls_right > 100))
		{
			velocity(120,120);
			if(i==3)
			{
				backward();
				_delay_ms(15);
			}
			
			if(wls_left < 100)
			turn_right();
			else if (wls_right < 100)
			turn_left();
			_delay_ms(5);
			stopmm();
			i++;
		}
		else if (wls_middle < 100 && (wls_left > 100 || wls_right> 100))
		{
			velocity(120,120);
			if(wls_left > 100)
			{
				soft_left();
				_delay_ms(50);
			}
			else if(wls_right > 100)
			{
				soft_right();
				_delay_ms(50);
			}
		}
		else
		{
			velocity(speed,speed*135/150);
			forward();
		}
	}
}
/*
*
* Function Name: line_follower
* Input: no
* Output: follow line in any loop
* Logic: follow line with the help of white line sensors
* Example Call: line_follower()
*
*/
void line_follower()
{
	int i=0,node_covered=0;
	read_wls();
	velocity(150,135);
	forward();
	ir();
	while (1)
	{
		read_wls();
		if (wls_middle < 30 && wls_left < 30 && wls_right < 30)
		{
			ir();
			if(ir_left<80)
			{
				forward();
				_delay_ms(10);
				continue;
			}
			
			velocity(150,132);
			stopmm();
			soft_left();
			_delay_ms(270);
			break;
		}
		else if (wls_middle > 100 && (wls_left > 100 || wls_right > 100))
		{
			velocity(120,120);
			if(i==3)
			{
				backward();
				_delay_ms(15);
			}
			
			if(wls_left < 100)
			turn_right();
			else if (wls_right < 100)
			turn_left();
			_delay_ms(5);
			stopmm();
			i++;
		}
		else if (wls_middle < 100 && (wls_left > 100 || wls_right> 100))
		{
			
			
			velocity(120,120);
			if(wls_left > 100)
			{
				soft_left();
				_delay_ms(50);
			}
			else if(wls_right > 100)
			{
				soft_right();
				_delay_ms(50);
			}
		}
		else
		{
			velocity(150,132);
			forward();
		}
		
	}
}
/*
*
* Function Name: forward_wls_zz
* Input: node,speed
* Output: follow zig zag line
* Logic: check surrounding values with sensor and use all_sensors_white()
* Example Call: forward_wls_zz(1,120)
*
*/

void forward_wls_zz(int node,int speed)
{
	int i=0,node_covered=0;
	read_wls();
	while (wls_middle > 100 && wls_left > 100 && wls_right > 100)
	{
		velocity(150,132);
		forward();
		read_wls();
		
	}
	velocity(150,135);
	forward();
	while (1)
	{
		read_wls();
		if (wls_middle < 50 && wls_left < 50 && wls_right < 50)
		{
			all_sensor_white();
		}
		//NODE DETECTED
		else if (wls_middle > 100 && wls_left > 100 && wls_right > 100)
		{
			velocity(150,132);
			backward();
			_delay_ms(30);
			stopmm();
			_delay_ms(20);
			node_covered++;
			i=0;
			if (node_covered == node)
			{
				break;
			}
			while (wls_middle > 100 && wls_left > 100 && wls_right > 100)
			{
				velocity(150, 132);
				forward();
				_delay_ms(30);
				read_wls();
			}
			
			stopmm();
			_delay_ms(250);
		}
		else if (wls_middle > 100 && (wls_left > 100 || wls_right > 100))
		{
			velocity(120,120);
			if(i==3)
			{
				backward();
				_delay_ms(15);
			}
			if(wls_left < 100)
			turn_right();
			else if (wls_right < 100)
			turn_left();
			_delay_ms(5);
			stopmm();
			i++;
		}
		else if (wls_middle < 100 && (wls_left > 100 || wls_right> 100))
		{
			
			
			velocity(120,120);
			if(wls_left > 100)
			{
				soft_left();
				_delay_ms(50);
			}
			else if(wls_right > 100)
			{
				soft_right();
				_delay_ms(50);
			}
		}
		else
		{
			velocity(150,132);
			forward();
		}
		
		

	}
}

/*
*
* Function Name: left_turn_wls
* Input: no
* Output: takes 90 degree left turn
* Logic: takes turn until middle sensor reads black again
* Example Call: left_turn_wls()
*
*/
void left_turn_wls(void)
{
	stopmm();
	bool from_node=FALSE;
	if (left<70 && right<70)
	{
		from_node=TRUE;
		left=0;
		right=0;
		while(left<80 && right<80)
		{
			velocity(150,135);
			forward();
		}
		stopmm();
	}
	
	right=0;
	while(right<150)
	{
		velocity(130,130);
		if(from_node)
			soft_left();
		else
			turn_left();
	}
	stopmm();
	read_wls();
	while (wls_middle < 130)
	{
		velocity(110,110);
		soft_left();

		read_wls();
	}
	stopmm();
}
/*
*
* Function Name: right_turn_wls
* Input: no
* Output: takes 90 degree right turn
* Logic: takes turn until middle sensor reads black again
* Example Call: right_turn_wls()
*
*/
void right_turn_wls(void)
{
	bool from_node=FALSE;
if (left<70 && right<70)
{
	from_node=TRUE;
	left=0;
	right=0;
	while(left<80 && right<80)
	{
		velocity(150,135);
		forward();
	}
	stopmm();
}

left=0;
while(left<150)
{
	velocity(130,130);
	if(from_node)
	 soft_right();
	else
	 turn_right();
}
stopmm();
read_wls();

	while (wls_middle < 130)
	{
		velocity(115,115);
		soft_right();

		read_wls();

	}
	stopmm();
}

/*
*
* Function Name: buzzer
* Input: no
* Output: buzzer sounds
* Logic: frequency of digital signals
* Example Call: buzzer()
*
*/
void buzzer()
{
	DDRD=0xF0;
	for(int i=0;i<25000;i++)       //5 seconds 25500
	{
		PORTD=0b10000000;
		_delay_us(100);
		PORTD=0b00000000;
		_delay_us(100);
	}
	DDRD=0x00;
	
}
/*
*
* Function Name: turn_180
* Input: no
* Output: takes 180 degree turn
* Logic: takes turn until middle sensor reads black two times
* Example Call: turn_180()
*
*/
void turn_180()
{
	left=0;
	right=0;
	velocity(120,120);
	while (left<=150 && right<=150)
	{
		forward();
	}
		
	velocity(120,120);
	turn_right();
	_delay_ms(1500);
	stopmm();
	read_wls();
	while(wls_right<100)
	{
		velocity(100,100);
		turn_right();
		read_wls();
	}
	stopmm();
}

///////////////////////////////////////////////////////////////////////
//                       for arm                                     //
//////////////////////////////////////////////////////////////////////
/*
*
* Function Name: servox     where x=0,00,01,1,2,3,4
* Input: no
* Output: servo moves between two positions
* Logic: PWM signal is generated
* Example Call: servo1(100,150)
*
*/
void servo0(int initial,int final)
{
	if(initial<=525 && final<=525)
	{
		servo01(initial,final);
	}
	else if(initial<=525 && final>525)
	{
		servo01(initial,525);
		servo00(97,final-525);
		
	}
	else if(initial>=525 && final<525)
	{
		servo00(initial-525,97);
		servo01(525,final);	
	}
	else if(initial>=525 && final>=525)
	{
		servo00(initial-525,final-525);
	}
}
void servo00(int initial,int final)
{
	if(initial<final)
	{
		for(int i=initial;i<=final;i++)
		{
			OCR3A = i;
			_delay_ms(4);
		}
	}
	else if(final<initial)
	{
		for(int i=initial;i>=final;i--)
		{
			OCR3A = i;
			_delay_ms(4);
		}
	}
}

void servo01(int initial,int final)
{
	if(initial<final)
	{
		for(int i=initial;i<=final;i++)
		{
			OCR3B = i;
			_delay_ms(4);
		}
	}
	else if(final<initial)
	{
		for(int i=initial;i>=final;i--)
		{
			OCR3B = i;
			_delay_ms(4);
		}
	}
	
}
void servo1(int initial,int final)
{
	if(initial<final)
	{
		for(int i=initial;i<=final;i++)
		{
			OCR4A = i;
			_delay_ms(10);
		}
	}
	else if(final<initial)
	{
		for(int i=initial;i>=final;i--)
		{
			OCR4A = i;
			_delay_ms(10);
		}
	}
	
}
void servo2(int initial,int final)
{
	if(initial<final)
	{
		for(int i=initial;i<=final;i++)
		{
			OCR4B = i;
			_delay_ms(10);
		}
	}
	else if(final<initial)
	{
		for(int i=initial;i>=final;i--)
		{
			OCR4B = i;
			_delay_ms(10);
		}
	}
}
void servo3(int initial,int final)
{
	if(initial<final)
	{
		for(int i=initial;i<=final;i++)
		{
			OCR1B = i;
			_delay_ms(10);
		}
	}
	else if(final<initial)
	{
		for(int i=initial;i>=final;i--)
		{
			OCR1B = i;
			_delay_ms(10);
		}
	}
}
void servo4(int initial,int final)
{
	if(initial<final)
	{
		for(int i=initial;i<=final;i++)
		{
			OCR1A = i;
			_delay_ms(10);
		}
	}
	else if(final<initial)
	{
		for(int i=initial;i>=final;i--)
		{
			OCR1A = i;
			_delay_ms(10);
		}
	}
}
/////////////////////////////////////////////////////////////////////

/*
*
* Function Name: velocity
* Input: no
* Output: gives velocity to motors
* Logic: PWM signal is generated
* Example Call: velocity(150,150)
*
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/*
*
* Function Name: read_wls
* Input: no
* Output: reads white line sensor values
* Logic: ADC_Conversion
* Example Call: read_wls()
*
*/
void read_wls(void)
{
	ADCSRA=0x87;
	ADCSRB=0x00;
	ADMUX=0xA0;
	ADMUX = 0xA0;
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	while(ADCSRA & (1<<ADSC)){}
	wls_left=ADCH;
	ADMUX = 0xA1;
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	while(ADCSRA & (1<<ADSC)){}
	wls_middle=ADCH;
	
	ADMUX = 0xA2;
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	while(ADCSRA & (1<<ADSC)){}
	wls_right=ADCH;
}

void stopmm()
{
	PORTD =0b00000000;
}
void forward()
{
	PORTD =0b00000101;
}
void backward()
{
	PORTD =0b00001010;
}
void turn_right()
{
	PORTD =0b00001001;
}
void turn_left()
{
	PORTD =0b00000110;
}
void soft_right()
{
	PORTD =0b00000001;
}
void soft_left()
{
	PORTD =0b00000100;
}

/*
*
* Function Name: pick_bot
* Input: no
* Output: pick the material from bottom
* Logic: arm mechanism
* Example Call: pick_bot()
*
*/
void pick_bot()
{
	servo2(450,355);
	servo1(370,110);
	servo2(355,260);
	servo1(110,100);
	servo4(300,150);
	servo1(100,270);
	servo2(260,355);
	servo1(270,370);
	servo2(355,450);
	
}
 /*
 *
 * Function Name:  place_lr
 * Input: no
 * Output: place the material in low rise house
 * Logic: arm mechanism
 * Example Call:  place_lr()
 *
 */
void place_lr()
{
	servo2(450,335);
	servo1(370,160);
	servo4(150,300);
	servo1(160,270);
	servo2(335,355);
	servo1(270,370);
	servo2(355,450);
	
}

/*
 *
 * Function Name:  place_hr
 * Input: no
 * Output: place the material in high rise house
 * Logic: arm mechanism
 * Example Call:  place_hr()
 *
 */
void place_hr()
{
	servo2(450,360);
	servo1(370,230);
	servo4(150,300);
	servo1(230,365);
	servo2(360,450);
	
	
}
/*
 *
 * Function Name:  place_in
 * Input: d >> direction of bot movement
          pi >> the material which is picked up from the ground
		  pl >> the position on the bot where the picked material is to be placed
 * Output: place the material on the bot
 * Logic: arm mechanism
 * Example Call:  place_in (anclk, 5, 3)
 *
 */
void place_in(int d, int pi, int pl)               
{
	
	int a;
	if(d==clk)
	{
		if(pi==5)
		a=316;
		else if(pi==6)
		a=860;
	}
	else if(d==anclk)
	{
		if(pi==5)
		a=860;
		else if(pi==6)
		a=316;
	}
	
	
	if(a==316)
	{
		if(pl==1)
		{
			servo2(450,420);
			servo0(a,780);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(780,97);
		}
		else if(pl==2)
		{
			servo2(450,420);
			servo0(a,1000);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(1000,97);
		}
		else if(pl==3)
		{
			servo2(450,420);
			servo0(a,243);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(243,97);
		}
		else if(pl==4)
		{
			servo2(450,420);
			servo0(a,460);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(460,97);
		}
		
	}
	else if(a==860)
	{
		if(pl==1)
		{
			servo2(450,420);
			servo0(a,780);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(780,97);
			
		}
		else if(pl==2)
		{
			servo2(450,420);
			servo0(a,1000);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(1000,97);
			
		}
		else if(pl==3)
		{
			servo2(450,420);
			servo0(a,243);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(243,97);
		}
		else if(pl==4)
		{
			servo2(450,420);
			servo0(a,460);
			servo2(420,450);
			servo3(340,370);
			servo1(370, 250);
			servo4(150, 300);
			servo1(250,370);
			servo3(370,340);
			servo0(460,97);
		}
	}
}
/*
 *
 * Function Name:  pick_from
 * Input: d >> direction of bot movement
          a >> the material position on the bot to be picked up
		  p >> the position on the ground where the picked material is to be placed
 * Output: pick the material from the bot
 * Logic: arm mechanism
 * Example Call:  pick_from(anclk, 3, 5)
 *
 */
void pick_from(int d, int a, int p)
{
	switch(a)
	{
		case 1:
		{
			servo2(450,420);
			
			servo0(97,780);
			servo2(420,450);
			servo3(316,370);
			servo1(370, 250);
			servo4(300, 150);
			servo2(450,420);
			servo1(250,370);
			servo3(370,316);
			
			if(d==clk)
			{
				if((p==5) || (p==6))
				{
					servo0(780,316);
				}
				
				else if((p==7) || (p==10))
				{
					servo0(780,860);
				}
				else if((p==8) || (p==9))
				{
					servo0(780,97);
				}
			}
			else if(d==anclk)
			{
				if((p==5) || (p==6))
				{
					servo0(780,860);
				}
				
				else if((p==7) || (p==10))
				{
					servo0(780,316);
				}
				else if((p==8) || (p==9))
				{
					servo0(780,97);
				}
				
			}
			servo2(420,450);
			break;
		}
		case 2:
		{
			servo2(450,420);
			servo0(97,1000);
			servo2(420,450);
			servo3(316,370);
			servo1(370, 250);
			servo4(300, 150);
			servo2(450,420);
			servo1(250,370);
			servo3(370,316);
			if(d==clk)
			{
				if(p==5 || p==6)
				{
					servo0(1000,316);
					//stepper(stepac, 1000);
				}
				
				else if((p==7) || (p==10))
				{
					servo0(1000,860);
				}
				else if(p==8 || p==9)
				{
					servo0(1000,97);
				}
			}
			else if(d==anclk)
			{
				if(p==5 || p==6)
				{
					servo0(1000,860);
				}
				else if((p==7) || (p==10))
				{
					servo0(1000,316);
				}
				else if(p==8 || p==9)
				{
					servo0(1000,97);
				}
				
			}
			servo2(420,450);
			break;
		}
		case 3:
		{
			servo2(450,420);
			servo0(97,243);
			servo2(420,450);
			servo3(316,370);
			servo1(370, 250);
			servo4(300, 150);
			servo2(450,420);
			servo1(250,370);
			servo3(370,316);
			if(d==clk)
			{
				if(p==5 || p==6)
				{
					servo0(243,316);
				}
				
				else if((p==7) || (p==10))
				{
					servo0(243,860);
				}
				else if(p==8 || p==9)
				{
					servo0(243,97);
				}
			}
			else if(d==anclk)
			{
				if(p==5 || p==6)
				{
					servo0(243,860);
				}
				else if((p==7) || (p==10))
				{
					servo0(243,316);
				}
				else if(p==8 || p==9)
				{
					servo0(243,97);
				}
				
			}
			servo2(420,450);
			break;
		}
		case 4:
		{
			servo2(450,420);
			servo0(97,460);
			servo2(420,450);
			servo3(316,370);
			servo1(370, 250);
			servo4(300, 150);
			servo2(450,420);
			servo1(250,370);
			servo3(370,316);
			if(d==clk)
			{
				if(p==5 || p==6)
				{
					servo0(460,316);
				}
				
				else if((p==7) || (p==10))
				{
					servo0(460,860);
				}
				else if(p==8 || p==9)
				{
					servo0(460,97);
				}
			}
			else if(d==anclk)
			{
				if(p==5 || p==6)
				{
					servo0(460,860);
				}
				else if((p==7) || (p==10))
				{
					servo0(460,316);
	
				}
				else if(p==8 || p==9)
				{
					servo0(460,97);
				}
				
			}
		}
		servo2(420,450);
		break;
	}
	
}

/*
 *
 * Function Name:  papom
 * Input: dir >> direction of bot movement
          pick >> picking postion of one material
		  place >> place position of material
 * Output: pick and place of a material
 * Logic: arm mechanism
 * Example Call:  pick_from(anclk, 3, 5)
 *
 */

void papom(int pick, int place,int node)
{
	wls_print();
	LCD_Clear();
	
	if(node!=11)
	{
	
		if(left<70 && right<70)
		{
			left=0;
			right=0;
			while(left<170 && right<170)
			{
				velocity(150,132);
				forward();
			}
		}
		
		
	
        read_wls();
		all_sensor_white();
		
	}
	stopmm();
	int i;
	//direction ==1, clockise        direction ==0, anticlockwise
	//pick 1,2,3,4-on the bot, 5-outside, 6-inside
	//place1,2,3,4-on the bot, 5-outside low rise, 6- outside high rise , 7-inside, 8-front low rise, 9-front high rise
	if(direction==clk)
	{
		if(pick>0 )
		{
			switch(pick)
			{
				case 1:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 2:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 3:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 4:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 5:
				{
					servo0(97,316);
					pick_bot();
					break;
					
				}
				case 6:
				{
					servo0(97,860);
					pick_bot();
					break;
				}
			}
			
		}
		
		if(place>0)
		{
			switch(place)
			{
				case 1:
				{
					place_in(direction,pick, place);
					break;
				}
				case 2:
				{
					place_in(direction,pick, place);
					break;
				}
				case 3:
				{
					place_in(direction,pick, place);
					break;
				}
				case 4:
				{
					place_in(direction,pick, place);
					break;
				}
				case 5:
				{
					place_lr();
					
					servo0(316,97);
					
					break;
				}
				case 6:
				{
					place_hr();
					servo0(316,97);
					break;
				}
				case 7:
				{
					servo2(450,335);
					servo1(370,180);
					servo4(150,300);
					servo1(180,370);
					servo2(335,450);
					servo0(860,97);
					
					break;
				}
				case 8:
				{
					place_lr();
					break;
				}
				case 9:
				{
					place_hr();
					break;
				}
				case 10:
				{
					place_hr();
					servo0(860,97);
					break;
				}
			}
		}
		
	}
	else if(direction==anclk)
	{
		if(pick>0 )
		{
			switch(pick)
			{
				case 1:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 2:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 3:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 4:
				{
					pick_from(direction,pick,place);
					break;
				}
				case 5:
				{
					servo0(97,860);
					pick_bot();
					break;
					
				}
				case 6:
				{
					servo0(97,316);
					pick_bot();
					break;
				}
			}
			
		}
		
		if(place>0)
		{
			switch(place)
			{
				case 1:
				{
					place_in(direction,pick, place);
					break;
				}
				case 2:
				{
					place_in(direction,pick, place);
					break;
				}
				case 3:
				{
					place_in(direction,pick, place);
					break;
				}
				case 4:
				{
					place_in(direction,pick, place);
					break;
				}
				case 5:
				{
					place_lr();
					servo0(860,97);
					
					break; 
				}
				case 6:
				{
					place_hr();
					servo0(860,97);
					break;
				}
				case 7:
				{
					servo2(450,335);
					servo1(370,190);
					servo4(150,300);
					servo1(190,370);
					servo2(335,450);
					servo0(316,97);
					//stepper(stepac,800);
					break;
				}
				case 8:
				{
					place_lr();
					break;
				}
				case 9:
				{
					place_hr();
					break;
				}
				case 10:
				{
					
					place_hr();
					servo0(316,97);
					break;
				}
			}
		}
		
	}
	
	
}
/*
 *
 * Function Name:  pop
 * Input: val >> pick(1,2) or place(0)
          node_val >> value of node
 * Output: void
 * Logic: 
 * Example Call:  pick_from(1,7)
 *
 */

void pop(int val,int node_value)
{
	//0 9
	int i;
	//pick 
	if(val>0)
	{ 
		//which place is empty in bot for CM to place
		for(i=0;i<4;i++)
		{
			
			if(CM_place_in_bot[i]==0)
				break;
		}
		// 5 out 6 in
		if(mat_remain[node_value-1])
		{
			papom(5,i+1,node_value);
			CM_place_in_bot[i]=node_value;
			mat_remain[node_value-1]--;
		}
		else
		{
		  papom(6,i+1,node_value);
		  CM_place_in_bot[i]=node_value;
		  
		}
	}
	//place
	else
	{
		//place1,2,3,4-on the bot, 5-outside low rise, 6- outside high rise , 7-inside low, 8-front low rise, 9-front high rise,10-inside high
		int a=0;
		for(i=0;i<4;i++)
		{
			int place;
			if(mat_req_for_house[node_value-7]>0)
			{
				if(CM_place_in_bot[i]==CM[2*(node_value-7)+a])
				{
					int state;
					//in front facing
					if(f==1)
					{
						//high rise
					  if(high_or_low[node_value-7])
						place=9;
						//low rise
					  else
					   place=8;
					}
					else
					{
						//h5
						if(node_value==11)
						{
							if(high_or_low[node_value-7])
								place=10;
							else
								place=7;
						}
							
							//other houses
						else
						{
							if(high_or_low[node_value-7])
								place=6;
							else
								place=5;
						}
					}

					papom(i+1,place,node_value);
					CM_place_in_bot[i]=0;
					i=-1;
					a++;
					mat_req_for_house[node_value-7]--;
					
				}
			}
			else
				break;
			
		}
	}
	
}


	
/*
 *
 * Function Name:  wall_follower
 * Input: no
 * Output: follows wall
 * Logic: try to keep the bot at a fixed distance from the wall
 * Example Call:  wall_follower()
 *
 */
void wall_follower()
{
	line_follower();
	velocity(130,130);
	ir();
	
	while(ir_left>55)
	{
		
		ir();
		read_wls();
		if(wls_left>100 || wls_middle>100 || wls_right>100)
		{
			break;
		}
		if(ir_left<90)
		{
			soft_left();
		}
		else if(ir_left>95)
		{
			soft_right();
		}
		else
		{
			forward();
		}
		_delay_ms(100);
	}
	
	forward_wls(1,150);
}

/*
 *
 * Function Name:  ir
 * Input: no
 * Output: reads sharp ir values
 * Logic: ADC_conversion
 * Example Call:  ir()
 *
 */
void ir()
{
	ADMUX = 0xA4;
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	while(ADCSRA & (1<<ADSC)){}
	ir_right=ADCH;
	ADMUX = 0xA5;
	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	while(ADCSRA & (1<<ADSC)){}
	ir_left=ADCH;
}



int main(void)
{
	_delay_ms(2000);
 	unsigned char c[10];
	all_init();

	task5();
	buzzer();

	return 0;
}

void task5()
{
	f=0;
	direction = clk;
	make_path();
	unsigned char c[10];
	//array to store node value of CM  and house
	//					 M1  M2 M3  M4 M5  M6 H1  H2 H3  H4 H5
	int node_value[11]={ 15, 3, 13, 5, 11, 7, 14, 4, 12, 6, 9};
	
	
	forward_wls(1,150);
	if(node_value[or_path[0]-1]<9)
	{
		direction=anclk;
		left_turn_wls();
	}
	else if(node_value[or_path[0]-1]>9)
	{
		direction=clk;
		right_turn_wls();
	}
	LCD_String_xy(0,0,"start");
	itoa(node_value[or_path[0]-1], 7, 10);
	
	LCD_String_xy(0,4,c);
	
	_delay_ms(50);

	//travel from start to or_path[0]
	shortest_path(1,node_value[or_path[0]-1]);
	pop(1,or_path[0]);
	
	//first going to H5
	for(int i=0;i<4;i++)
	{
		LCD_Clear();
		itoa(node_value[or_path[i]-1], c, 10);
		
		LCD_String_xy(0,0,c);
		
		itoa(node_value[or_path[i+1]-1], c, 10);
		
		LCD_String_xy(0,3,c);
		LCD_String_xy(1,0,"ja rha");
		_delay_ms(50);

		

		shortest_path(node_value[or_path[i]-1],node_value[or_path[i+1]-1]);
		if(i==3)
		pop(0,or_path[i+1]);
		
		else
		pop(1,or_path[i+1]);
	}
	for(int i=4;i<14;i++)
	{
		
		LCD_Clear();
		itoa(node_value[or_path[i]-1], c, 10);
		
		LCD_String_xy(0,0,c);
		
		itoa(node_value[or_path[i+1]-1], c, 10);
		
		LCD_String_xy(0,2,c);
		_delay_ms(50);

		
		if(or_path[i+1]!=0)
		{
			shortest_path(node_value[or_path[i]-1],node_value[or_path[i+1]-1]);
			pop(i%3,or_path[i+1]);
		}
		else
		{
			i++;
			if(or_path[i+1]!=0)
			{
				shortest_path(node_value[or_path[i-1]-1],node_value[or_path[i+1]-1]);
				pop(i%3,or_path[i+1]);
			}
			else
			{
				i++;
				shortest_path(node_value[or_path[i-2]-1],node_value[or_path[i+1]-1]);
				pop(0,or_path[i+1]);
			}
		}
	}
	
	
	return 0;
}
