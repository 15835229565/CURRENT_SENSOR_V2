// Application specific variable declarations
#include <p18cxxx.h>
#include <usart.h>
#include <timers.h>
#include <eep.h>
#include <string.h>

#include "CAN18XX8.h"
#include "j1939_data.h"

#define BuffNO 16

void main (void);
void InterruptHandlerHigh (void);
void InterruptHandlerLow (void);

J1939_MESSAGE UartRx_Msg, CanRx_Msg;
J1939_SE_ID FlagAddres,EEAddres;

// CAN module related variables
enum CAN_RX_MSG_FLAGS RecFlags;
BYTE RxFilterMatch;
// Application specific initialization code follows
char tp_char;
BYTE UartRxCnt, UartTxCnt;
BYTE CanRxBuff, UartTxBuff;
BYTE UartSync = 0;
BYTE UTicTac = 0;

unsigned int current_ad_value;
unsigned int reference_ad_value;
float capacity=0;
int delta_ad[100];
float current=0;
char count=0;

typedef union combo {
	int Int;	   
	char Char[2];	   
} Tcombo;



void main(void)
{
    WDTCON = 0x00;


    
    //TRISA = 0xC0;
    TRISB = 0xFF;
    TRISC = 0xFF;

    UartTxCnt = 0;
    UartRxCnt = 0;

    CanRxBuff = 0;
    UartTxBuff = 0;

	// Initialize TIMER0
    INTCON2bits.TMR0IP = 0; //Timer0 INT-LOW
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_128);
	//  24Mhz/4/128 := 46875 -> 65536 - 46875 = 18661
    WriteTimer0(49911); // 1Sec interval at 8Mhz
    
/*    EEAddres.Bytes[0] = Read_b_eep(0);
    EEAddres.Bytes[1] = Read_b_eep(1);
    EEAddres.Bytes[2] = Read_b_eep(2);
    EEAddres.Bytes[3] = Read_b_eep(3);
    EEAddres.SE_ID = 0x20123456;
*/

	// Initialize CAN module with no message filtering
    // 8MHz Fosc 250Kb/s
    // 8MHz -> 2MHz @ 250Khz = 8Tq (1+2+3+2)
    CANInitialize(1 ,0x02 ,2 ,3 ,2 , CAN_CONFIG_VALID_XTD_MSG); //256kb at 8Mhz crystal
    /*CANSetOperationMode(CAN_OP_MODE_CONFIG);
    CANSetFilter(CAN_FILTER_B1_F1, Can_bootF.SE_ID, CAN_CONFIG_XTD_MSG);
    CANSetFilter(CAN_FILTER_B1_F2, Can_nodeF.SE_ID, CAN_CONFIG_XTD_MSG);
    CANSetMask(CAN_MASK_B1, Can_Mask1.SE_ID, CAN_CONFIG_XTD_MSG);
    CANSetFilter(CAN_FILTER_B2_F1, Can_addrLO.SE_ID, CAN_CONFIG_XTD_MSG);
    CANSetMask(CAN_MASK_B2, Can_Mask2.SE_ID, CAN_CONFIG_XTD_MSG);
	*/    
	CANSetOperationMode(CAN_OP_MODE_NORMAL);

	//ADC configuration
	//reference is connected to A0
	//voltage output connected to A1
	LATA = 0x00;
	PORTA=0;
	TRISA=0xFF; //A port as input
    ADCON1 = 0b00111101;//VSS,VDD ref. AN0 analog only
	//ADCON2 = 0b00001001;//ADCON2 setup: Left justified, Tacq=2Tad, Tad=2*Tosc (or Fosc/2)
	ADCON2 = 0b10101011;
	ADCON0bits.ADON = 0x01;//Enable A/D module
	
    INTCONbits.GIE = 1; //enable global interrupts
    Tcombo int2byte;
					UartRx_Msg.Data[4] = 1;//CanRx_Msg.Data[0];
	  				UartRx_Msg.Data[5] = 2;//CanRx_Msg.Data[0];
					UartRx_Msg.Data[6] = 3;//CanRx_Msg.Data[7];
	  				UartRx_Msg.Data[7] = 4;//CanRx_Msg.Data[7];

    while(1)
    {
        ClrWdt();
		/********************************/
		//cobtinuasly reading of current*/
		/********************************/		
		//ADC chanel A0	- voltage	
	    ADCON0bits.CHS0 = 0;//clear ADCON0 to select channel 0 (AN0)
	    ADCON0bits.CHS1 = 0;//clear ADCON0 to select channel 0 (AN0)
		ADCON0bits.CHS2 = 0;//clear ADCON0 to select channel 0 (AN0)
	    ADCON0bits.CHS3 = 0;//clear ADCON0 to select channel 0 (AN0)
		Delay100TCYx (2);
	    ADCON0bits.GO = 1;
	    while (ADCON0bits.GO);  //wait for conversion
		current_ad_value=ADRES;
	    ClrWdt();    
		
		Delay100TCYx (2);
		//ADC chanel A1 - rederence		    
		ADCON0bits.CHS0 = 1; //select channel 1 (AN1)
	    Delay100TCYx (2);
	    ADCON0bits.GO = 1;
	    while (ADCON0bits.GO);  //wait for conversion
		reference_ad_value= ADRES;
	 	
		//delta ADC = reference votlatge - real voltage
		//we filter it thru buffer
		count++;
		if(count>=100) count=0;
		delta_ad[count] = reference_ad_value-current_ad_value;
	
		if (UTicTac >= 1) {
		    UTicTac = 0;
            
			//current conversion
			//I=1.6V*200/1024*0.625  * delta_ad ==0.52083333333333333333333333333333 *delta_ad
			//reference votlatge - real voltage
			current=0;
			for(char i=0;i<100;i++){
				current+=delta_ad[i];
			}
			//current *= 0.0052083; //above formula /100

			current *= 0.00463541637; //above formula with correction factor of 0.89
		

			int2byte.Int = (int)current;
	  		UartRx_Msg.Data[0] = int2byte.Char[1];
	  		UartRx_Msg.Data[1] = int2byte.Char[0];

			//capcity
			capacity += (current/3600);	
			int2byte.Int=(int)capacity;
   			UartRx_Msg.Data[2] = int2byte.Char[1];
   			UartRx_Msg.Data[3] = int2byte.Char[0];

					
			// CAN adresses
		    UartRx_Msg.Address.SE_ID = 0x010000F2;
		    CANSendMessage(UartRx_Msg.Address.SE_ID, &UartRx_Msg.Data[0], 8, CAN_TX_PRIORITY_0 & CAN_TX_XTD_FRAME & CAN_TX_NO_RTR_FRAME);
			ClrWdt();

		}



        if (CANIsRxReady())                        // Check for CAN message
        {
            CANReceiveMessage(&CanRx_Msg.Address.SE_ID, &CanRx_Msg.Data[0], &CanRx_Msg.Length.Len, &RecFlags);
//            if ( RecFlags & CAN_RX_OVERFLOW )
//            {
//                                                 // Rx overflow occurred; handle it
//            }
            if ( RecFlags & CAN_RX_INVALID_MSG )
            {                                      // Invalid message received; handle it
            } 
            else
            {
               /* if ( RecFlags & CAN_RX_RTR_FRAME )
                {                                  // RTR frame received
                    //UartRx_Msg.Data[6]++;
			   		
					CanRxBuff++;
                    if (CanRxBuff >= BuffNO) 
                        CanRxBuff = 0; 
                }
                else
                {                                  // Regular frame received.
					//UartRx_Msg.Data[7]++;
			   		CanRxBuff++;
                    if (CanRxBuff >= BuffNO) 
                        CanRxBuff = 0; 

                    
                }*/
				    
					if(CanRx_Msg.Address.SE_ID==0x020000F2) capacity=(char)CanRx_Msg.Data[0];
					/*UartRx_Msg.Data[4] = CanRx_Msg.Data[0];
	  				UartRx_Msg.Data[5] = CanRx_Msg.Data[1];
					UartRx_Msg.Data[6] = CanRx_Msg.Data[6];
	  				UartRx_Msg.Data[7] = CanRx_Msg.Data[7];*/


            }
//            if ( RecFlags & CAN_RX_XTD_FRAME )
//            {
//                                                 // Extended Identifier received; handle it
//            }
//            else
//            {
//                                                 // Standard Identifier received.
//            }
//                                                 // Extract receiver filter match, if it is to be used
//            RxFilterMatch = RecFlags & CAN_RX_FILTER_BITS;
        }
    


                                             // Process received message
      /*  if ((BusyUSART() == 0) && (UartSync == 1))
        {
            if (UartTxCnt > 0)                   // Preveri èe posiljamo CAN telegram na UART
            {
                WriteUSART(CanRx_Msg[UartTxBuff].Array[UartTxCnt]);  // Pošlji znak
                UartTxCnt++;                     // poveèaj stevec za naslednji znak
                if ((UartTxCnt) >= (J1939_MSG_LENGTH + J1939_DATA_LENGTH)) // Preveri èe je zadni znak
                {
                    UartTxCnt = 0;               // Postavi na prvi znak
                    UartTxBuff++;                // Premakni na novi Buffer
                    if (UartTxBuff >= BuffNO)    // Preveri da nismo cez mejo
                        UartTxBuff =0;
                }
            }
            else
            {
                if (CanRxBuff != UartTxBuff)
                {
                    WriteUSART(CanRx_Msg[UartTxBuff].Array[UartTxCnt]);  // Pošlji ga na UART
                    UartTxCnt++;                // Poveèaj števec na naslednji znak
                }
            }
        }

        if (UartRxCnt >= (J1939_MSG_LENGTH + J1939_DATA_LENGTH)) // Ali imamo vse znake.
        {
            if (CANIsTxReady())                  // Preveri ali ja CAN prost
            {
//              void CANSendMessage(unsigned long id, BYTE *Data, BYTE DataLen enum CAN_TX_MSG_FLAGS MsgFlags);
                CANSendMessage(UartRx_Msg.Address.SE_ID, &UartRx_Msg.Data[0], 8, CAN_TX_PRIORITY_0 & CAN_TX_XTD_FRAME & CAN_TX_NO_RTR_FRAME);
                UartRxCnt = 0;                   // Pripravi za sprejem novega telegrama iz UART-a
                LATAbits.LATA5 = !LATAbits.LATA5;
            }
        }
        if DataRdyUSART()                        // Preveri ali v UART èaka nov znak
        {
            UTicTac = 0;
            tp_char = ReadUSART();               // Preber znak v polje
            if (UartSync)
            {
                UartRx_Msg.Array[UartRxCnt] = tp_char;
                UartRxCnt++;                     // in poveèaj stevec prebranih znakov iz UART-a
            }
            else
            {
                 if (tp_char == '#') 
                    UartSync = 1;
            }
        }

        if (UTicTac > 5)                         // Èe je števec èez mejo, telegram ni veljeven
        {
            UartRxCnt = 0;                       // Postavi na prvi znak v telegramu
            UTicTac = 0;
        }*/
    }                                            // Do this forever
}
//**********************************************************************************
// High priority interrupt vector
#pragma code InterruptVectorHigh=0x08
void InterruptVectorHigh(void)
{
      _asm
        goto InterruptHandlerHigh  //jump to interrupt routine
      _endasm
}

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh()
{
    unsigned int t0;
    if(PIR1bits.TMR1IF)
    {
        PIR1bits.TMR1IF=0;                  //clear interrupt flag
    }
    if(INTCONbits.TMR0IF)                   //check for TMR0 overflow interrupt flag
    {
        INTCONbits.TMR0IF = 0;              //clear interrupt flag
        t0=0;//ReadTimer0();
        WriteTimer0(t0 +49911);     //18661        //Nastavi na 1.0 Sek
        UTicTac++;                          // Povecaj idle števec
        LATAbits.LATA3 = !LATAbits.LATA3;
    }
}

//----------------------------------------------------------------------------
// Low priority interrupt vector
#pragma code InterruptVectorLow=0x18
void InterruptVectorLow (void)
{
      _asm
          GOTO InterruptHandlerLow  //jump to interrupt routine
      _endasm
}

#pragma code
#pragma interrupt InterruptHandlerLow //save=disp0,disp1,disp2,disp3,TBLPTRU

void InterruptHandlerLow (void)
{
    unsigned int t0;
    if(INTCONbits.TMR0IF)                   //check for TMR0 overflow interrupt flag
    {
        INTCONbits.TMR0IF = 0;              //clear interrupt flag
        t0=ReadTimer0();
        WriteTimer0(t0 +18661);             //Nastavi na 1.0 Sek
        UTicTac++;                          // Povecaj idle števec
        LATAbits.LATA3 = !LATAbits.LATA3;
    }
}
// End of program


