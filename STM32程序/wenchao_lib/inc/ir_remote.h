#ifndef _IR_H_
#define _IR_H_


sbit IRIN=P3^3;

unsigned char IrValue[6];
unsigned char Time;
unsigned char ir_key_value = 99;

/*******************************************************************************
* ? ? ?         : delay_ir
* ????		   : ????,i=1?,????10us
*******************************************************************************/
void delay_ir(unsigned int i){
	while(i--);	
}


unsigned char get_ir_key(unsigned char ir_v){
	unsigned char ir_key = 0;
	switch( ir_v ){
		case 22 : ir_key = 0; break;
		case 12 : ir_key = 1; break;
		case 24 : ir_key = 2; break;
		case 94 : ir_key = 3; break;
		case  8 : ir_key = 4; break;
		case 28 : ir_key = 5; break;
		case 90 : ir_key = 6; break;
		case 66 : ir_key = 7; break;
		case 82 : ir_key = 8; break;
		case 74 : ir_key = 9; break;
		case 13 : ir_key = 10; break;
		case 21 : ir_key = 11; break;
		case 25 : ir_key = 12; break;
		case 64 : ir_key = 13; break;
		case  7 : ir_key = 14; break;
		case  9 : ir_key = 15; break;
	}
	return ir_key;
}


void ir_decode(){
	unsigned char j,k;
	unsigned int err;
	Time=0;					 
	delay_ir(700);	//7ms
	if(IRIN==0)		//??????????????
	{	 
		ir_key_value = 99;  ///////////////???
		err=1000;				//1000*10us=10ms,????????????
		/*???????????,????????????????,????????
		?,??????*/	
		while((IRIN==0)&&(err>0))	//????9ms??????  		
		{			
			delay_ir(1);
			err--;
		} 
		if(IRIN==1)			//??????9ms???
		{
			err=500;
			while((IRIN==1)&&(err>0))		 //??4.5ms????????
			{
				delay_ir(1);
				err--;
			}
			for(k=0;k<4;k++)		//??4???
			{				
				for(j=0;j<8;j++)	//??????
				{

					err=60;		
					while((IRIN==0)&&(err>0))//???????560us?????
					{
						delay_ir(1);
						err--;
					}
					err=500;
					while((IRIN==1)&&(err>0))	 //???????????
					{
						delay_ir(10);	 //0.1ms
						Time++;
						err--;
						if(Time>30)
						{
							return;
						}
					}
					IrValue[k]>>=1;	 //k???????
					if(Time>=8)			//?????????565us,???1
					{
						IrValue[k]|=0x80;
					}
					Time=0;		//?????????							
				}
			}
		}
		if(IrValue[2]!=~IrValue[3])
		{
			return;
		}
		
		ir_key_value  = get_ir_key(IrValue[2]);
	}
}     


void IrInit()
{
	IT1=1;//?????
	EX1=1;//????0??
	EA=1;	//?????

	IRIN=1;//?????
	
}

void ReadIr() interrupt 2
{
	//IrValue[2] = 99;
	ir_decode();
}

#endif