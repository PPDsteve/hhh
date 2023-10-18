/*
Apply UVC to KHR.  May 8,2022 : Vre 1.0
*/

#include "stdio.h"
#include "kcb5.h"
#include "i2c.h"
#include "uart.h"
#include "pio.h"
#include "rom.h"
#include "timer.h"
#include "ics.h"
#include "ad.h"
#include "dac.h"
#include "math.h"

#define OFS_L 129.0			//�����̌Ҋ֐߂܂ł̋����i�G�J�N�h�~�j
#define OFS_S 213			//�����̌Ҋ֐߃T�[�{�p
#define CHG_SVA 1718.9		//�T�[�{�p�ɕϊ�
#define HEIGHT 185			//185�r��
#define RET_F_LEG 3			//�V�r�Ǐ]��
#define RST_F_LEG 3			//���������ߗ�

//// �O���[�o���ϐ�
int16_t K0W[2];			//�Ҋ֐ߑO����������p
int16_t K1W[2];			//�Ҋ֐߉����������p
int16_t K2W[2];			//�Ҋ֐߉����������p
int16_t HW [2];			//�G�֐ߏ����p
int16_t A0W[2];			//����㉺���������p
int16_t A1W[2];			//���񉡕��������p
int16_t U0W[2];			//���O����������p
int16_t U1W[2];			//��������������p
int16_t U2W[2];			//�����[�������p
int16_t EW[2];			//�I�����p
int16_t WESTW;			//����]�����p
int16_t HEADW;			//����]�����p
int16_t K0R,K0RB,U0R,U0L,U1R,U1L,EWS;
int16_t K0L,K0LB,U0WB;
int16_t K0WB;
int16_t jikuasi;
int16_t motCt,motCtBak,motCtBak2,motCtdat;
int16_t mode,modeNxt,subMode,keyMode;
int16_t	pitch_gyr,roll_gyr,yaw_gyr;
int16_t	cycle,tst0;
int16_t	walkCt,walkCtLim;		//����
int16_t	p_ofs,r_ofs;
int16_t ir,ip,ira,ipa;
int16_t irb,ipb,ct;
int16_t	pitchs,rolls,pitch_ofs, roll_ofs, yaw, yaw_ofs;
int16_t	landF,landB;

int32_t	tBak, pitchi;
uint32_t tNow;

uint8_t cmd[2];
uint8_t ff[45];
uint8_t dsp[110];
uint8_t krr[4];
int8_t	kn;
int8_t	LEDct;	//LED�_���J�E���^

float fwctEnd,fwct,fwctUp;
float pitch,roll,pitcht,rollt;
float pitch_gyrg,         roll_gyrg;
float wk,wt;
float dyi,dyib,dyis;
float dxi,dxib,dxis;
float rollg,fw,fwi,fws,sw,freeBak,tt0;
float supportingLeg,swingLeg;	//�x���r�A�V�r�Ҋ֐ߐU�o�p�x
float footH;					//�r�グ����
float swx,swy,swMax;			//���U���
float autoH,fh,fhMax;			//�r�グ����

//// �ėp ////
int32_t	ii;
int16_t	i,j;
float k,k0,k1,ks,kxy,kl;



////////////////////////
//// �^�[�~�i���\�� ////
////////////////////////
void printS(char *StringData){
	unsigned char count = 0;
	while(StringData[count] != '\0')count++;
	uart_tx(UART_COM, (unsigned char*)StringData, 0, count);
}



////////////////////////
//// �x�����Ԃ̐ݒ� ////
////////////////////////
void delay(int32_t t)
{
	int32_t tb=0;
	t*=1000;
	timer_write(TIMER,t);
	timer_start(TIMER);
	t=0;
	while(t >= tb){
		tb=t;
		t=timer_read(TIMER);
	}
}



/////////////////////
//// I2C�A�N�Z�X ////
/////////////////////
uint8_t read8(uint8_t reg )
{
	cmd[0]=reg;
	//�A�h���X��0x28���獶��1bit�V�t�g��0x50�AKONDO��I2C���C�u�����o�O�Ή�
	i2c_read(0x50, cmd, 1, ff, 1);  //BNO055_CHIP_ID_ADDR(0x28)
	return ff[0];
}
bool readLen(uint8_t reg, uint8_t len)
{
	cmd[0]=reg;
	//�A�h���X��0x28���獶��1bit�V�t�g��0x50�AKONDO��I2C���C�u�����o�O�Ή�
	i2c_read(0x50, cmd, 1, ff, len);  //BNO055_CHIP_ID_ADDR(0x28)
	return true;
}
bool write8(uint8_t reg, uint8_t dat)
{
	cmd[0]=dat;
	//�A�h���X��0x28���獶��1bit�V�t�g��0x50�AKONDO��I2C���C�u�����o�O�Ή�
	i2c_write(0x50, reg, cmd, 1);  //BNO055_CHIP_ID_ADDR(0x28)
	return true;
}



/////////////////////////////////////
//// �ڕW�l�܂ŏ��X�ɊԐڂ𓮂��� ////
/////////////////////////////////////
void movSv(short *s,int d){
//���� s:���T�[�{�ʒu d:�ڕW�T�[�{�ʒu
	if(motCt<1)	*s = d;
	else		*s += (d-*s)/motCt;
}



///////////////////////////
//// ���o�p�x���Z������ ////
///////////////////////////
void angAdj(void){

// **** pitch & roll ****
	if( pitchs<=ipb+1 && pitchs>=ipb-1 &&
		rolls <=irb+1 && rolls >=irb-1 	){
		++ip;
		ipa+=pitchs;
		ira+=rolls;
	}
	else {
		ip=0;
		ipa=0;
		ira=0;
	}
	ipb=pitchs;
	irb=rolls;
	sprintf( (char *)dsp,"P:%4d R:%4d C:%4d\r",pitchs,rolls,ip );
	printS((char *)dsp);
}



///////////////////////
//// �S�����]�|���m ////
///////////////////////
void detAng(void){
	if( 0.35>fabs(pitch) && 0.35>fabs(roll) )return;
	sprintf( (char *)dsp," PRA:%4d %4d PRG:%4d %4d\r\n",pitchs,rolls,pitch_gyr,roll_gyr );
	printS((char *)dsp);

	ics_set_pos	( UART_SIO2, 1, 4735 +1350 );	//U0R
	ics_set_pos	( UART_SIO4, 1, 9320 -1350 );	//U0L
	ics_set_pos	( UART_SIO2, 4, 4800 +2700 );	//ER
	ics_set_pos	( UART_SIO4, 4,10150 -2700 );	//EL
	ics_set_pos	( UART_SIO1, 7, 7500 -2100 );	//K0R
	ics_set_pos	( UART_SIO3, 7, 7500 +2100 );	//K0L
	ics_set_pos ( UART_SIO1, 8, 9260 -4200 );	//HR
	ics_set_pos	( UART_SIO3, 8, 5740 +4200 );	//HL
	ics_set_pos	( UART_SIO1, 9, 7910 +2100 );	//A0R
	ics_set_pos	( UART_SIO3, 9, 7100 -2100 );	//A0L
	delay(30);
	ics_set_pos		( UART_SIO1, 5, 0 );	//K2R
	ics_set_pos		( UART_SIO2, 1, 0 );	//U0R
	ics_set_pos		( UART_SIO3, 5, 0 );	//K2L
	ics_set_pos		( UART_SIO4, 1, 0 );	//U0L
	ics_set_pos		( UART_SIO1, 6, 0 );	//K1R
	ics_set_pos		( UART_SIO2, 2, 0 );	//U1R
	ics_set_pos		( UART_SIO3, 6, 0 );	//K1L
	ics_set_pos		( UART_SIO4, 2, 0 );	//U1L
	ics_set_pos		( UART_SIO1, 7, 0 );	//K0R
	ics_set_pos		( UART_SIO3, 7, 0 );	//K0L
	ics_set_pos		( UART_SIO1, 8, 0 );	//HR
	ics_set_pos		( UART_SIO2, 4, 0 );	//ER
	ics_set_pos		( UART_SIO3, 8, 0 );	//HL
	ics_set_pos		( UART_SIO4, 4, 0 );	//EL
	ics_set_pos		( UART_SIO1, 9, 0 );	//A0R
	ics_set_pos		( UART_SIO3, 9, 0 );	//A0L
	sprintf( (char *)dsp,"turn over :%4d %4d\r\n",mode,pitchs );
	printS ( (char *)dsp );
	while(1){}
}



/////////////////////
//// UVC�⏕���� ////
/////////////////////
void uvcSub(void){

	// ************ UVC�I�����x���r�𐂒��ɖ߂� ************
	if( fwct<=landF ){

		// **** ���E���� ****
		k = dyi/(11-fwct);
		dyi -= k;
		dyis += k;

		// **** �O����� ****
		if( dxi>RST_F_LEG ){
			dxi  -= RST_F_LEG;
			dxis -= RST_F_LEG;
		}
		else if( dxi<-RST_F_LEG ){
			dxi  += RST_F_LEG;
			dxis += RST_F_LEG;
		}
		else{
			dxis -= dxi;
			dxi   = 0;
		}
	}
	if(dyis> 70)	dyis=  70;
	if(dxis<-70)	dxis= -70;
	if(dxis> 70)	dxis=  70;

	// ************ �r������ ************
	if(HEIGHT>autoH){						//�r�������X�ɕ��A������
		autoH +=(HEIGHT-autoH)*0.07;//0.07
	}
	else	autoH = HEIGHT;

	if( fwct>fwctEnd-landB && rollt>0){	//���n���p���𗎂��Ռ��z��
		autoH -= ( fabs(dyi-dyib)+fabs(dxi-dxib) ) * 0.02;
	}
	if(140>autoH)autoH=140;					//�Œ�r���⏞
}

void uvcSub2(void){
	float k0,k1;

	// ************ UVC�I�����x���r�𐂒��ɖ߂� ************

	// **** ���E���� ****
	k1 = dyi/(fwctEnd-fwct+1);
	dyi -= k1;

	// **** �O����� ****
	k0 = dxi/(fwctEnd-fwct+1);
	dxi  -= k0;

	// **** ����] ****
	wk -= wk/(fwctEnd-fwct+1);
	WESTW  -= WESTW/(fwctEnd-fwct+1);
	K2W[0]  = WESTW;
	K2W[1]  =-WESTW;

	if( fwct<=landF ){
		// **** ���E���� ****
		dyis += k1;

		// **** �O����� ****
		dxis -= k0;
	}
	else{
		dyis -= dyis/(fwctEnd-fwct+1);
		dxis -= dxis/(fwctEnd-fwct+1);
	}

	// **** �r������ ****
	autoH += (HEIGHT-autoH)/(fwctEnd-fwct+1);

	if(dyis> 70)	dyis=  70;
	if(dxis<-70)	dxis= -70;
	if(dxis> 70)	dxis=  70;
}



/////////////////
//// UVC���� ////
/////////////////
void uvc(void){
	float pb,rb,k;

	// ************ �X�Ίp�ւ̃I�t�Z�b�g�K�p ************
	rb=roll;		//�ꎞ�ޔ�
	pb=pitch;
	k=sqrt(pitch*pitch+roll*roll);	//�����X�Ίp
	if( k>0.033 ){
		k=(k-0.033)/k;
		pitch *=k;
		roll  *=k;
	}
	else{
		pitch =0;
		roll  =0;
	}


	// ************ �X�Ίp�ɌW���K�p ************
	rollt =0.25*roll;
	if(jikuasi==0)	rollt = -rollt;		//��������������
	pitcht=0.25*pitch;

	if(fwct>landF && fwct<=fwctEnd-landB ){

		// ************ UVC��v�Z ************
		k	  = atan ((dyi-sw)/autoH );	//�Ћr�̉����ɑ΂���J�r�p
		kl	  = autoH/cos(k);			//�O���猩���r���e��
		ks = k+rollt;					//�J�r�p�ɉ��X���p�����Z
		k  = kl*sin(ks);					//���_���牡�ڒn�_�܂ł̍��E����
		dyi	  = k+sw;					//������UVC�␳����
		autoH = kl*cos(ks);				//K1�܂ł̍����X�V

		// **** UVC�i�O��j *****
		k 	  = atan( dxi/autoH );		//�Ћr��X�쓮�ʉ������猩�������_�̑O��J�r�p
		kl	  = autoH/cos(k);			//�Ћr��X�쓮�ʉ������猩���r��
		ks	  = k+pitcht;				//�U�o�p�ɑO��X���p�����Z
		k	  = kl*sin(ks);				//�O�����UVC�␳����
		dxi	  = k;						//�O�����UVC�␳����
		autoH = kl*cos(ks);				//K1�܂ł̍����X�V

		// ************ UVC�ϕ��l���~�b�g�ݒ� ************
		if(dyi<  0)		dyi=   0;
		if(dyi> 45)		dyi=  45;
		if(dxi<-45)		dxi= -45;
		if(dxi> 45)		dxi=  45;

		// ************ �V�r����Ǐ]������ ************

		// **** ���E���� *****
		dyis = dyi;					//�V�rY�ڕW�l

		// **** �O����� *****
		dxis = -dxi;				//�V�rX�ڕW�l

		// ************ ���r�������s�␳ ************
		if(jikuasi==0){		//���r����s�ȉ��ɂ��Ȃ�
			k = -sw+dyi;	//�E���̊O���J���
			ks=  sw+dyis;	//�����̊O���J���
		}
		else{
			ks= -sw+dyi;	//�����̊O���J���
			k =  sw+dyis;	//�E���̊O���J���
		}
		if(k+ks<0)dyis-=k+ks;	//�V�r�𕽍t�ɕ␳
	}
	roll =rb;
	pitch=pb;
}



////////////////////
//// �r�グ���� ////
////////////////////
void footUp(void){

	if( fwct>landF && fwct<=(fwctEnd-landB) )fh = fhMax * sin( M_PI*(fwct-landF)/(fwctEnd-(landF+landB)) );
	else									 fh = 0;
}



////////////////////
//// ���U�萧�� ////
////////////////////
void swCont(void){
	float k,t;

	k=swMax*sinf(M_PI*fwct/fwctEnd);//sin�J�[�u
	t=atan( (fabs(dxi)+fabs(21.5*sin(wt)))/(dyi+21.5*cos(wt)-wt) );
	if(dxi>0)	swx =  k*sin(t);
	else		swx = -k*sin(t);
	swy=k*cos(t);
}



////////////////
//// �r���� ////
////////////////
void armCont(void){
	U1W[0]=510*dyis/70; //�ҕ��ɉ����r���L����
	if(U1W[0]<0)U1W[0]=0;
	U1W[1]=U1W[0];
}



////////////////////
//// �ŏI�r�쓮 ////
////////////////////
void footCont(float x,float y,float h,int s){
//x:���_��0�Ƃ���ݒu�_�O����������i�O+�j
//y:���_��0�Ƃ���ݒu�_���E���������i�E+�j
//h:���񃍁[��������ɌҊ֐߃��[�����܂ł̒n�㍂(Max194.5)
//s:����0/�V�r1�A�w��
//�^���ʂ���݂�K1-A1�ԋ��� 	k = sqrt( x*x + h*h );
//K1-K0��40mm A0-A1��24.5mm ���v64.5mm

//���i�����^������݂�K0-A0�Ԓ��p���� k = k - 64.5
//K0-A0�Ԓ������� k = sqrt( x*x + k*k ); �܂Ƃ߂�Ɓ�
//����h�̍ő��40+65+65+24.5=194.5mm

	float k;

	k = sqrt(x*x+pow(sqrt(y*y+h*h)-64.5,2));	//K0-A0�Ԓ�������
	if(k>129){
		autoH = sqrt(pow(sqrt(129*129-x*x)+64.5,2)-y*y);//�����␳
		k=129;
	}

	x = CHG_SVA*asin(x/k);						//K0�r�U��p�x
	k = CHG_SVA*acos(k/130);					//�G�Ȃ��p�x
	if(k>1800)k=1800;							//60��Max

	if		(2*k-HW[s]> 100) k=(HW[s]+100)/2;	//��]���x�ő� 0.13s/60deg = 138count
	else if	(2*k-HW[s]<-100) k=(HW[s]-100)/2;
//	if(mode!=0 && jikuasi==s)HW[s]	= k*1.98;	//�����̂���݂��l��
	if(mode!=0 && jikuasi==s)HW[s]	= k*2;
	else					 HW[s]	= k*2;
	K0W[s]	= k+x;
	A0W[s]	= k-x;

	k = CHG_SVA*atan(y/h);						//K1�p�x

	if		(k-K1W[s]> 100) k=K1W[s]+100;		//��]���x�ő� 0.13s/60deg = 138count
	else if	(k-K1W[s]<-100) k=K1W[s]-100;

	if(mode!=0 && jikuasi==s)K1W[s] = k;		//�����̂���݂��l��
	else					 K1W[s] = k;
	A1W[s] = -k;
}



////////////////////
//// �����r�쓮 ////
////////////////////
void feetCont1(float x0, float y0, float x1, float y1, int s){

	if(s==1){
		if(y0+21.5==0)		wt = 0;			//0���Z���
		else if(jikuasi==0){
			wt = 0.5*atan( x0/(y0+21.5) );	//����]�p�x
			wk=fabs(15.0*x0/45);			//45:�ő���� 15:�������ő�ړ���
		}
		else{
			wt = 0.5*atan( -x1/(y1+21.5) );	//����]�p�x
			wk=fabs(15.0*x1/45);
		}

		WESTW = wt*CHG_SVA;					//����]�i+�ŏ�̉E��]�j
		K2W[0]= WESTW;
		K2W[1]=-WESTW;
	}

	if(jikuasi==0){
		footCont( x0,	y0-wk,	autoH   ,	0 );
		footCont( x1,	y1-wk,	autoH-fh,	1 );
	}
	else{
		footCont( x0,	y0-wk,	autoH-fh,	0 );
		footCont( x1,	y1-wk,	autoH   ,	1 );
	}
}



void feetCont2(int s){
	if(jikuasi==0)	feetCont1( dxi  -swx, dyi  -swy,	dxis -swx, dyis +swy ,s );
	else			feetCont1( dxis -swx, dyis +swy,	dxi  -swx, dyi  -swy ,s );
}



///////////////////////////
//// �����J�E���^�̐��� ////
///////////////////////////
void counterCont(void){
	if(fwct>=fwctEnd){	//�ȈՎd�l�i�Œ�����j
		jikuasi^=1;
		fwct=0;

		fh=0;
		k=dyis;
		dyis=dyi;
		dyib=dyi;
		dyi=k;

		k=dxis;
		dxis=dxi;
		dxib=dxi;
		dxi=k;
	}
	else{
		fwct+=fwctUp;
		if(fwct>fwctEnd)fwct=fwctEnd;	//�ő�l�⏞
	}
}



//##################
//#### ���s���� ####
//##################
void walk(void){

	switch(mode){

//**** �@ �J�n�A�����p���Ɉȍ~ ****
case 710:
	movSv(&K0W[0],  661);
	movSv(&K1W[0],    0);
	movSv(&K2W[0],    0);
	movSv(& HW[0], 1322);
	movSv(&A0W[0],  661);
	movSv(&A1W[0],    0);//+30
	movSv(&U0W[0],-2700);
	movSv(&U1W[0],    0);
	movSv(&U2W[0],    0);
	movSv(& EW[0],    0);
	movSv(& WESTW,    0);

	movSv(&K0W[1],  661);
	movSv(&K1W[1],    0);
	movSv(&K2W[1],    0);
	movSv(& HW[1], 1322);
	movSv(&A0W[1],  661);
	movSv(&A1W[1],    0);//-30
	movSv(&U0W[1],-2760);
	movSv(&U1W[1],    0);
	movSv(&U2W[1],    0);
	movSv(& EW[1],    0);
	movSv(& HEADW,    0);

	if(motCt>0)--motCt;
	else{
		//// �p�x�␳�p ////
		p_ofs	=0;
		r_ofs	=0;
		ip		=0;
		ir		=0;
		ipb		=0;
		irb		=0;
		ipa		=0;
		ira		=0;

		//// UVC�ϕ��p ////
		dxi		=0;
		dyi		=0;
		dxis	=0;
		dyis	=0;
		dxib	=0;
		dyib	=0;

		landF	=0;
		landB	=0;
		fwctEnd	=18;
		walkCt	=0;
		fwctUp	=1;
		fwct=1;
		autoH=HEIGHT;
		sw=0;
		swx=0;
		swy=0;
		jikuasi=1;

		//// �����p�� ////
		footCont(0,0,HEIGHT,0);
		jikuasi=0;
		footCont(0,0,HEIGHT,1);
		mode=720;					//��ԑJ��
		sprintf( (char *)dsp,"mode=720\r\n" );
		printS((char *)dsp);
	}
	break;


//**** �A �J�n���̌X�Ίp�␳ ****
case 720:
	angAdj();
	if( ip==100 ){
		p_ofs=ipa/100;
		r_ofs=ira/100;
		mode=730;		//��ԑJ��
		sprintf( (char *)dsp,"\r\nmode=730 ** Ready ** Pa:%4d Po:%4d  Ra:%4d Ro:%4d\r\n",pitchs,p_ofs,rolls,r_ofs );
		printS((char *)dsp);
	}
	detAng();			//�]�|���m
	break;


//**** �B �����p���ɂ�����X�Ό��m ****
case 730:

	// **** offset�l�␳ ****
	if( ip>=50 ){
		ip=0;
		if(rolls >0) ++r_ofs;
		if(rolls <0) --r_ofs;
		if(pitchs>0) ++p_ofs;
		if(pitchs<0) --p_ofs;
	}
	else ++ip;

	feetCont2(1);

	if(	fabs(roll)>0.033 || fabs(pitch)>0.044 ){
		if(roll>0)	jikuasi=1;
		else		jikuasi=0;
		fwct=1;
		mode=740;					//��ԑJ��
		sprintf( (char *)dsp,"mode=740\r\n" );
		printS((char *)dsp);
		break;
	}
	sprintf( (char *)dsp,"P:%4d R:%4d C:%4d\r",pitchs,rolls,ip );
	printS((char *)dsp);
	break;


//**** �C UVC����J�n ****
case 740:
	uvc();					//UVC���C������
	uvcSub();				//UVC�T�u����
	footUp();				//�r�グ�ɂ��Ҋ֐ߊp�Z�o
	feetCont2(1);			//�r�쓮
	armCont();
	counterCont();			//�����J�E���^�̐���
	if(fwct==0){
		mode=750;			//��ԑJ��
		sprintf( (char *)dsp,"mode=750\r\n" );
		printS((char *)dsp);
	}
	detAng();			//�]�|���m
	break;


//**** �D UVC��A�U�������҂� ****
case 750:
	feetCont2(1);
	if(	fwct>30 ){
		fwct=1;

		k=sqrt(0.5*dxis*dxis+dyis*dyis);	//�ړ��ʁA�O������͌���������
		swMax=17+17*k/45;

		mode=760;		//��ԑJ��
		sprintf( (char *)dsp,"mode=760 %2d \r\n",(int)swMax );
		printS((char *)dsp);
		break;
	}
	else{
		sprintf( (char *)dsp,"C:%4d\r",(int)fwct );
		printS((char *)dsp);
		++fwct;
	}
	detAng();			//�]�|���m
	break;


//**** �E �񕜓��� ****
case 760:
	landF=25;
	fwctEnd=landF+25;//15
	uvcSub2();			//UVC�T�u����
	footUp();			//�r�グ�ɂ��Ҋ֐ߊp�Z�o
	swCont();			//���U�萧��
	feetCont2(0);		//�r�쓮
	armCont();
	counterCont();		//�����J�E���^�̐���5
	if(fwct==0){
		landF=0;
		dxi		=0;
		dyi		=0;
		dxis	=0;
		dyis	=0;
		dxib	=0;
		dyib	=0;
		landF	=0;
		landB	=0;
		fwctEnd	=18;
		walkCt	=0;
		fwctUp	=1;
		fwct=1;
		autoH=HEIGHT;
		sw=0;
		swx=0;
		swy=0;
		jikuasi=1;
		mode=770;			//��ԑJ��
		sprintf( (char *)dsp,"mode=770\r\n" );
		printS((char *)dsp);
	}
	detAng();			//�]�|���m
	break;


//**** �F �񕜌�A�U�������҂� ****
case 770:
	feetCont2(0);
	if(	fwct>50 ){//50
		fwct=1;
		mode=730;		//��ԑJ��
		sprintf( (char *)dsp,"mode=730\r\n" );
		printS((char *)dsp);
		break;
	}
	else{
		sprintf( (char *)dsp,"C:%4d\r",(int)fwct );
		printS((char *)dsp);
		++fwct;
	}
	detAng();			//�]�|���m
	break;


case 780:
	feetCont2(1);
	break;


case 790:
	break;

case 791:

	dxis=-dxi;
	dyis=dyi;

	feetCont2(1);
	break;


case 700:				//���j�^�[
	sprintf( (char *)dsp,"R:%4d  %4d  RG:%4d  %4d\r\n", (int)(roll*1000),(int)(pitcht*1000), (int)roll_gyr,(int)pitch_gyr);
	printS((char *)dsp);
	break;
	}
}



//######################
//#### �L�[�Ǎ����� ####
//######################
void keyCont(void){

	//////////////////////////
	////// TTY�R�}���h���� //////
	//////////////////////////
	ff[0]=0;
	ff[1]=0;
	uart_rx (UART_COM, ff, 1,1);
	if(ff[0]!=0){
		sprintf( (char *)dsp,"%c \r\n",ff[0] );
		printS((char *)dsp);

		if(ff[0]==' ')keyMode=0;//�L�[���[�h���Z�b�g

		///////////////////////
		//// ��{���̓��[�h ////
		///////////////////////
		if(keyMode==0){
			switch(ff[0]){
				case 'r':		//���Z�b�g
					motCt=100;
					mode=710;
					sprintf( (char *)dsp,"**** Reset ****\r\n" );
					printS((char *)dsp);
					break;

				case 'g':		//�J�n
					sprintf( (char *)dsp,"**** Go ****\r\n" );
					printS((char *)dsp);
					break;

				case 't':		//����
					mode=790;
					sprintf( (char *)dsp,"**** angle Disp ****\r\n" );
					printS((char *)dsp);
					break;

				case 'y':		//����
					mode=791;
					sprintf( (char *)dsp,"**** debug ****\r\n" );
					printS((char *)dsp);
					break;

//				case 'm':		//�p�����^���[�h
//					keyMode=1;
//					break;
				case 'p':		//����ϐ��ݒ胂�[�h
					keyMode=5;
					break;
				case 'k':		//�T�[�{ K�O���[�v
					keyMode=2;
					break;
				case 'u':		//�T�[�{ U�O���[�v
					keyMode=3;
					break;
				case 'a':		//�T�[�{ A�O���[�v
					keyMode=4;
					break;
				case 'h':		//�T�[�{ H�O���[�v
					keyMode=50;
					break;
				case 'e':		//�T�[�{ E�O���[�v
					keyMode=60;
					break;
				case 'z':		//�T�[�{ HEAD
					keyMode=700;
					break;
				case 'w':		//�T�[�{ WEST
					keyMode=800;
					break;
			}
		}


		///////////////////////////
		//// ����ϐ��ݒ胂�[�h ////
		///////////////////////////
		if(keyMode==5){
			switch(ff[0]){
				case '0':
					kn=0;
					goto dd2;
				case '1':
					kn=1;
					goto dd2;
				case '2':
					kn=2;
					goto dd2;
				case '3':
					kn=3;
					goto dd2;
				case '4':
					kn=4;
					goto dd2;
				case '5':
					kn=5;
					goto dd2;
				case '6':
					kn=6;
					goto dd2;
				case '7':
					kn=7;
					goto dd2;
				case '8':
					kn=8;
					goto dd2;
				case '9':
					kn=9;
					goto dd2;
				case '+':
				case '-':
					switch(kn){
					case 0:dxi			+= ff[0]=='+'?	1:	-1;		break;
					case 1:dyi			+= ff[0]=='+'?	1:	-1;		break;
					case 2:swMax		+= ff[0]=='+'?	1:	-1;		break;
					case 3:pitch_gyrg	+= ff[0]=='+'?	0.01:-0.01;	break;
					case 4:roll_gyrg	+= ff[0]=='+'?	0.01:-0.01;	break;
					case 5:fh			+= ff[0]=='+'?  1:  -1;		break;
					case 6:fhMax		+= ff[0]=='+'?	1:  -1;		break;
					case 7:walkCtLim	+= ff[0]=='+'?  1:  -1;		break;
					case 8:autoH		+= ff[0]=='+'?  1:  -1;		break;
					}
dd2:
					sprintf( (char *)dsp, "No:%d\r\n0 dx:%d\r\n1 dy:%d\r\n2 sw:%d\r\n3 pg:%d\r\n4 rg:%d\r\n5 fh:%d\r\n6 fh%d\r\n7  wc%d\r\n8 aH:%d\r\n"
						,kn 	,(int)dxi,(int)dyi,(int)swMax,(int)(pitch_gyrg*100),(int)(roll_gyrg*100),(int)fh,(int)fhMax,(int)walkCtLim,(int)autoH);
					printS((char *)dsp);
					break;
			}
		}


		/////////////////////////
		//// �T�[�{�ݒ胂�[�h ////
		/////////////////////////
		if(keyMode==2){			//�T�[�{ K�O���[�v
			switch(ff[0]){
				case '0':		//K0�I��
					keyMode=20;
					break;
				case '1':		//K1�I��
					keyMode=21;
					break;
				case '2':		//K2�I��
					keyMode=22;
					break;
			}
		}
		if(keyMode==3){			//�T�[�{ U�O���[�v
			switch(ff[0]){
				case '0':		//U0�I��
					keyMode=30;
					break;
				case '1':		//U1�I��
					keyMode=31;
					break;
				case '2':		//U2�I��
					keyMode=32;
					break;
			}
		}
		if(keyMode==4){			//�T�[�{ A�O���[�v
			switch(ff[0]){
				case '0':		//A0�I��
					keyMode=40;
					break;
				case '1':		//A1�I��
					keyMode=41;
					break;
			}
		}

		if(keyMode>=20&&keyMode<=60){		//�T�[�{ K,U,A,N,E�O���[�v
			switch(ff[0]){
				case 'r':		//K0�I��
					keyMode=keyMode*10;
					break;
				case 'l':		//K1�I��
					keyMode=keyMode*10+1;
					break;
				case 'b':		//K2�I��
					keyMode=keyMode*10+2;
					break;
			}
		}

		if(keyMode>=200&&keyMode<=800){		//�T�[�{ K,U,A,N,E�O���[�v
			i=0;
			if(ff[0]=='+')i= 30;
			if(ff[0]=='-')i=-30;
			if(ff[0]=='+'||ff[0]=='-'){
				switch(keyMode){
				case 200:	K0W[0]+=i;	break;
				case 201:	K0W[1]+=i;	break;
				case 202:	K0W[0]+=i;K0W[1]+=i;break;
				case 210:	K1W[0]+=i;	break;
				case 211:	K1W[1]+=i;	break;
				case 212:	K1W[0]+=i;K1W[1]+=i;break;
				case 220:	K2W[0]+=i;	break;
				case 221:	K2W[1]+=i;	break;
				case 222:	K2W[0]+=i;K2W[1]+=i;break;

				case 300:	U0W[0]+=i;	break;
				case 301:	U0W[1]+=i;	break;
				case 302:	U0W[0]+=i;U0W[1]+=i;break;
				case 310:	U1W[0]+=i;	break;
				case 311:	U1W[1]+=i;	break;
				case 312:	U1W[0]+=i;U1W[1]+=i;break;
				case 320:	U2W[0]+=i;	break;
				case 321:	U2W[1]+=i;	break;
				case 322:	U2W[0]+=i;U2W[1]+=i;break;

				case 400:	A0W[0]+=i;	break;
				case 401:	A0W[1]+=i;	break;
				case 402:	A0W[0]+=i;A0W[1]+=i;break;
				case 410:	A1W[0]+=i;	break;
				case 411:	A1W[1]+=i;	break;
				case 412:	A1W[0]+=i;A1W[1]+=i;break;

				case 500:	HW[0]+=i;	break;
				case 501:	HW[1]+=i;	break;
				case 502:	HW[0]+=i;HW[1]+=i;break;

				case 600:	EW[0]+=i;	break;
				case 601:	EW[1]+=i;	break;
				case 602:	EW[0]+=i;EW[1]+=i;break;

				case 700:	HEADW+=i;	break;
				case 701:	HEADW+=i;	break;

				case 800:	WESTW+=i;	break;
				case 801:	WESTW+=i;	break;
				}
				sprintf( (char *)dsp,    "Mode=%d\r\n",modeNxt );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "K0:%7d %7d K1:%7d %7d K2:%7d %7d \r\n",K0W[0],K0W[1],K1W[0],K1W[1],K2W[0],K2W[1] );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "H:%7d %7d \r\n",HW[0] ,HW[1]  );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "A0:%7d %7d A1:%7d %7d \r\n\r\n",A0W[0],A0W[1],A1W[0],A1W[1] );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "U0:%7d %7d U1:%7d %7d U2:%7d %7d \r\n",U0W[0],U0W[1],U1W[0],U1W[1],U2W[0],U2W[1] );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "E:%7d %7d \r\n\r\n",EW[0] ,EW[1]  );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "HD:%7d WT:%7d \r\n",HEADW ,WESTW  );
				printS((char *)dsp);
			}
		}
	}
}




// **************************************************************************
// *************** Main routine *********************************************
// **************************************************************************
int main(void){

	K0W[0]=0;			//�Ҋ֐ߑO������E�����p
	K1W[0]=0;			//�Ҋ֐߉������E�����p
	K2W[0]=0;			//�Ҋ֐߉������E�����p
	HW [0]=0;			//�G�֐߉E�����p
	A0W[0]=0;			//����㉺�����E�����p
	A1W[0]=0;			//���񉡕����E�����p
	U0W[0]=-5400;			//���O������E�����p
	U1W[0]=0;			//����������E�����p
	U2W[0]=0;			//�����[���E�����p
	 EW[0]=0;			//�I�E�����p
	 WESTW=0;			//����]�����p

	K0W[1]=0;			//�Ҋ֐ߑO������������p
	K1W[1]=0;			//�Ҋ֐߉������������p
	K2W[1]=0;			//�Ҋ֐߉������������p
	HW [1]=0;			//�G�֐ߍ������p
	A0W[1]=0;			//����㉺�����������p
	A1W[1]=0;			//���񉡕����������p
	U0W[1]=-5400;			//���O������������p
	U1W[1]=0;			//����������������p
	U2W[1]=0;			//�����[���������p
	 EW[1]=0;			//�I�������p
	 HEADW=0;			//����]�����p


	///////////////////////
	//// �^�C�}�̏����� ////
	///////////////////////
	timer_init(TIMER, TIMER_MODE_TIMER32, 1000000);//��芸���������P�b�ɐݒ�
	delay(500);//wait 500ms ���̎��ԑ҂��Ȃ��ƃ_��


	///////////////////
	//// PIO������ ////
	///////////////////
	pio_init(PIO_LED1, PIO_SET_OUT);// PIO(LED1)���o�͂ɐݒ�
	pio_init(PIO_LED2, PIO_SET_OUT);// PIO(LED2)���o�͂ɐݒ�
	pio_init(PIO_T1, PIO_SET_IN);	// PIO(T1)����͂ɐݒ�
	pio_init(PIO_T2, PIO_SET_OUT);	// PIO(T2)���o�͂ɐݒ�
	pio_init(PIO_T3, PIO_SET_IN);	// PIO(T3)����͂ɐݒ�
	pio_init(PIO_T4, PIO_SET_IN);	// PIO(T4)����͂ɐݒ�
	pio_init(PIO_T5, PIO_SET_OUT);	// PIO(T5)���o�͂ɐݒ�
	pio_init(PIO_T6, PIO_SET_OUT);	// PIO(T6)���o�͂ɐݒ�
	pio_init(PIO_SW1, PIO_SET_IN);	// PIO(SW1)����͂ɐݒ�
	pio_init(PIO_SW2, PIO_SET_IN);	// PIO(SW2)����͂ɐݒ�
	pio_write (PIO_LED2, HIGH);		// �� OFF


	/////////////////////////////
	//// �V���A���|�[�g������ ////
	/////////////////////////////
	uart_init(UART_COM, UART, BR115200, 8, PARITY_NONE);
	i2c_init ( 400000, I2C_MASTER );

	sio_init (UART_SIO1, BR1250000);	// SIO�̏�����
	sio_init (UART_SIO2, BR1250000);	// SIO�̏�����
	sio_init (UART_SIO3, BR1250000);	// SIO�̏�����
	sio_init (UART_SIO4, BR1250000);	// SIO�̏�����


	/////////////////////////////
	//// �A�i���O�|�[�g������ ////
	/////////////////////////////
	ad_init(PIO_AD1, SWEEP);		//�A�i���O�|�[�g�P�ݒ�
	ad_init(PIO_AD2, SWEEP);		//�A�i���O�|�[�g�Q�ݒ�
	dac_init();						//�A�i���O�o�͐ݒ�


	////////////////////////
	//// �X�V������\�� ////
	////////////////////////
	sprintf((char *)dsp,"Version: %s %s\r\n", __DATE__, __TIME__);
	printS((char *)dsp);


	/////////////////////////
	//// BMO055 �J�n���� ////
	/////////////////////////
	//// Make sure we have the right device ////
	if(read8(0) !=0xA0 ){		// BNO055_ID
		delay(1000);			// hold on for boot
		if(read8(0) !=0xA0 ){	// BNO055_ID
			printS("*** NG1 ***\r\n");
			return false;		// still not? ok bail
		}
	}
	//// Switch to config mode (just in case since this is the default) ////
	write8(0X3D, 0);			//BNO055_OPR_MODE_ADDR  //OPERATION_MODE_CONFIG
	delay(30);
	//// Reset ////
	write8(0X3F, 0x20);			//BNO055_SYS_TRIGGER_ADDR
	delay(500);
	while (read8(0) != 0xA0){	//BNO055_CHIP_ID_ADDR  //BNO055_ID
		delay(1000);			// hold on for boot
		if(read8(0) != 0xA0 ){			// BNO055_ID
			printS("*** NG2 ***\r\n");
			return false;		// still not? ok bail
		}
	}
	delay(50);
	//// Set to normal power mode ////
	write8(0X3E, 0X00);			//BNO055_PWR_MODE_ADDR  //POWER_MODE_NORMAL
	delay(10);
	write8(0X07, 0);			//BNO055_PAGE_ID_ADDR
	write8(0X3F, 0);			//BNO055_SYS_TRIGGER_ADDR
	delay(10);
	//// Set the requested operating mode (see section 3.3) ////
	//  _mode = 0X0C;			//OPERATION_MODE_NDOF
	write8(0X3D, 0X0C);			//BNO055_OPR_MODE_ADDR  //mode
	delay(1000);
	//// Use external crystal for better accuracy ////
	write8(0X3D, 0);			//BNO055_OPR_MODE_ADDR  //OPERATION_MODE_CONFIG
	delay(50);
	write8(0X07, 0);			//BNO055_PAGE_ID_ADDR
	write8(0x0, 0x80);			//BNO055_SYS_TRIGGER_ADDR
	delay(10);
	//// Set the requested operating mode (see section 3.3) ////
	write8(0X3D, 0X0C);			//BNO055_OPR_MODE_ADDR  //modeback
	delay(50);
	printS("*** BNO055 INIT OK ***\r\n");


	/////////////////////////////
	//// �T�[�{���݊p�x ////
	/////////////////////////////
	i=ics_set_pos ( UART_SIO2, 1, 0 );	//U0R�o���U�C�ʒu
	U0W[0]=-i+4735;
	i=ics_set_pos ( UART_SIO2, 2, 0 );	//U1R +2700
	U1W[0]=-i+10110;
	i=ics_set_pos ( UART_SIO2, 3, 0 );	//U2R
	U2W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO2, 4, 0 );	//ER
	EW [0]=i-4800;
	i=ics_set_pos ( UART_SIO4, 1, 0 );	//U0L�o���U�C�ʒu
	U0W[1]=i-9320;
	i=ics_set_pos ( UART_SIO4, 2, 0 );	//U1L -2700
	U1W[1]=i-4850;
	i=ics_set_pos ( UART_SIO4, 3, 0 );	//U2L
	U2W[1]=i-7500;
	i=ics_set_pos ( UART_SIO4, 4, 0 );	//EL
	EW [1]=-i+10150;
	i=ics_set_pos ( UART_SIO1, 5, 0 );	//K2R
	K2W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO1, 6, 0 );	//K1R
	K1W[0]=-i+7470;
	i=ics_set_pos ( UART_SIO1, 7, 0 );	//K0R
	K0W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO1, 8, 0 );	//HR +1760
	HW [0]=-i+9260;
	i=ics_set_pos ( UART_SIO1, 9, 0 );	//A0R +350
	A0W[0]=i-7910;
	i=ics_set_pos ( UART_SIO1,10, 0 );	//A1R
	A1W[0]=i-7585;
	i=ics_set_pos ( UART_SIO3, 5, 0 );	//K2L
	K2W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 6, 0 );	//K1L
	K1W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 7, 0 );	//K0L
	K0W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 8, 0 );	//HL -1760
	HW [1]=i-5740;
	i=ics_set_pos ( UART_SIO3, 9, 0 );	//A0L -350
	A0W[1]=-i+7100;
	i=ics_set_pos ( UART_SIO3,10, 0 );	//A1L
	A1W[1]=-i+7530;
	i=ics_set_pos ( UART_SIO4, 0, 0 );	//HEADL
	HEADW=i-7500;
	i=ics_set_pos ( UART_SIO2, 0, 0 );	//WESTR
	WESTW=i-7500;


	/////////////////////////////
	//// �T�[�{�X�g���b�`�ݒ� ////
	/////////////////////////////
//	ics_set_param ( UART_SIO1, 7,ICS_STRC_SC,250);	//K0R
//	ics_set_param ( UART_SIO3, 7,ICS_STRC_SC,250);	//K0L

	ics_set_param ( UART_SIO2, 1,ICS_STRC_SC,20);	//U0R
	ics_set_param ( UART_SIO4, 1,ICS_STRC_SC,20);	//U0L

	ics_set_param ( UART_SIO2, 2,ICS_STRC_SC,20);	//U1R
	ics_set_param ( UART_SIO4, 2,ICS_STRC_SC,20);	//U1L

	ics_set_param ( UART_SIO2, 4,ICS_STRC_SC,20);	//ER
	ics_set_param ( UART_SIO4, 4,ICS_STRC_SC,20);	//EL


	////////////////////////
	//// 10ms�^�C�}�J�n ////
	////////////////////////
	timer_write(TIMER,1000000);
	timer_start(TIMER);


	////////////////////
	//// �ϐ������� ////
	////////////////////

	LEDct=0;	//LED�_���J�E���^

	tBak=0;
	pitchi=0;
	tNow=0;

	p_ofs=0;
	r_ofs=0;
	ir=0;
	ip=0;
	irb=0;
	ipb=0;

	kn=0;

	motCt=100;
	keyMode=0;
	cycle=10000;
	mode=710;
	pitch_gyrg=0.08;
	roll_gyrg=0.1;

	swMax=25;//22
	fhMax=35;
	walkCtLim=3;



//----------------------------------------------------------------------------------
	////////////////////////////////////////////////
	//////////////////  MAIN LOOP  /////////////////
	////////////////////////////////////////////////
top:
	//////////////////////
	//// 10ms�҂����� ////
	//////////////////////
pio_write (PIO_T2, HIGH);	//OFF(wait���Ԋm�F)

	do{
		tNow=timer_read(TIMER);
	}while(tNow<cycle);

	if(tNow>cycle+10){
		sprintf( (char *)dsp,"************** %d \r\n",(int)tNow);
		printS ( (char *)dsp );
	}
	timer_start(TIMER);


	////////////////////
	//// �T�[�{�ݒ� ////
	////////////////////

	//// �֐߃��~�b�g ////
	if(K1W[0]> 800)K1W[0]	= 800;
	if(K1W[0]<-450)K1W[0]	=-450;
	if(K1W[1]> 800)K1W[1]	= 800;
	if(K1W[1]<-450)K1W[1]	=-450;
	if(A0W[0]> 3500)A0W[0]	= 3500;
	if(A0W[0]<-3500)A0W[0]	=-3500;
	if(A0W[1]> 3500)A0W[1]	= 3500;
	if(A0W[1]<-3500)A0W[1]	=-3500;
	if(A1W[0]> 420)A1W[0]	= 420;	//�Y�t�i�ȉ��H+�A���~�\�[���ł̎�������
	if(A1W[0]<-900)A1W[0]	=-900;	//�Y�t�i�ȉ��H+�A���~�\�[���ł̎�������
	if(A1W[1]> 420)A1W[1]	= 420;	//�Y�t�i�ȉ��H+�A���~�\�[���ł̎�������
	if(A1W[1]<-900)A1W[1]	=-900;	//�Y�t�i�ȉ��H+�A���~�\�[���ł̎�������

	//// ICS�f�[�^����M ////
	//// ���Ӄ|�W�V�����f�[�^��10500�𒴂���ƃT�[�{���������Ȃ��i�Ԍ������j
	//// ���Ӄ|�W�V�����f�[�^�� 3600�ȉ��ł͔����͂��邪�����Ȃ�

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 5, 7560 -(K2W[0])-60 );	//K2R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K2R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO2, 1, 4735 -(U0W[0]) );	//U0R�o���U�C�ʒu
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U0R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U0R=-(ii-4735);

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 5, 7500 +(K2W[1])-90 );	//K2L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K2L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO4, 1, 9320 +(U0W[1])-60 );	//U0L�o���U�C�ʒu
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U0L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U0L=(ii-9230);

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 6, 7470 -(K1W[0])-30 );	//K1R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K1R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO2, 2,10110 -(U1W[0]) );	//U1R +2700
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U1R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U1R=-(ii-10110);

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 6, 7650 +(K1W[1])-90 );//K1L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K1L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO4, 2, 4850 +(U1W[1]) );	//U1L -2700
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U1L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U1L=ii-4850;


	pio_write (PIO_T2, LOW );	//ON
	ii=ics_set_pos ( UART_SIO1, 7, 7480 -(K0W[0])-30 );	//K0R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K0R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	K0R=7510-ii;

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 3, 7500 -(U2W[0]) );	//U2R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U2R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ii=ics_set_pos ( UART_SIO3, 7, 7500 +(K0W[1]) );	//K0L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K0L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	K0L=ii-7500;

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 3, 7500 +(U2W[1]) );	//U2L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U2L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 8, 9320 -(HW [0]) );	//HR +1760
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"HR**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 4, 4800 +(EW [0]+i) );	//ER
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"ER**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 8, 5770 +(HW [1])-120 );	//HL -1760
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"HL**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 4,10150 -(EW [1]+i) );	//EL
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"EL**** %d \r\n",(int)tNow);printS ( (char *)dsp );}


	i=pitch_gyrg*pitch_gyr;
	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 9, 7870-10 +(A0W[0]) + i+60 );	//A0R

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 0, 7500 +(WESTW ) );			//WESTR

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 9, 7100    -(A0W[1]) - i );	//A0L

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 0, 7500 +(HEADW ) );			//HEADL

	i=roll_gyrg*roll_gyr;
	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1,10, 7470 +(A1W[0]) - i-30 );		//A1R

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO3,10, 7560 -(A1W[1]) - i-60 );		//A1L


	/////////////////////
	//// IMU�ǎ揈�� ////
	/////////////////////
	//      +----------+
	//      |         *| RST   PITCH  ROLL  HEADING
	//  ADR |*        *| SCL
	//  INT |*        *| SDA     ^            /->
	//  PS1 |*        *| GND     |            |
	//  PS0 |*        *| 3VO     Y    Z-->    \-X
	//      |         *| VIN
	//      +----------+

	//����@2020 2/6    roll : �E���X���i2879�j �����X���i-2879�j臒l2879
	//                  pitch: �O���X���i+�j    �㑤�X���i-�j    �␳�l20

	readLen(0X1A, 6);	//��Ίp�x�Ǎ�(deg��16�{�\��)
	yaw	   = ((int16_t)ff[0]) | (((int16_t)ff[1]) << 8); //�����E��]�� +
	pitchs = ((int16_t)ff[2]) | (((int16_t)ff[3]) << 8); //�����O�X��   -
	rolls  = ((int16_t)ff[4]) | (((int16_t)ff[5]) << 8); //�����E�X�΂� +
	if(rolls>0)	rolls= 2879-rolls;
	else		rolls=-2879-rolls;

	pitchs -= p_ofs;	//�␳
	rolls  -= r_ofs;	//�␳

	pitch = (float)pitchs*(M_PI/(180.0*16.0));	//rad�ɕϊ�
	roll  = (float)rolls *(M_PI/(180.0*16.0));	//rad�ɕϊ�


	readLen(0X14, 6);	//�p���x�Ǎ� ��roll��yaw������Ǝ��ۂ��t
	roll_gyr 	= ((int16_t)ff[0]) | (((int16_t)ff[1]) << 8);	//�����E�X�΂� �{
	pitch_gyr	= ((int16_t)ff[2]) | (((int16_t)ff[3]) << 8);	//�����O�X��   �|
	yaw_gyr		= ((int16_t)ff[4]) | (((int16_t)ff[5]) << 8);	//�����E��]�� �{


	pio_write (PIO_T2, LOW );	//ON


	keyCont();


	walk();


	///////////////
	//// ��LED ////
	///////////////
	++LEDct;
	if( LEDct > 100 )LEDct = -100;				//IMU Ready

	if( mode<=720 && LEDct > 10 )LEDct = -10;	//IMU not Ready

	if( LEDct > 0   ){
		dac_write (0xffff);
		pio_write (PIO_LED1, LOW );	//ON
	}
	else{
		dac_write (0);
		pio_write (PIO_LED1, HIGH);	//OFF
	}

	goto top;
}
