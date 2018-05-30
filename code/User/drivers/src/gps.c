#include "drivers.h"
#include "gps.h"

static nmea_msg gpsx;
static nmea_msg mGpsxBuf[GPS_BUF_SIZE];
static uint8_t mGpsxNum;
static uint8_t mLastGpsxNum;

static uint8_t mGpsStandby = FALSE;

void gps_hook(void)
{
    nmea_msg *msg = &mGpsxBuf[mLastGpsxNum];
    uint8_t i;

    if(msg->latitude != gpsx.latitude
            || msg->longitude != gpsx.longitude) {
        logi("%s: new gps comming!", __func__);
        mLastGpsxNum = mGpsxNum;
        memcpy(&mGpsxBuf[mGpsxNum++], &gpsx, sizeof(nmea_msg));
        if(mGpsxNum == GPS_BUF_SIZE) {
            for(i = 0; i < GPS_BUF_SIZE; i++) {
                logi("%s: lng = %d, lat = %d, time = %d-%d-%d %d:%d:%d",
                        __func__,
                        mGpsxBuf[i].longitude,
                        mGpsxBuf[i].latitude,
                        mGpsxBuf[i].utc.year,
                        mGpsxBuf[i].utc.month,
                        mGpsxBuf[i].utc.date,
                        mGpsxBuf[i].utc.hour,
                        mGpsxBuf[i].utc.min,
                        mGpsxBuf[i].utc.sec);
            }
            //transmit_multi_location(mGpsxBuf, GPS_BUF_SIZE);
            //upload the gps data
            mGpsxNum = 0;
            for(i = 0; i < GPS_BUF_SIZE; i++) {
                memset(&mGpsxBuf[i], 0x00, sizeof(nmea_msg));
            }
        }
    }
}

void gps_init(void)
{
    uint8_t i;

    mGpsxNum = 0;
    for(i = 0; i < GPS_BUF_SIZE; i++) {
        memset(&mGpsxBuf[i], 0x00, sizeof(nmea_msg));
    }

    gps_poweron();
}

void gps_poweron(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_12);
    //wait 2s for gps ready
    os_delay(2);
    gps_set_rate(5000);
    mGpsStandby = FALSE;
}

void gps_poweroff(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}

void gps_set_rate(uint16_t ms)
{
    char buf[32];
    uint8_t checksum;
    uint8_t i, n;

    checksum = 0x00;
    memset(buf, 0x00, 32);
    n = snprintf(buf, 32, "$PMTK220,%d*1F\r\n", ms);
    for(i = 1; i < n; i++) {
        if(buf[i] == '*')
            break;
        checksum ^= buf[i];
    }
    snprintf(buf, 32, "$PMTK220,%d*%02X\r\n", ms, checksum);
    gps_write(buf);
}

void gps_standby(uint8_t on)
{
    char buf[32];
    uint8_t checksum;
    uint8_t i, n;

    if(mGpsStandby == on)
        return;

    checksum = 0x00;
    memset(buf, 0x00, 32);
    n = snprintf(buf, 32, "$PMTK161,%d*1F\r\n", on);
    for(i = 1; i < n; i++) {
        if(buf[i] == '*')
            break;
        checksum ^= buf[i];
    }
    snprintf(buf, 32, "$PMTK161,%d*%02X\r\n", on, checksum);
    gps_write(buf);
    mGpsStandby = on;
}

void gps_aiding(uint8_t index, uint8_t *epoBuf)
{
    uint16_t i;
    uint32_t eBuf[18];
    char *buf;
    uint8_t checksum;
    uint16_t offset, n;

    buf = (char *)malloc(512);
    memcpy(eBuf, epoBuf, 72);
    checksum = 0x00;
    memset(buf, 0x00, 512);

    offset = 0;
    n = snprintf(buf, 512, "$PMTK721,%d,", (index + 1));
    offset += n;
    for(i = 0; i < 17; i++) {
        n = snprintf(buf + offset, 512, "%08X,", eBuf[i]);
        offset += n;
    }
    n = snprintf(buf + offset, 512, "%08X*", eBuf[17]);
    offset += n;
    for(i = 1; i < offset; i++) {
        if(buf[i] == '*')
            break;
        checksum ^= buf[i];
    }
    logi("buf: %s", buf);
    snprintf(buf, 512, "%s%02X\r\n", buf, checksum);
    gps_write(buf);
    free(buf);
}

void gps_set_utc(uint16_t year, uint8_t mon, uint8_t day,
        uint8_t hour, uint8_t min, uint8_t sec)
{
    char buf[128];
    uint8_t checksum;
    uint8_t i, n;

    checksum = 0x00;
    memset(buf, 0x00, 128);
    n = snprintf(buf, 128, "$PMTK740,%04d,%02d,%02d,%02d,%02d,%02d*1F\r\n",
            year, mon, day, hour, min, sec);
    for(i = 1; i < n; i++) {
        if(buf[i] == '*')
            break;
        checksum ^= buf[i];
    }
    snprintf(buf, 128, "$PMTK740,%04d,%02d,%02d,%02d,%02d,%02d*%02X\r\n",
            year, mon, day, hour, min, sec, checksum);
    gps_write(buf);
}

void gps_write(char *buf)
{
    //comClearRxFifo(COM_GPS);
    comSendBuf(COM_GPS, (uint8_t *)buf, strlen(buf));
    logi("-> %s", buf);
}

void vTaskGpsProcess(void *unused)
{
    uint8_t u_data;
    uint8_t s_buf[256];
    uint8_t s_index = 0;

    memset(s_buf, 0x00, 256);
    memset(&gpsx, 0x00, sizeof(nmea_msg));
    gps_init();
    while(1) {
        os_delay_ms(20);
        while(comGetChar(COM_GPS, &u_data)) {
#ifdef GPS_DEBUG_ENABLE
            printf("%c", u_data);
#endif
            s_buf[s_index] = u_data;
            if(s_index >= 2 && s_buf[s_index] == 0x0a && s_buf[s_index - 1] == 0x0d) {
                //logi("%s: %s", __func__, s_buf);
                GPS_Analysis(&gpsx, s_buf);
                gps_hook();
                memset(s_buf, 0x00, 256);
                s_index = 0;
                /*
                   logi("svnum = %d, latitude = %d, longitude = %d, posslnum = %d",
                   gpsx.svnum,
                   gpsx.latitude,
                   gpsx.longitude,
                   gpsx.posslnum);
                   */
            } else {
                s_index ++;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////	 
//NEO-6M GPS模块驱动代码	   
//修改日期:2014/2/8
//版本：V2.0			
//********************************************************************************
//V2.0 修改说明 20140208
//1,添加Ublox_Cfg_Cfg_Save函数
//2,添加Ublox_Cfg_Msg函数
//3,添加Ublox_Cfg_Prt函数.				  
////////////////////////////////////////////////////////////////////////////////// 	   

//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
    u8 *p=buf;
    while(cx)
    {		 
        if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
        if(*buf==',')cx--;
        buf++;
    }
    return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
u32 NMEA_Pow(u8 m,u8 n)
{
    u32 result=1;	 
    while(n--)result*=m;    
    return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(u8 *buf,u8*dx)
{
    u8 *p=buf;
    u32 ires=0,fres=0;
    u8 ilen=0,flen=0,i;
    u8 mask=0;
    int res;
    while(1) //得到整数和小数的长度
    {
        if(*p=='-'){mask|=0X02;p++;}//是负数
        if(*p==','||(*p=='*'))break;//遇到结束了
        if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
        else if(*p>'9'||(*p<'0'))	//有非法字符
        {	
            ilen=0;
            flen=0;
            break;
        }	
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++;	//去掉负号
    for(i=0;i<ilen;i++)	//得到整数部分数据
    {  
        ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;	//最多取5位小数
    *dx=flen;	 		//小数点位数
    for(i=0;i<flen;i++)	//得到小数部分数据
    {  
        fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
    } 
    res=ires*NMEA_Pow(10,flen)+fres;
    if(mask&0X02)res=-res;		   
    return res;
}	  							 
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p,*p1,dx;
    u8 len,i,j,slx=0;
    u8 posx;   	 
    p=buf;
    p1=(u8*)strstr((const char *)p,"$GPGSV");
    len=p1[7]-'0';								//得到GPGSV的条数
    posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
    if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
    for(i=0;i<len;i++)
    {	 
        p1=(u8*)strstr((const char *)p,"$GPGSV");  
        for(j=0;j<4;j++)
        {	  
            posx=NMEA_Comma_Pos(p1,4+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
            else break; 
            posx=NMEA_Comma_Pos(p1,5+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
            else break;
            posx=NMEA_Comma_Pos(p1,6+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
            else break; 
            posx=NMEA_Comma_Pos(p1,7+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
            else break;
            slx++;	   
        }   
        p=p1+1;//切换到下一个GPGSV信息
    }   
}
void NMEA_GNGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p,*p1,dx;
    u8 len,i,j,slx=0;
    u8 posx;   	 
    p=buf;
    p1=(u8*)strstr((const char *)p,"$GNGSV");
    len=p1[7]-'0';								//得到GPGSV的条数
    posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
    if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
    for(i=0;i<len;i++)
    {	 
        p1=(u8*)strstr((const char *)p,"$GPGSV");  
        for(j=0;j<4;j++)
        {	  
            posx=NMEA_Comma_Pos(p1,4+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
            else break; 
            posx=NMEA_Comma_Pos(p1,5+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
            else break;
            posx=NMEA_Comma_Pos(p1,6+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
            else break; 
            posx=NMEA_Comma_Pos(p1,7+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
            else break;
            slx++;	   
        }   
        p=p1+1;//切换到下一个GPGSV信息
    }   
}
//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx;    
    p1=(u8*)strstr((const char *)buf,"$GPGGA");
    posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
    if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
    if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
    posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
    if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx;    
    p1=(u8*)strstr((const char *)buf,"$GNGGA");
    posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
    if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
    if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
    posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
    if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
//分析GPGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx; 
    u8 i;   
    p1=(u8*)strstr((const char *)buf,"$GPGSA");
    posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
    if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
    for(i=0;i<12;i++)										//得到定位卫星编号
    {
        posx=NMEA_Comma_Pos(p1,3+i);					 
        if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
        else break; 
    }				  
    posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
    if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
    posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
    if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
    posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
    if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx; 
    u8 i;   
    p1=(u8*)strstr((const char *)buf,"$GNGSA");
    posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
    if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
    for(i=0;i<12;i++)										//得到定位卫星编号
    {
        posx=NMEA_Comma_Pos(p1,3+i);					 
        if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
        else break; 
    }				  
    posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
    if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
    posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
    if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
    posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
    if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
//分析GPRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx;     
    u32 temp;	   
    float rs;  
    p1=(u8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
    posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
        gpsx->utc.hour=temp/10000;
        gpsx->utc.min=(temp/100)%100;
        gpsx->utc.sec=temp%100;	 	 
    }	
    posx=NMEA_Comma_Pos(p1,3);								//得到纬度
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);		 	 
        gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
        gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
    }
    posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
    if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
    posx=NMEA_Comma_Pos(p1,5);								//得到经度
    if(posx!=0XFF)
    {												  
        temp=NMEA_Str2num(p1+posx,&dx);		 	 
        gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
        gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
    }
    posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
    if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
    posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
        gpsx->utc.date=temp/10000;
        gpsx->utc.month=(temp/100)%100;
        gpsx->utc.year=2000+temp%100;	 	 
    } 
}
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx;     
    u32 temp;	   
    float rs;  
    p1=(u8*)strstr((const char *)buf,"GNRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
    posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
        gpsx->utc.hour=temp/10000;
        gpsx->utc.min=(temp/100)%100;
        gpsx->utc.sec=temp%100;	 	 
    }	
    posx=NMEA_Comma_Pos(p1,3);								//得到纬度
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);		 	 
        gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
        gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
    }
    posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
    if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
    posx=NMEA_Comma_Pos(p1,5);								//得到经度
    if(posx!=0XFF)
    {												  
        temp=NMEA_Str2num(p1+posx,&dx);		 	 
        gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
        gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
    }
    posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
    if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
    posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
        gpsx->utc.date=temp/10000;
        gpsx->utc.month=(temp/100)%100;
        gpsx->utc.year=2000+temp%100;	 	 
    } 
}
//分析GPVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx;    
    p1=(u8*)strstr((const char *)buf,"$GPVTG");							 
    posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
    if(posx!=0XFF)
    {
        gpsx->speed=NMEA_Str2num(p1+posx,&dx);
        if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
    }
}  
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;			 
    u8 posx;    
    p1=(u8*)strstr((const char *)buf,"$GNVTG");							 
    posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
    if(posx!=0XFF)
    {
        gpsx->speed=NMEA_Str2num(p1+posx,&dx);
        if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
    }
}  
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
    NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
    NMEA_GNGSV_Analysis(gpsx,buf);	//GNGSV解析
    NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA解析 	
    NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA解析 	
    NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
    NMEA_GNGSA_Analysis(gpsx,buf);	//GNGSA解析
    NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC解析
    NMEA_GNRMC_Analysis(gpsx,buf);	//GNRMC解析
    NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
    NMEA_GNVTG_Analysis(gpsx,buf);	//GNVTG解析
}
