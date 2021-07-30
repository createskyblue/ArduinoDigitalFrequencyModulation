//位运算宏函数
#define setbit(x,y)       x|=(1<<y)
#define clrbit(x,y)       x&=~(1<<y)
#define reversebit(x,y)   x^=(1<<y)
#define getbit(x,y)       ((x) >> (y)&1)


void setup() {
  //Serial1.begin(115200);
}

void loop() {
  char mes[50];
  
  while(1) {
    sprintf(mes,"%ld\n",millis()/1000);
    Send((uint8_t*)mes,strlen(mes));
  }
}

#define SEND_DELAY 13
void Send(uint8_t datas[],uint8_t size) {
  for (int i=0;i<size;i++) {
    //Serial1.write(tmp);
    //Serial1.println(":");
    for (int j=0;j<8;j++) {
      //Serial1.print(getbit(datas[i],7 - j)?1:0);
      //比特起始信号
      tone(PA8, 1500);
      delay(SEND_DELAY);
      //数据信号
      tone(PA8, 1200 + 700 * getbit(datas[i], 7 - j));
      delay(SEND_DELAY);
    }
    //字节结束信号
    tone(PA8, 2300);
    delay(SEND_DELAY);
    noTone(PA8);
    //Serial1.println("");
  }
}
