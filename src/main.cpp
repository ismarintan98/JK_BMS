#include <Arduino.h>


#define RE 5
#define DE 4

float BMS_voltTotal,BMS_Current;
float BMS_Cell[16];
int BMS_SOC;


int statusKirim = 1;

int readInterval;

int flip;

char dataKirim[21] = {0x4E, 0x57, 0x00, 0x13 ,0x00 ,0x00, 0x00 ,0x00 ,0x06 ,0x03 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x68 ,0x00 ,0x00 ,0x01 ,0x29};

char buffTerima[268];
int idxTerima;

uint32_t time_s;

int searchIdx;

void EnRx(void)
{
  digitalWrite(DE,LOW);
  digitalWrite(RE,LOW);  
}

void EnTx(void)
{
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);  
}


void dataProses(char buffData[268]);



void setup() {
  // pinMode(2,OUTPUT);

  pinMode(RE,OUTPUT);
  pinMode(DE,OUTPUT);

  Serial.begin(115200);
  Serial1.begin(115200);
  delay(5000);
  // pinMode(2,OUTPUT);


  
  
}

void loop() {

  if(statusKirim==1)
  {
    statusKirim = 2;
    EnTx();
    delayMicroseconds(100);
    for(int i =0;i<21;i++)
    {
      Serial.print(dataKirim[i]);
    }
  }
  else if (statusKirim==2)
  {
    
    // delay(2);
    delayMicroseconds(1800);
    EnRx();
    statusKirim=3;
    
  
  }
  else if(statusKirim==3)
  {
    if(Serial.available())
    {
      

      buffTerima[idxTerima] = Serial.read();
      idxTerima++;
      
      // Serial1.write(Serial.read());

    }
  }
  

  if(millis()-time_s>100)
  {
    time_s = millis();

    if(statusKirim==3)
    {

        dataProses(buffTerima);
        
        // for(int i = 0;i<4;i++)
        // {
        //   Serial1.print(BMS_Cell[i]);
        //   Serial1.print(',');
        // }

        // Serial1.println();

        Serial1.println(BMS_Current);
        // Serial1.println(BMS_SOC);

       
        //Start Read Ulang
        idxTerima = 0;
        statusKirim = 1;

    }
  }


}



void dataProses(char buffData[268])
{
  if(buffData[0]==0x4E && buffData[1]==0x57 && buffData[11]==0x79)
  {
    for(int i=0;i<268;i++)
    {
      if(buffData[i]==0x84)
        searchIdx = i;
    }

    BMS_voltTotal = (buffData[47]<<8 | buffData[48]) * 0.01;
    BMS_Cell[0] = (buffData[14]<<8 | buffData[15]) * 0.001;
    BMS_Cell[1] = (buffData[17]<<8 | buffData[18]) * 0.001;
    BMS_Cell[2] = (buffData[20]<<8 | buffData[21]) * 0.001;
    BMS_Cell[3] = (buffData[22]<<8 | buffData[23]) * 0.001;
    BMS_Cell[4] = (buffData[25]<<8 | buffData[26]) * 0.001;
    BMS_Cell[5] = (buffData[28]<<8 | buffData[29]) * 0.001;
    BMS_Cell[6] = (buffData[30]<<8 | buffData[31]) * 0.001;
    BMS_Cell[7] = (buffData[33]<<8 | buffData[34]) * 0.001;
    BMS_Cell[8] = (buffData[36]<<8 | buffData[37]) * 0.001;
    BMS_SOC = buffData[53];
    BMS_Current = ((buffData[51] + buffData[50]*256) & 0xfff) * -0.01;
    
    


  }
}