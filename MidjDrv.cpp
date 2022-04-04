/************************************************************************
*        Gsatcom LTD., All Rights Reserved
* -----------------------------------------------------------------------
* FILE NAME     : MidjDrv.c
* AUTHORS       : Raviv Kiron
* CREATE TIME   : 25/07/2009 09:39:58
************************************************************************/


/***** Include files ***************************************************/

#include "MidjDrv_p.h"
#include <iostream>

std::queue<BYTE> FIFO;
BYTE buffer[100];

BYTE MidjRxIdx=0;

mySerial serialMIDG("/dev/ttyUSB0",115200);

RC MidjDrv_Init ()
{
   serialMIDG.Open();
   return 0;
}

/************************************************************************
* Name       : RC MidjDrv_Init (void)
* Description:
* Arguments  :
* Effected   :
* Note(s)    :
************************************************************************/












/*********************************************************************************************************
* Name       : RC MidjDrv_Rx (MSG **msg)
* Description: Build MIDJ info from received port
* Arguments  :
* Effected   :
* Note(s)    :
********************************************************************************************************/
RC MidjDrv_Rx (unsigned char* msg)
{
   RC rc=0;
   BYTE len;
   BYTE DataLen;
   RC RxValid=0;

   len=serialMIDG.Receive((unsigned char*)buffer,2);
   if (len<=0)
      return RxValid;
   for (int i=0;i<len;i++)
      FIFO.push(buffer[i]);

   // make sure we have buffer

   // wait for preamble and ID
   if (MidjRxIdx < 3)
   {      
      
      len = 0;


         //len=serialMIDG.Receive((unsigned char*)&(msg[MidjRxIdx]),1);
      if (!FIFO.empty())
      {
         msg[MidjRxIdx]=FIFO.front();
         FIFO.pop();
      }   
      else
         return RxValid;
      
      if (MidjRxIdx == 0)
      {
               

         
         if (msg[0] == MIDJ_PRE_0)
         {
            MidjRxIdx++;
            
            

         }

      }
      else if (MidjRxIdx == 1)
      {
            
               
               if (msg[1] == MIDJ_PRE_1)
               {
                  MidjRxIdx++;
                          
               }  
               else
               {
                     MidjRxIdx = 0;
               }
      }
      else if (MidjRxIdx == 2)
      {
                     if (msg[2] == MIDJ_ID_ATT )//|| MidjRxMsg->Data[2] == MIDJ_ID_GPS || MidjRxMsg->Data[2] == MIDJ_ID_UTC)
                     {
			               MidjRxIdx++;
                         
                     }   
                     else
                        MidjRxIdx = 0;
			
      }
            
   }

   // Get Data Len
   if (MidjRxIdx == 3)
   {
      
      len = 0;
   // while (len==0)
         //len=serialMIDG.Receive((unsigned char*)&(msg[3]),1);
      if (!FIFO.empty())
      {
            msg[MidjRxIdx]=FIFO.front();
            FIFO.pop();
      }
      else
         return RxValid;   
      //rc = ExtUartDrv_Rx (MIDJ_PORT, &(MidjRxMsg->Data[3]), &len);

      if (msg[3] > 45)
      {
         MidjRxIdx = 0;
         return(RxValid);
      }
      MidjRxIdx++;
   
   }

   if (MidjRxIdx > 3)
   {
      // Get Msg data
      // Preamble + ID + Count field + Data_Count + CS_Len
      DataLen = 2 + 1 + 1 + msg[3] + 2;
      len = DataLen - MidjRxIdx;
      int len0=0;
   //while (len0==0)
      //len0=serialMIDG.Receive((unsigned char*)&msg[MidjRxIdx],len);
      int i=0;
      for (i=0;i<len;i++)
      {
         if (!FIFO.empty())
         {
            msg[MidjRxIdx]=FIFO.front();
            FIFO.pop();
         }
         else
            return RxValid;
         MidjRxIdx++;

      }   
      //MidjRxIdx += len0;

      // check if we got all [Msg_Hdr + Data_Len + CS_Len]
      if (MidjRxIdx >= DataLen)
      {
         MidjRxIdx = 0;
         UINT16 CheckSum=0;
       //CalcFletcherCheckSum(&(msg[2]), DataLen-4, &CheckSum);
         //if (CheckSum == ((msg[DataLen-1] << 8) | msg[DataLen-2]))
         RxValid=1;    
         rc = MidjDrv_FixMsg((BYTE*)msg);
      }

      
   }
   return(RxValid);
}

/*********************************************************************************************************
* Name       : RC MidjDrv_Rx (MSG *Msg)
* Description: Build MIDJ info from received port
* Arguments  :
* Effected   :
* Note(s)    :
********************************************************************************************************/
RC MidjDrv_FixMsg (BYTE *Msg)
{
   MIDJ_InsMsg *InsMsg= (MIDJ_InsMsg *)Msg;
  // MIDJ_GpsMsg *GpsMsg;


         

         InsMsg->xRate = LittleXBig16(InsMsg->xRate);

         InsMsg->yRate = LittleXBig16(InsMsg->yRate);
         InsMsg->zRate = LittleXBig16(InsMsg->zRate);
         InsMsg->xAcc = LittleXBig16(InsMsg->xAcc);
         InsMsg->yAcc = LittleXBig16(InsMsg->yAcc);
         InsMsg->zAcc = LittleXBig16(InsMsg->zAcc);
         InsMsg->Yaw = LittleXBig16(InsMsg->Yaw);
         InsMsg->Pitch = LittleXBig16(InsMsg->Pitch);
         InsMsg->Roll = LittleXBig16(InsMsg->Roll);
         InsMsg->wQuaternion = LittleXBig32(InsMsg->wQuaternion);
         InsMsg->xQuaternion = LittleXBig32(InsMsg->xQuaternion);
         InsMsg->yQuaternion = LittleXBig32(InsMsg->yQuaternion);
         InsMsg->zQuaternion = LittleXBig32(InsMsg->zQuaternion);
         InsMsg->TimeStamp = LittleXBig32(InsMsg->TimeStamp);










   return(0);
}
UINT32 LittleXBig32(UINT32 val)
{
   UINT32 retVal, tmpVal;

   // MSB
   retVal = val & 0xFF;
   retVal = retVal << 24;

   // LSB
   tmpVal = val >> 24;
   retVal |= tmpVal;

   // MSB - 1
   tmpVal = (val << 8) & 0xFF0000;
   retVal |= tmpVal;

   // LSB + 1
   tmpVal = (val >> 8) & 0xFF00;
   retVal |= tmpVal;

   return(retVal);
}

RC CalcFletcherCheckSum(void *pData, INT16 DataLen, UINT16 *pCheckSum)
{
   BYTE CS0, CS1;
   int i;

   if (!pData || !pCheckSum)
      return(0);

   CS0 = CS1 = 0;

   // calculate sum of bytes
   for (i = 0; i < DataLen; i++)
      {
      CS0 += ((BYTE *)pData)[i];
      CS1 += CS0;
      }

   *pCheckSum = (CS1 << 8) | CS0;

   return(0);
}

