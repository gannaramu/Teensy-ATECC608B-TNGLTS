/***********************************************************

   This is an application example for Microchip PAC193x

   © 2020 Microchip Technology Inc. and its subsidiaries.

   Subject to your compliance with these terms, you may use Microchip
   software and any derivatives of this software. You must retain the above
   copyright notice with any redistribution of this software and the
   following disclaimers.
   It is your responsibility to comply with third party license terms
   applicable to your use of third party software (including open source
   software) that may accompany this Microchip software.

   THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
   EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING
   ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
   FOR A PARTICULAR PURPOSE.

   IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
   INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
   WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
   HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
   TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
   CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
   FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

***********************************************************/

/***********************************************************

	Version 1.0.0

***********************************************************/

#include <Microchip_PAC193x.h>
#include <Wire.h>

Microchip_PAC193x PAC;

void setup() {

  Wire.begin();
  PAC.begin();
  SerialUSB.begin(9600);
  while (! SerialUSB); // Wait until SerialUSB is ready - Leonardo
  SerialUSB.print("\n Product      ID: ");
  PAC.UpdateProductID();
  SerialUSB.print(PAC.ProductID, HEX);
  SerialUSB.print("\n Manufacturer ID: ");
  PAC.UpdateManufacturerID();
  SerialUSB.print(PAC.ManufacturerID, HEX);
  SerialUSB.print("\n Revision     ID: ");
  PAC.UpdateRevisionID();
  SerialUSB.print(PAC.RevisionID, HEX);

}

void loop() {

  SerialUSB.print("\n\nRead start:");
  SerialUSB.print("\n Voltage1    (mV) = ");
  PAC.UpdateVoltage();
  SerialUSB.print(PAC.Voltage1);
  SerialUSB.print("\n Voltage2    (mV) = ");
  SerialUSB.print(PAC.Voltage2);
  SerialUSB.print("\n Voltage3    (mV) = ");
  SerialUSB.print(PAC.Voltage3);
  SerialUSB.print("\n Voltage4    (mV) = ");
  SerialUSB.print(PAC.Voltage4);




      
      PAC.UpdateVsense();
      SerialUSB.print("\n Vsense1     (mV) = ");
      SerialUSB.print(PAC.Vsense1,6);
      SerialUSB.print("\n Vsense2     (mV) = ");
      SerialUSB.print(PAC.Vsense2,6);
      SerialUSB.print("\n Vsense3     (mV) = ");
      SerialUSB.print(PAC.Vsense3,6);
      SerialUSB.print("\n Vsense4     (mV) = ");
      SerialUSB.print(PAC.Vsense4,6);
  //    SerialUSB.print("\n Current    (mA) = ");
  //    PAC.UpdateCurrent();
  //    SerialUSB.print(PAC.Current,6);
  //    SerialUSB.print("\n Raw Power (HEX) = ");
  //    PAC.UpdatePowerRaw();
  //    SerialUSB.print(PAC.PowerRaw, HEX);
  //    SerialUSB.print("\n Power      (mW) = ");
  //    PAC.UpdatePower();
  //    SerialUSB.print(PAC.Power,6);
  //    SerialUSB.print("\n Power Acc  (mW) = ");
  //    PAC.UpdatePowerAcc() ;
  //    SerialUSB.print(PAC.PowerAcc,6);
  //    SerialUSB.print("\n Energy    (mWh) = ");
  //    PAC.UpdateEnergy();
  //    SerialUSB.print(PAC.Energy,6);
  //
  delay(2000);

}
