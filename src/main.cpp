#include <Arduino.h>
#include "TeensyDebug.h"
#pragma GCC optimize("O0")

/*
  ArduinoECCX08 - CSR (Certificate Signing Request)

  This sketch can be used to generate a CSR for a private key
  generated in an ECC508/ECC608 crypto chip slot.

  If the ECC508/ECC608 is not configured and locked it prompts
  the user to configure and lock the chip with a default TLS
  configuration.

  The user is prompted for the following information that is contained
  in the generated CSR:
  - country
  - state or province
  - locality
  - organization
  - organizational unit
  - common name

  The user can also select a slot number to use for the private key
  A new private key can also be generated in this slot.

  The circuit:
  - Arduino MKR board equipped with ECC508 or ECC608 chip

  This example code is in the public domain.
*/

#include <ArduinoECCX08.h>
#include <utility/ECCX08CSR.h>
#include <utility/ECCX08DefaultTLSConfig.h>
#include "atcacert_date.h"
#include "atcacert_def.h"
#include "SHA256.h"
#include <utility/ECCX08DeviceCert.h>
#include <utility/ECCX08Cert.h>

const int keySlot = 0;
const int compressedCertSlot = 10;
const int serialNumberAndAuthorityKeyIdentifierSlot = 11;
const int deviceIdSlot = 12;

byte DeviceCer[72];
byte Signature[64];
uint8_t EncodedDates[3];
byte SignerID[2];
byte Template_Chain_ID[1];
byte SN_FRMT[1];

const int cert_sn_size = 16;
unsigned char cert_sn[cert_sn_size];
unsigned char subject_public_key[72];
// unsigned char comp_cert[72];
unsigned char msg[67];
unsigned char digest[32];

// struct __attribute__((__packed__)) CompressedCert
// {
//   uint8_t signature[64];
//   struct
//   {
//     uint8_t year : 5;
//     uint8_t month : 4;
//     uint8_t day : 5;
//     uint8_t hour : 5;
//     uint8_t expires : 5;
//   } dates;
//   uint8_t unused[5];
// };

uint8_t _comp_cert[72];
ECCX08CertClass ECCX08Cert;

// void get_device_cert()
// {

//   if (!ECCX08.readSlot(10, _comp_cert, sizeof(_comp_cert)))
//   {
//     Serial.println("Failed Reading Slot");
//   }
//   struct CompressedCert *compressedCert = (struct CompressedCert *)_comp_cert;
//   unsigned char enc_dates[3];
//   memcpy(enc_dates, &compressedCert->dates, 3);

//   //TODO: bit field interpretation issue need to look into
//   compressedCert->dates.year = (enc_dates[0] >> 3);
//   compressedCert->dates.month = ((enc_dates[0] & 0x07) << 1) | ((enc_dates[1] & 0x80) >> 7);
//   compressedCert->dates.day = ((enc_dates[1] & 0x7C) >> 2);
//   compressedCert->dates.hour = ((enc_dates[1] & 0x03) << 3) | ((enc_dates[2] & 0xE0) >> 5);
//   compressedCert->dates.expires = (enc_dates[2] & 0x1F);

//   int year = (compressedCert->dates.year + 2000);
//   int month = compressedCert->dates.month;
//   int day = compressedCert->dates.day;
//   int hour = compressedCert->dates.hour;
//   int expireYears = compressedCert->dates.expires;

//   uint8_t *out = buffer;
//   *out++ = ASN1_SEQUENCE;
//   *out++ = 30 + ((year > 2049) ? 2 : 0) + (((year + expireYears) > 2049) ? 2 : 0);
//   out += ASN1Utils.appendDate(year, month, day, hour, 0, 0, out);
//   out += ASN1Utils.appendDate(year + expireYears, month, day, hour, 0, 0, out);

//   // ECCX08.readSlot(10, DeviceCer, 72);

//   // memcpy(Signature, &DeviceCer[0], 64);
//   // memcpy(EncodedDates, &DeviceCer[64], 3);
//   // memcpy(SignerID, &DeviceCer[67], 2);
//   // memcpy(Template_Chain_ID, &DeviceCer[69], 1);
//   // memcpy(SN_FRMT, &DeviceCer[70], 1);

//   // atcacert_tm_utc_t issue_date;
//   // atcacert_tm_utc_t expire_date;
//   // atcacert_date_dec_compcert(EncodedDates, DATEFMT_RFC5280_UTC, &issue_date, &expire_date);
//   // compressedCert->dates.year = (issueYear - 2000);
// }

atcacert_std_cert_element_t cert;
atcacert_build_state_t build_state;
String readLine()
{
  String line;

  while (1)
  {
    if (Serial.available())
    {
      char c = Serial.read();

      if (c == '\r')
      {
        // ignore
        continue;
        
      }
      else if (c == '\n')
      {
        break;
      }

      line += c;
    }
  }

  return line;
}

String promptAndReadLine(const char *prompt, const char *defaultValue)
{
  Serial.print(prompt);
  Serial.print(" [");
  Serial.print(defaultValue);
  Serial.print("]: ");

  String s = readLine();

  if (s.length() == 0)
  {
    s = defaultValue;
  }

  Serial.println(s);

  return s;
}

// void get_subject_public_key(unsigned char public_key)

void printResult()
{
  int i = 0;
  while (SHA256.available())
  {
    byte b = SHA256.read();
    digest[i] = b;
    if (b < 16)
    {
      // Serial.print("0");
    }

    // Serial.print(b, HEX);
    i++;
  }
  // Serial.println();
}

void setup()
{
  // Use the first serial port as you usually would
  Wire.begin();
  Serial.begin(115200);
  while (!Serial)
    ;

  byte error, address;
  int nDevices;

  while (1)
  {
    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("  !");

        nDevices++;
      }
      else if (error == 4)
      {
        Serial.print("Unknown error at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.println(address, HEX);
      }
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
    {

      Serial.println("done\n");
      break;
    }

    delay(5000); // wait 5 seconds for next scan
  }
  // // Debugger will use second USB Serial; this line is not need if using menu option
  // debug.begin(SerialUSB1);
  // // debug.begin(Serial1);   // or use physical serial port
  // halt();

  if (!ECCX08.begin(0x35))
  {
    Serial.println("No ECCX08 present!");
    while (1)
      ;
  }

  String serialNumber = ECCX08.serialNumber();

  Serial.print("ECCX08 Serial Number = ");
  Serial.println(serialNumber);
  Serial.println();

  if (!ECCX08.locked())
  {
    String lock = promptAndReadLine("The ECCX08 on your board is not locked, would you like to PERMANENTLY configure and lock it now? (y/N)", "N");
    lock.toLowerCase();

    if (!lock.startsWith("y"))
    {
      Serial.println("Unfortunately you can't proceed without locking it :(");
      while (1)
        ;
    }

    if (!ECCX08.writeConfiguration(ECCX08_DEFAULT_TLS_CONFIG))
    {
      Serial.println("Writing ECCX08 configuration failed!");
      while (1)
        ;
    }

    if (!ECCX08.lock())
    {
      Serial.println("Locking ECCX08 configuration failed!");
      while (1)
        ;
    }

    Serial.println("ECCX08 locked successfully");
    Serial.println();
  }

  //  get_subject_public_key(subject_public_key);
}
uint8_t buffer[35];
void loop()
{
  // ECCX08DeviceCert.beginReconstruction(0, 10);
  // ECCX08DeviceCert.setCommonName();
  // ECCX08DeviceCert.setSerialNumber();
  // ECCX08DeviceCert.setOrganizationName("Microchip Technology Inc");
  // ECCX08DeviceCert.setOrganizationalUnitName("Crypto Authentication Signer F660");
  // ECCX08DeviceCert.endReconstruction();

  // // ECCX08DeviceCert.endStorage();

  // String cert = ECCX08DeviceCert.endStorage();

  // if (!cert)
  // {
  //   Serial.println("Error generating self signed cert!");
  //   while (1)
  //     ;
  // }

  // Serial.println("Here's your self signed cert, enjoy!");
  // Serial.println();
  // Serial.println(cert);
  // Serial.println();

  // if (!ECCX08Cert.beginReconstruction(keySlot, compressedCertSlot, serialNumberAndAuthorityKeyIdentifierSlot))
  // {
  //   Serial.println("Error starting ECCX08 cert reconstruction!");
  //   while (1)
  //     ;
  // }

  ECCX08Cert.setIssuerCountryName("US");
  ECCX08Cert.setIssuerOrganizationName("Microchip Technology Inc");
  ECCX08Cert.setIssuerOrganizationalUnitName("Crypto Authentication Signer F660");
  ECCX08Cert.setIssuerCommonName("Crypto Authentication Signer F660");

  String Devcert = ECCX08Cert.endReconstruction();
  Serial.println("Devcert = ");

  Serial.println();
  Serial.println(Devcert);
  Serial.println();
  // if (!) {
  //   Serial.println("Error reconstructing ECCX08 compressed cert!");
  //   while (1);
  // }
  // Serial.println("Compressed cert = ");

  // const byte* certData = ECCX08Cert.bytes();
  // int certLength = ECCX08Cert.length();

  // for (int i = 0; i < certLength; i++) {
  //   byte b = certData[i];

  //   if (b < 16) {
  //     Serial.print('0');
  //   }
  //   Serial.print(b, HEX);
  // }
  // Serial.println();

  delay(5000);

  // Serial.print("ASN Date: ");
  // for (int i = 0; i < out - buffer; i++)
  // {
  //   if (buffer[i] < 16)
  //   {
  //     Serial.print("0");
  //   }
  //   Serial.print(buffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println("");

  // ECCX08.generatePublicKey(0, subject_public_key);
  // memcpy(&msg[0], &subject_public_key[0], 64);
  // memcpy(&msg[64], &EncodedDates[0], 3);

  // SHA256.beginHash();
  // SHA256.write(msg);
  // SHA256.endHash();
  // printResult();
  // memcpy(cert_sn, digest, cert_sn_size);
  // cert_sn[0] &= 0x7F;
  // cert_sn[0] |= 0x40;
  // Serial.print("Serial Number: ");
  // for (int i = 0; i < cert_sn_size; i++)
  // {
  //   Serial.print(cert_sn[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Serial.print("Signature: ");
  // for (int i = 0; i < 64; i++)
  // {
  //   Serial.print(Signature[i], HEX);
  //   Serial.print(" ");
  // }
  Serial.println();
}
