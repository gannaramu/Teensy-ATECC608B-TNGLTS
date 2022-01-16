/*
  ECCX08 Random Number

  This sketch uses the ECC508 or ECC608 to generate a random number 
  every second and print it to the Serial monitor

  Circuit:
   - MKR board with ECC508 or ECC608 on board

  created 19 July 2018
  by Sandeep Mistry
*/
#include <Arduino.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include "tng_root_cert.h"
#include <utility/PEMUtils.h>
#define TNGTLS_CERT_TEMPLATE_1_SIGNER_SIZE 520

const uint8_t g_cryptoauth_root_ca_002_cert[501] = {
    0x30, 0x82, 0x01, 0xf1, 0x30, 0x82, 0x01, 0x97, 0xa0, 0x03, 0x02, 0x01,
    0x02, 0x02, 0x10, 0x77, 0xd3, 0x6d, 0x95, 0x6e, 0xc8, 0xae, 0x62, 0x05,
    0xe5, 0x8e, 0x3a, 0xcb, 0x98, 0x5a, 0x81, 0x30, 0x0a, 0x06, 0x08, 0x2a,
    0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x30, 0x4f, 0x31, 0x21, 0x30,
    0x1f, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x18, 0x4d, 0x69, 0x63, 0x72,
    0x6f, 0x63, 0x68, 0x69, 0x70, 0x20, 0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f,
    0x6c, 0x6f, 0x67, 0x79, 0x20, 0x49, 0x6e, 0x63, 0x31, 0x2a, 0x30, 0x28,
    0x06, 0x03, 0x55, 0x04, 0x03, 0x0c, 0x21, 0x43, 0x72, 0x79, 0x70, 0x74,
    0x6f, 0x20, 0x41, 0x75, 0x74, 0x68, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x61,
    0x74, 0x69, 0x6f, 0x6e, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41,
    0x20, 0x30, 0x30, 0x32, 0x30, 0x20, 0x17, 0x0d, 0x31, 0x38, 0x31, 0x31,
    0x30, 0x38, 0x31, 0x39, 0x31, 0x32, 0x31, 0x39, 0x5a, 0x18, 0x0f, 0x32,
    0x30, 0x35, 0x38, 0x31, 0x31, 0x30, 0x38, 0x31, 0x39, 0x31, 0x32, 0x31,
    0x39, 0x5a, 0x30, 0x4f, 0x31, 0x21, 0x30, 0x1f, 0x06, 0x03, 0x55, 0x04,
    0x0a, 0x0c, 0x18, 0x4d, 0x69, 0x63, 0x72, 0x6f, 0x63, 0x68, 0x69, 0x70,
    0x20, 0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f, 0x6c, 0x6f, 0x67, 0x79, 0x20,
    0x49, 0x6e, 0x63, 0x31, 0x2a, 0x30, 0x28, 0x06, 0x03, 0x55, 0x04, 0x03,
    0x0c, 0x21, 0x43, 0x72, 0x79, 0x70, 0x74, 0x6f, 0x20, 0x41, 0x75, 0x74,
    0x68, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x20,
    0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41, 0x20, 0x30, 0x30, 0x32, 0x30,
    0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02, 0x01,
    0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03, 0x42,
    0x00, 0x04, 0xbd, 0x54, 0xe6, 0x6d, 0xe3, 0x87, 0x54, 0x84, 0x00, 0x6b,
    0x53, 0xae, 0x15, 0x80, 0xd5, 0x0a, 0xa0, 0x69, 0xe7, 0x8a, 0xdf, 0x55,
    0x78, 0xd8, 0x5c, 0xe2, 0xd5, 0x4d, 0xd5, 0xb8, 0x30, 0x29, 0x6b, 0xff,
    0xdd, 0x6e, 0x6f, 0x72, 0x56, 0xfb, 0xd9, 0x9e, 0xf1, 0xa1, 0x16, 0xb1,
    0x1d, 0x33, 0xad, 0x49, 0x10, 0x3a, 0xa1, 0x85, 0x87, 0x39, 0xdc, 0xfa,
    0xe4, 0x37, 0xe1, 0x9d, 0x63, 0x4e, 0xa3, 0x53, 0x30, 0x51, 0x30, 0x1d,
    0x06, 0x03, 0x55, 0x1d, 0x0e, 0x04, 0x16, 0x04, 0x14, 0x7a, 0xed, 0x7d,
    0x6d, 0xc6, 0xb7, 0x78, 0x9d, 0xb2, 0x38, 0x01, 0xa5, 0xe8, 0x4a, 0x8c,
    0xb0, 0xa4, 0x0e, 0x2a, 0x8c, 0x30, 0x1f, 0x06, 0x03, 0x55, 0x1d, 0x23,
    0x04, 0x18, 0x30, 0x16, 0x80, 0x14, 0x7a, 0xed, 0x7d, 0x6d, 0xc6, 0xb7,
    0x78, 0x9d, 0xb2, 0x38, 0x01, 0xa5, 0xe8, 0x4a, 0x8c, 0xb0, 0xa4, 0x0e,
    0x2a, 0x8c, 0x30, 0x0f, 0x06, 0x03, 0x55, 0x1d, 0x13, 0x01, 0x01, 0xff,
    0x04, 0x05, 0x30, 0x03, 0x01, 0x01, 0xff, 0x30, 0x0a, 0x06, 0x08, 0x2a,
    0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x03, 0x48, 0x00, 0x30, 0x45,
    0x02, 0x21, 0x00, 0xa1, 0xdc, 0x63, 0x45, 0x90, 0xec, 0x81, 0x9e, 0xe1,
    0xde, 0x5b, 0x81, 0x12, 0x65, 0x51, 0xad, 0xd4, 0xc2, 0xc4, 0xf8, 0xe5,
    0x95, 0x28, 0x2e, 0xe0, 0x4b, 0xe7, 0x68, 0xec, 0x7c, 0x02, 0x73, 0x02,
    0x20, 0x3e, 0x6b, 0xa7, 0x4e, 0x9e, 0x4c, 0x0a, 0xd6, 0x8c, 0x24, 0xb0,
    0xfb, 0x2e, 0xe7, 0x93, 0xd2, 0xe6, 0xbe, 0x94, 0x65, 0xca, 0x15, 0xd0,
    0xea, 0x5b, 0xc8, 0x7f, 0x55, 0x79, 0x99, 0x5c, 0xad
};

const uint8_t g_tngtls_cert_template_1_signer[TNGTLS_CERT_TEMPLATE_1_SIGNER_SIZE] = {
    0x30, 0x82, 0x02, 0x04, 0x30, 0x82, 0x01, 0xaa, 0xa0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x10, 0x44,
    0x0e, 0xe4, 0x17, 0x0c, 0xb5, 0x45, 0xce, 0x59, 0x69, 0x8e, 0x30, 0x56, 0x99, 0x0a, 0x5d, 0x30,
    0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x30, 0x4f, 0x31, 0x21, 0x30,
    0x1f, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x18, 0x4d, 0x69, 0x63, 0x72, 0x6f, 0x63, 0x68, 0x69,
    0x70, 0x20, 0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f, 0x6c, 0x6f, 0x67, 0x79, 0x20, 0x49, 0x6e, 0x63,
    0x31, 0x2a, 0x30, 0x28, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0c, 0x21, 0x43, 0x72, 0x79, 0x70, 0x74,
    0x6f, 0x20, 0x41, 0x75, 0x74, 0x68, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e,
    0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41, 0x20, 0x30, 0x30, 0x32, 0x30, 0x20, 0x17, 0x0d,
    0x31, 0x38, 0x31, 0x31, 0x30, 0x38, 0x30, 0x34, 0x30, 0x30, 0x30, 0x30, 0x5a, 0x18, 0x0f, 0x32,
    0x30, 0x34, 0x39, 0x31, 0x31, 0x30, 0x38, 0x30, 0x34, 0x30, 0x30, 0x30, 0x30, 0x5a, 0x30, 0x4f,
    0x31, 0x21, 0x30, 0x1f, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x18, 0x4d, 0x69, 0x63, 0x72, 0x6f,
    0x63, 0x68, 0x69, 0x70, 0x20, 0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f, 0x6c, 0x6f, 0x67, 0x79, 0x20,
    0x49, 0x6e, 0x63, 0x31, 0x2a, 0x30, 0x28, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0c, 0x21, 0x43, 0x72,
    0x79, 0x70, 0x74, 0x6f, 0x20, 0x41, 0x75, 0x74, 0x68, 0x65, 0x6e, 0x74, 0x69, 0x63, 0x61, 0x74,
    0x69, 0x6f, 0x6e, 0x20, 0x53, 0x69, 0x67, 0x6e, 0x65, 0x72, 0x20, 0x46, 0x46, 0x46, 0x46, 0x30,
    0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a, 0x86,
    0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04, 0x84, 0x98, 0x44, 0x0a, 0x31, 0x9b,
    0x3f, 0x71, 0xe2, 0x5d, 0x52, 0x26, 0x00, 0x90, 0x00, 0xc7, 0x56, 0xbd, 0x5c, 0x0f, 0xae, 0x4a,
    0x1b, 0x84, 0x1a, 0xd4, 0xa3, 0x3f, 0x21, 0xab, 0xa0, 0x9a, 0x48, 0x10, 0x1c, 0x75, 0xc8, 0x28,
    0x24, 0x90, 0xb3, 0xb6, 0x5a, 0x52, 0x80, 0x27, 0x29, 0xbd, 0x3a, 0x75, 0x2c, 0x3d, 0xf0, 0xdd,
    0x1b, 0x04, 0xa2, 0xa1, 0xb5, 0x7e, 0x0c, 0x92, 0x24, 0x47, 0xa3, 0x66, 0x30, 0x64, 0x30, 0x0e,
    0x06, 0x03, 0x55, 0x1d, 0x0f, 0x01, 0x01, 0xff, 0x04, 0x04, 0x03, 0x02, 0x01, 0x86, 0x30, 0x12,
    0x06, 0x03, 0x55, 0x1d, 0x13, 0x01, 0x01, 0xff, 0x04, 0x08, 0x30, 0x06, 0x01, 0x01, 0xff, 0x02,
    0x01, 0x00, 0x30, 0x1d, 0x06, 0x03, 0x55, 0x1d, 0x0e, 0x04, 0x16, 0x04, 0x14, 0xbc, 0xd4, 0xfd,
    0xe8, 0x80, 0x8a, 0x2d, 0xc9, 0x0b, 0x6d, 0x01, 0xa8, 0xc5, 0xb9, 0xb2, 0x47, 0x33, 0x7e, 0xbd,
    0xda, 0x30, 0x1f, 0x06, 0x03, 0x55, 0x1d, 0x23, 0x04, 0x18, 0x30, 0x16, 0x80, 0x14, 0x7a, 0xed,
    0x7d, 0x6d, 0xc6, 0xb7, 0x78, 0x9d, 0xb2, 0x38, 0x01, 0xa5, 0xe8, 0x4a, 0x8c, 0xb0, 0xa4, 0x0e,
    0x2a, 0x8c, 0x30, 0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x03, 0x48,
    0x00, 0x30, 0x45, 0x02, 0x21, 0x00, 0xc5, 0x07, 0xb8, 0x2a, 0x7b, 0xf9, 0xa3, 0x3a, 0x1b, 0x78,
    0xdc, 0xeb, 0x01, 0xc9, 0x26, 0x92, 0x9e, 0xf3, 0x78, 0x3d, 0x46, 0x8e, 0x69, 0xa2, 0x84, 0xd3,
    0x6a, 0xba, 0xb9, 0x25, 0x1b, 0xef, 0x02, 0x20, 0x0e, 0x6d, 0x7f, 0x76, 0x8d, 0x65, 0xa7, 0x49,
    0xfa, 0x71, 0x2d, 0xda, 0x2b, 0x69, 0x25, 0x35, 0xcd, 0x57, 0x7d, 0x65, 0x01, 0x96, 0xa3, 0xd2,
    0xbf, 0x3b, 0x22, 0x78, 0x8e, 0x75, 0x41, 0x86
};

const size_t g_cryptoauth_root_ca_002_cert_size = sizeof(g_cryptoauth_root_ca_002_cert);


void setup()
{
  // Wire.begin();
  Serial.begin(9600);
  while (!Serial)
    ;

  if (!ECCX08.begin(0x35))
  {
    Serial.println("Failed to communicate with ECC508/ECC608!");
    while (1)
      ;
  }

  if (!ECCX08.locked())
  {
    Serial.println("The ECC508/ECC608 is not locked!");
    while (1)
      ;
  }
}

void loop()
{
  Serial.print("Random number = ");
  Serial.println(ECCX08.random(65535));

  // Serial.println("Root CA: ");
  // for (size_t i = 0; i <= g_cryptoauth_root_ca_002_cert_size; i++)
  // {
  //   Serial.print(g_cryptoauth_root_ca_002_cert[i],HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  
  Serial.println("Root CA: ");
  String out=PEMUtils.base64Encode(g_cryptoauth_root_ca_002_cert, g_cryptoauth_root_ca_002_cert_size, "-----BEGIN CERTIFICATE-----\n", "\n-----END CERTIFICATE-----\n");
  Serial.println(out);

  Serial.println("Signer CA: ");
   String out2=PEMUtils.base64Encode(g_tngtls_cert_template_1_signer, TNGTLS_CERT_TEMPLATE_1_SIGNER_SIZE, "-----BEGIN CERTIFICATE-----\n", "\n-----END CERTIFICATE-----\n");
  Serial.println(out2);
  delay(1000);
}
