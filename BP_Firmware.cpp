/*
****************************************************
***  Tinkerkit Braccio Firmware v1.0             ***
***  Lukas Bilek, (c) 2023 - www.lukasbilek.com  ***
****************************************************
*/

// Inicializace knihoven
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// Inicializace servomotoru
Servo M1; // base
Servo M2; // shoulder
Servo M3; // elbow
Servo M4; // wrist_rot
Servo M5; // wrist_ver
Servo M6; // gripper

// *** Global promenne **
// zakladni pozice ramene
float step_q0 = 90;
float step_q1 = 100;
float step_q2 = 95;
float step_q3 = 95;
float step_M5 = 90;
float step_M6 = 73;

// definice pracovni oblasti
int safe_x1 = 400;  // osa X +
int safe_x0 = -400; // osa X -
int safe_y1 = 400;  // osa Y +
int safe_y0 = -1;   // osa Y + zakladna ramene
int safe_z1 = 1000; // osa Z +
int safe_z0 = 5;    // osa Z - (zem, pri zaporu hrozi nabourani do podlozky)

// promenne pro zadavani souradnic z COM linky
double comm_x, comm_y, comm_z, comm_q4;

// promenne pro vypocet inverzni kinematicke ulohy
float q_main[5], q_alt[5];
double q1, q2, q3, q4, q5;
double q234, zeta, eta;

// promenne delky ramen
float a0, a1, a2, a3;

// promenne pro uzivatelske instrukce
int message;
int gcode;

// promenne pro USB komunikaci - autor: uzivatel DryRun - Arduino forum - https://forum.arduino.cc/t/serial-input-output-of-string-of-numbers-error-in-code/627465
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;
int dataNumber = 0; // new for this version
int usb_message;
int flag;

bool central_stop_flag;

// *** Inicializace funkci ***
void LEDmajak_init(); // interni inicializacni funkce led majaku
void USB_init();      // interni inicializacni funkce USB komunikace
void Servo_init();    // interni inicializacni funkce servomotoru
void CS_init();       // interni inicializacni funkce tlacitka CENTRAL-STOP

void LEDmajak(char barva); // interni funkce pro obsluhu svetelneho majaku
void CENTRAL_STOP();       // interni funkce pro CENTRAL-STOP tlacitko

void M00(int step_delay, float q0, float q1, float q2, float q3, float vM5); // interni pohybova funkce pro linearitu pohybu
void IKU(double x, double y, double z, double uhel_efektoru);                // interni funkce pro vypocet inverzni kinematicke ulohy
void Auto_IKU(double x, double y, double z);                                 // interni funkce pro automaticky vypocet IKU bez zadani uhlu efektoru
void Grip(bool stav);                                                        // interni funkce pro ovladani koncoveho efektoru - zavrit/otevrit
void USB_read();                                                             // interni funkce pro cteni USB komunikace - autor: uzivatel DryRun - Arduino forum - https://forum.arduino.cc/t/serial-input-output-of-string-of-numbers-error-in-code/627465

void G10();         // uzivatelska funkce - otevreni koncoveho efektoru
void G11();         // uzivatelska funkce - zavreni koncoveho efektoru
void user_G12();    // uzivatelska funkce - nastaveni presneho uhlu rozevreni koncoveho efektoru
void G12(int uhel); // uzivatelska funkce - nastaveni presneho uhlu rozevreni koncoveho efektoru

void G28(); // uzivatelska funkce - AutoHOME - presun ramene do domovske polohy
void G90(); // uzivatelska funkce - zadani absolutni pozice souradnicemi - Inverzni Kinematika
void G92(); // uzivatelska funkce - nastaveni aktualni polohy vsech servomotoru

void G101(); // uzivatelska funkce - ukazkova uloha 101 - Presun nakladu
void G102(); // uzivatelska funkce - ukazkova uloha 102 - Stavba veze
void G103(); // uzivatelska funkce - ukazkova uloha 103 - Stavba pyramidy
void G104(); // uzivatelska funkce - ukazkova uloha 104 - Dve veze
void G105(); // uzivatelska funkce - ukazkova uloha 105 - Obihani

// *** Inicializacni funkce ***

void LEDmajak_init()
{
  // LED maják
  pinMode(A5, OUTPUT); // Red
  pinMode(A4, OUTPUT); // Yellow
  pinMode(A3, OUTPUT); // Green
}

void USB_init()
{
  Serial.begin(9600);
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║                  Tinkerkit Braccio - řídicí system                  ║");
  Serial.println("║   Firmware v1.0 --- © 2023 UJEP & Lukas Bilek, www.lukasbilek.com   ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
}

void Servo_init()
{
  // aktivace servo driveru
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  // inicializace Servomotoru
  M1.attach(11);
  M2.attach(10);
  M3.attach(9);
  M4.attach(6);
  M5.attach(5);
  M6.attach(3);
}

void CS_init()
{
  // EMERGENCY STOP
  pinMode(A0, OUTPUT);
  central_stop_flag = false;
}

// *** Interni funkce ***

void LEDmajak(char barva) // interni funkce pro obsluhu svetelneho majaku
{
  digitalWrite(A5, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A3, LOW);

  switch (barva)
  {

  case 'R':
    digitalWrite(A5, HIGH);
    break;

  case 'Y':
    digitalWrite(A4, HIGH);
    break;

  case 'G':
    digitalWrite(A3, HIGH);
    break;

  case 'X':
    digitalWrite(A5, HIGH);
    digitalWrite(A4, HIGH);
    digitalWrite(A3, HIGH);
    break;
  }
}

void CENTRAL_STOP() // interni funkce pro CENTRAL-STOP tlacitko
{
  central_stop_flag = true;
  LEDmajak('R');
  Serial.print("\n■ CENTRAL-STOP ■");

  while (digitalRead(A0) == LOW)
  {
    delay(10);
  }
}

double u2r(double stupne) // pomoc pro prevod stupnu na radiany
{
  return stupne * PI / 180.0;
}

void M00(int step_delay, float q0, float q1, float q2, float q3, float vM5) // interni pohybova funkce pro linearitu pohybu
{
  if (central_stop_flag == false)
  {

    bool smycka = true; // pomocny flag pro kontrolu dosazene pozice

    int cyklus = 0;

    // Test zda jsou zadane uhly fyzicky proveditelne
    if (step_delay > 100)
      step_delay = 100;
    if (step_delay < 10)
      step_delay = 10;
    if (q0 < 0)
      q0 = 0;
    if (q0 > 180)
      q0 = 180;
    if (q1 < 15)
      q1 = 15;
    if (q1 > 165)
      q1 = 165;
    if (q2 < 0)
      q2 = 0;
    if (q2 > 180)
      q2 = 180;
    if (q3 < 0)
      q3 = 0;
    if (q3 > 180)
      q3 = 180;
    if (vM5 > 180)
      vM5 = 180;
    if (vM5 < 0)
      vM5 = 0;

    // Vypočet dilku
    float delta_q0 = abs(q0 - step_q0);
    float delta_q1 = abs(q1 - step_q1);
    float delta_q2 = abs(q2 - step_q2);
    float delta_q3 = abs(q3 - step_q3);
    float delta_M5 = abs(vM5 - step_M5);

    float delta_max = 0;

    if (delta_q0 > delta_max)
      delta_max = delta_q0;

    if (delta_q1 > delta_max)
      delta_max = delta_q1;

    if (delta_q2 > delta_max)
      delta_max = delta_q2;

    if (delta_q3 > delta_max)
      delta_max = delta_q3;

    if (delta_M5 > delta_max)
      delta_max = delta_M5;

    float part_q0 = delta_q0 / delta_max;
    float part_q1 = delta_q1 / delta_max;
    float part_q2 = delta_q2 / delta_max;
    float part_q3 = delta_q3 / delta_max;
    float part_M5 = delta_M5 / delta_max;

    // Dokud neni pozice dosazena
    while (smycka)
    {

      // pohyb kazdeho motoru zvlast o vypocteny dilek dopredu ci dozadu
      if (q0 != step_q0)
      {
        M1.write(step_q0);
        if (q0 > step_q0)
        {
          step_q0 = step_q0 + part_q0;
        }
        if (q0 < step_q0)
        {
          step_q0 = step_q0 - part_q0;
        }
      }

      if (q1 != step_q1)
      {
        M2.write(step_q1);
        if (q1 > step_q1)
        {
          step_q1 = step_q1 + part_q1;
        }
        if (q1 < step_q1)
        {
          step_q1 = step_q1 - part_q1;
        }
      }

      if (q2 != step_q2)
      {
        M3.write(step_q2);
        if (q2 > step_q2)
        {
          step_q2 = step_q2 + part_q2;
        }
        if (q2 < step_q2)
        {
          step_q2 = step_q2 - part_q2;
        }
      }

      if (q3 != step_q3)
      {
        M4.write(step_q3);
        if (q3 > step_q3)
        {
          step_q3 = step_q3 + part_q3;
        }
        if (q3 < step_q3)
        {
          step_q3 = step_q3 - part_q3;
        }
      }

      if (vM5 != step_M5)
      {
        M5.write(step_M5);
        if (vM5 > step_M5)
        {
          step_M5 = step_M5 + part_M5;
        }
        if (vM5 < step_M5)
        {
          step_M5 = step_M5 - part_M5;
        }
      }

      // zpozdeni mezi pohyby udavajici rychlost pohybu
      delay(step_delay);

      // Test zda vsechny motory dosahly sve pozadovane polohy
      if ((q0 == step_q0) && (q1 == step_q1) && (q2 == step_q2) && (q3 == step_q3) && (vM5 == step_M5))
      {
        step_q0 = q0;
        step_q1 = q1;
        step_q2 = q2;
        step_q3 = q3;
        step_M5 = vM5;
        smycka = false;
      }
      else
      {
        smycka = true;
      }

      // cyklus
      cyklus++;

      // test provedených kroků
      if (cyklus > delta_max)
        smycka = false;

      // CENTRAL-STOP test
      if (digitalRead(A0) == LOW)
      {
        CENTRAL_STOP();
        smycka = false;
      }
    }
  }
}

void IKU(double x, double y, double z, double uhel_efektoru) // interni funkce pro vypocet Inverzni Kinematicke Ulohy
{
  // delky ramen [mm]
  double a1 = 71.5;
  double a2 = 125;
  double a3 = 125;
  double a4 = 160;

  // body (klouby robotu)
  double P4[2];
  double P3[2];
  double P2[2];
  double P2_[2];
  double P13[2];
  double P1[2];
  double P0[2];

  // vektory
  double v13[2];
  double v34[2];
  double v32[2];
  double v32_[2];
  double v23[2];
  double v21[2];
  double v2_3[2];
  double v2_1[2];
  double v12[2];
  double v12_[2];
  double v10[2];

  // velikosti vektoru
  double V13;

  // pomocne promenne pro vypocet
  double q0, q1, q1_, q2, q2_, q3, q3_, q4;
  double d4;
  double a, h;

  double d_komp = 1.02; // Kompenzace pruvesu ramen 2%
  z = z + 15;           // Kompenzace pruvesu ramen

  // P0 lezi v pocatku souradnic
  P0[0] = 0;
  P0[1] = 0;

  // P1 lezi v pocatku souřadnic posunuty v Z o a1
  P1[0] = 0;
  P1[1] = a1;

  q4 = uhel_efektoru;

  d4 = sqrt(pow(x, 2) + pow(y, 2)) * d_komp;

  // P4 zadava uzivatel
  P4[0] = d4;
  P4[1] = z;

  // ** Vypočet P3 **
  // uhel efektoru je vetsi než 90
  if (q4 > 90)
  {
    P3[0] = d4 - (a4 * cos(u2r(q4 - 90)));
    P3[1] = z + (a4 * sin(u2r(q4 - 90)));
  }
  // uhel efektoru je presne 90
  else if (q4 == 90)
  {
    P3[0] = d4 - a4;
    P3[1] = z;
  }
  // uhel efektoru je mensi nez 90
  else
  {
    P3[0] = d4 - (a4 * sin(u2r(q4)));
    P3[1] = z - (a4 * cos(u2r(q4)));
  }

  // ** Vypočet P2 **
  v13[0] = P3[0] - P1[0];
  v13[1] = P3[1] - P1[1];
  V13 = sqrt(pow(v13[0], 2) + pow(v13[1], 2));

  a = V13 / 2;
  h = sqrt(pow(a2, 2) - pow(a, 2));

  P13[0] = P1[0] + ((a / V13) * (P3[0] - P1[0]));
  P13[1] = P1[1] + ((a / V13) * (P3[1] - P1[1]));

  // P2
  P2[0] = P13[0] + (h * ((v13[1]) / V13));
  P2[1] = P13[1] - (h * ((v13[0]) / V13));

  // P2'
  P2_[0] = P13[0] - (h * ((v13[1]) / V13));
  P2_[1] = P13[1] + (h * ((v13[0]) / V13));

  // ** Vypočet uhlu **

  q0 = atan2(y, x * -1);

  // q4 je zadan uzivatelem (uhel efektoru)

  // q3
  v34[0] = P4[0] - P3[0]; // d4-d3
  v34[1] = P4[1] - P3[1]; // z4-z3

  v32_[0] = P2_[0] - P3[0]; // d2_-d3
  v32_[1] = P2_[1] - P3[1]; // z2_-z3

  v32[0] = P2[0] - P3[0]; // d2-d3
  v32[1] = P2[1] - P3[1]; // z2-z3

  q3 = acos((((v34[0] * v32[0]) + (v34[1] * v32[1])) / (sqrt(pow(v34[0], 2) + pow(v34[1], 2)) * sqrt(pow(v32[0], 2) + pow(v32[1], 2)))));

  q3_ = acos((((v34[0] * v32_[0]) + (v34[1] * v32_[1])) / (sqrt(pow(v34[0], 2) + pow(v34[1], 2)) * sqrt(pow(v32_[0], 2) + pow(v32_[1], 2)))));

  // q2
  v23[0] = P3[0] - P2[0]; // d3-d2
  v23[1] = P3[1] - P2[1]; // z3-z2

  v21[0] = P1[0] - P2[0]; // d1-d2
  v21[1] = P1[0] - P2[1]; // z1-z2

  v2_3[0] = P3[0] - P2_[0]; // d3-d2
  v2_3[1] = P3[1] - P2_[1]; // z3-z2

  v2_1[0] = P1[0] - P2_[0]; // d1-d2_
  v2_1[1] = P1[1] - P2_[1]; // z1-z2_

  q2 = acos((((v23[0] * v21[0]) + (v23[1] * v21[1])) / (sqrt(pow(v23[0], 2) + pow(v23[1], 2)) * sqrt(pow(v21[0], 2) + pow(v21[1], 2)))));

  q2_ = acos((((v2_3[0] * v2_1[0]) + (v2_3[1] * v2_1[1])) / (sqrt(pow(v2_3[0], 2) + pow(v2_3[1], 2)) * sqrt(pow(v2_1[0], 2) + pow(v2_1[1], 2)))));

  // q1
  v12[0] = P2[0] - P1[0]; // d2-d1
  v12[1] = P2[1] - P1[1]; // z2-z1

  v12_[0] = P2_[0] - P1[0]; // d2_ -d1
  v12_[1] = P2_[1] - P1[1]; // z2_ -z1

  v10[0] = P0[0] - P1[0]; // d0-d1
  v10[1] = P0[1] - P1[1]; // z0-z1

  q1 = acos((((v12[0] * v10[0]) + (v12[1] * v10[1])) / (sqrt(pow(v12[0], 2) + pow(v12[1], 2)) * sqrt(pow(v10[0], 2) + pow(v10[1], 2)))));

  q1_ = acos((((v12_[0] * v10[0]) + (v12_[1] * v10[1])) / (sqrt(pow(v12_[0], 2) + pow(v12_[1], 2)) * sqrt(pow(v10[0], 2) + pow(v10[1], 2)))));

  // **Prepocet udaju z rad do deg + prevod do Braccio systemu**
  // primarni poloha
  q_main[0] = degrees(q0);
  q_main[1] = degrees(q1_) - 90;
  q_main[2] = degrees(q2_) - 90;
  q_main[3] = degrees(q3_) - 90;
  q_main[4] = q4;

  // sekundarni poloha
  q_alt[0] = degrees(q0);
  q_alt[1] = degrees(q1) - 90;
  q_alt[2] = 270 - degrees(q2_);
  q_alt[3] = degrees(q3) - 90;
  q_alt[4] = q4;
}

void Auto_IKU(double x, double y, double z) // interni funkce pro automaticky vypocet IKU bez zadani uhlu efektoru
{
  // pomocne flagy pro kontrolu platnosti vypoctenych poloh
  bool poloha_main_neplatna = false;
  bool poloha_alt_neplatna = false;

  // kontrola zadanych souradnic, zda jsou v prostoru
  if (x > safe_x1)
    x = safe_x1;
  if (x < safe_x0)
    x = safe_x0;
  if (y > safe_y1)
    y = safe_y1;
  if (y < safe_y0)
    y = safe_y0;
  if (z > safe_z1)
    z = safe_z1;
  if (z < safe_z0)
    z = safe_z0;

  // hledani platneho uhlu efektoru pro dany bod v prostoru
  for (int i = 180; i > 0; i--)
  {
    // IKU pro aktualni uhel z for loop
    IKU(x, y, z, i);

    // test platnosti
    poloha_main_neplatna = false;
    poloha_alt_neplatna = false;

    // test primarni polohy ramena
    if (q_main[1] < 15 || q_main[1] > 165)
      poloha_main_neplatna = true;

    if (q_main[2] < 0 || q_main[2] > 180)
      poloha_main_neplatna = true;

    if (q_main[3] < 0 || q_main[3] > 180)
      poloha_main_neplatna = true;

    if (isnan(q_main[1]) || isnan(q_main[2]) || isnan(q_main[3])) // detekce NaN
    {
      poloha_main_neplatna = true;
    }

    if (q_main[1] == 0 || q_main[2] == 0 || q_main[3] == 0) // detekce nulove polohy(neni-li bod dosazen)
      poloha_main_neplatna = true;

    // test alternativni polohy ramena
    if (q_alt[1] < 15 || q_alt[1] > 165)
      poloha_alt_neplatna = true;

    if (q_alt[2] < 0 || q_alt[2] > 180)
      poloha_alt_neplatna = true;

    if (q_alt[3] < 0 || q_alt[3] > 180)
      poloha_alt_neplatna = true;

    if (isnan(q_alt[1]) || isnan(q_alt[2]) || isnan(q_alt[3]))
    { // detekce NaN
      poloha_alt_neplatna = true;
    }

    if (q_alt[1] == 0 || q_alt[2] == 0 || q_alt[3] == 0) // detekce nulove polohy(neni-li bod dosazen)
      poloha_alt_neplatna = true;

    // je-li v pro uhel i nalezena alespon jedna platna poloha, ukoncit hledaci cyklus
    if (poloha_main_neplatna == false || poloha_alt_neplatna == false)
      break;
  }

  // je-li hlavni poloha platna, zavolej funkci pro pohyb
  if (poloha_main_neplatna == false)
  {
    Serial.print("Poloha nalezena :)");
    M00(30, q_main[0], q_main[1], q_main[2], q_main[3], 90);
  }
  else if (poloha_alt_neplatna == false) // je-li alternativni poloha platna, zavolej funkci pro pohyb
  {
    Serial.print("Poloha nalezena :)");
    M00(30, q_alt[0], q_alt[1], q_alt[2], q_alt[3], 90);
  }
  else // jinak vypis chybovou hlasku
  {
    Serial.print("Nedosažitelný bod :(");
  }
}

void Grip(bool stav) // interni funkce pro ovladani koncoveho efektoru - zavrit/otevrit
{
  int uhel;

  // je-li 1 - zavrit kleste/efektor
  if (stav == true)

    uhel = 10; // hodnota z manualu pro zavreni efektoru

  // pohybovy algoritmus z funkce M00
  while (uhel < 73)
  {
    M6.write(uhel);
    uhel++;
    delay(10);
  }

  // je-li 0 - otevrit kleste/efektor
  if (stav == false)

    uhel = 73; // hodnota z manualu pro rozevreni efektoru

  // pohybovy algoritmus z funkce M00
  while (uhel > 10)
  {
    M6.write(uhel);
    uhel--;
    delay(10);
  }

  delay(500);
}

void USB_read() // interni funkce pro cteni USB komunikace - autor: uzivatel DryRun - Arduino forum - https://forum.arduino.cc/t/serial-input-output-of-string-of-numbers-error-in-code/627465
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  if (Serial.available() > 0)
  {
    rc = Serial.read();

    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars)
      {
        ndx = numChars - 1;
      }
    }
    else
    {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }

  if (newData == true)
  {
    dataNumber = 0;                   // new for this version
    dataNumber = atoi(receivedChars); // new for this version
    // Serial.println(receivedChars)
    usb_message = dataNumber;
    newData = false;
  }
}

// *** Uzivatelske funkce ***

void G10() // uzivatelska funkce - otevreni koncoveho efektoru
{
  if (central_stop_flag == false)
  {
    LEDmajak('Y');

    Serial.print("G10 ");

    // pohybovy algoritmus z funkce M00
    while (step_M6 > 10)
    {
      M6.write(step_M6);
      step_M6--;
      delay(10);
    }
  }
}

void G11() // uzivatelska funkce - zavreni koncoveho efektoru
{
  if (central_stop_flag == false)
  {
    LEDmajak('Y');

    Serial.print("G11 ");

    // pohybovy algoritmus z funkce M00
    while (step_M6 < 90)
    {
      M6.write(step_M6);
      step_M6++;
      delay(10);
    }
  }
}

void user_G12() // uzivatelska funkce - nastaveni presneho uhlu rozevreni koncoveho efektoru
{
  if (central_stop_flag == false)
  {
    LEDmajak('Y');

    Serial.print("G12 ");

    int uhel_m6;
    int flag = 1;

    if (flag == 1)
    {
      Serial.print("\nuhel M6: ");
      flag = 0;
    }

    // čtení bufferu
    while (true)
    {
      usb_message = 0;
      USB_read();
      uhel_m6 = usb_message;
      if (usb_message != 0)
      {
        flag = 1;
        Serial.print(usb_message);
        Serial.print("°");
        break;
      }
      delay(1);
    }

    // pohybovy algoritmus z funkce M00
    while (step_M6 != uhel_m6)
    {
      M6.write(step_M6);
      if (uhel_m6 > step_M6)
      {
        step_M6 = step_M6 + 1;
      }
      if (uhel_m6 < step_M6)
      {
        step_M6 = step_M6 - 1;
      }
      delay(30);
    }
  }
}

void G12(int uhel) // uzivatelska funkce - nastaveni presneho uhlu rozevreni koncoveho efektoru
{
  if (central_stop_flag == false)
  {
    // pohybovy algoritmus z funkce M00
    while (step_M6 != uhel)
    {
      M6.write(step_M6);
      if (uhel > step_M6)
      {
        step_M6 = step_M6 + 1;
      }
      if (uhel < step_M6)
      {
        step_M6 = step_M6 - 1;
      }
      delay(30);
    }
  }
}

void G28() // uzivatelska funkce - AutoHOME - presun ramene do domovske polohy
{
  if (central_stop_flag == false)
  {
    Serial.print("G28 ");
    LEDmajak('Y');

    G11();
    M00(30, 179, 140, 15, 25, 90);
  }
}

void G90() // uzivatelska funkce - zadani absolutni pozice souradnicemi - Inverzni Kinematika
{
  LEDmajak('Y');

  Serial.print("G90 ");

  // osetrení pouze 1 vypisu
  if (flag == 1)
  {
    Serial.print("\nX: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    comm_x = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      Serial.print(usb_message);
      break;
    }
    delay(1);
  }

  // osetrení pouze 1 vypisu
  if (flag == 1)
  {
    Serial.print("\nY: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    comm_y = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      Serial.print(usb_message);
      break;
    }
    delay(1);
  }

  // osetrení pouze 1 vypisu
  if (flag == 1)
  {
    Serial.print("\nZ: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    comm_z = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      Serial.print(usb_message);
      break;
    }
    delay(1);
  }

  Serial.println("\nProvádím G90 ...");

  Auto_IKU(comm_x, comm_y, comm_z);
}

void G92() // uzivatelska funkce - nastaveni aktualni polohy vsech servomotoru
{
  LEDmajak('Y');

  Serial.print("G92 ");

  int q0_, q1_, q2_, q3_;

  int flag = 1;

  if (flag == 1)
  {
    Serial.print("\nq0: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    q0_ = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      Serial.print(usb_message);
      Serial.print("°");
      break;
    }
    delay(1);
  }

  if (flag == 1)
  {
    Serial.print("\nq1: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    q1_ = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      Serial.print(usb_message);
      Serial.print("°");
      break;
    }
    delay(1);
  }

  if (flag == 1)
  {
    Serial.print("\nq2: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    q2_ = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      Serial.print(usb_message);
      Serial.print("°");
      break;
    }
    delay(1);
  }

  if (flag == 1)
  {
    Serial.print("\nq3: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    q3_ = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      Serial.print(usb_message);
      Serial.print("°");
      break;
    }
    delay(1);
  }
  Serial.print("\nG92: Úhly přijaty");

  M00(30, q0_, q1_, q2_, q3_, 90);
}

// *** Ukazkove ulohy ***

void G101() // uzivatelska funkce - ukazkova uloha 101 - Presun nakladu
{
  Serial.println("G101");

  LEDmajak('Y');

  M00(30, 126, 65, 23, 123, 90); // X200 Y300
  G10();                         // gripper otevřít
  M00(50, 126, 30, 33, 148, 90);
  G11(); // gripper zavřít
  M00(30, 126, 65, 23, 123, 90);

  M00(30, 90, 72, 78, 77, 90);

  M00(30, 56, 65, 23, 123, 90);
  delay(200);
  M00(50, 56, 30, 33, 148, 90);
  G10(); // gripper otevřít

  M00(10, 56, 65, 23, 123, 90);
  M00(10, 56, 65, 23, 123, 0);
  M00(10, 56, 65, 23, 123, 180);
  M00(10, 56, 65, 23, 123, 90);

  G10();
  M00(50, 56, 30, 33, 148, 90);
  G11();
  M00(30, 56, 65, 23, 123, 90);

  M00(30, 90, 72, 78, 77, 90);

  M00(30, 130, 65, 23, 123, 90);
  delay(200);
  M00(50, 130, 30, 33, 148, 90);
  G10();
  M00(30, 130, 65, 23, 123, 90);

  G28();
}

void G102() // uzivatelska funkce - ukazkova uloha 102 - Stavba veze
{

  Serial.println("G102");

  LEDmajak('Y');

  M00(30, 90, 65, 90, 60, 90); // 0 300 h200
  G10();
  M00(40, 90, 55, 1, 155, 90); // 0 300 h10
  G11();
  M00(30, 90, 65, 90, 60, 90);   // 0 300 h200
  M00(30, 135, 65, 90, 60, 90);  // 200 300 h200
  M00(40, 135, 25, 50, 130, 90); // 200 200 h40
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(30, 52, 65, 90, 60, 90);  // -200 300 h200
  M00(40, 52, 35, 30, 145, 90); // -200 300 h40
  G11();
  M00(30, 52, 65, 90, 60, 90);   //-200 300 h200
  M00(30, 135, 65, 90, 60, 90);  // 200 300 h200
  M00(40, 135, 40, 33, 132, 90); // 200 300 h50
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(30, 15, 65, 90, 60, 90);  // -300 100 h200
  M00(40, 15, 60, 3, 147, 90);  // -300 100 h50
  G11();
  M00(30, 15, 65, 90, 60, 90);   // -300 100 h200
  M00(30, 135, 65, 90, 60, 90);  // 200 300 h200
  M00(40, 135, 50, 50, 105, 90); // 200 300 h100
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(30, 90, 65, 90, 60, 90);  // 0 300 h200
  M00(40, 90, 43, 1, 165, 90);  // 0 300 h20
  G11();
  M00(30, 90, 65, 90, 60, 90);  // 0 300 h200
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(40, 135, 42, 75, 90, 90); // 200 300 h150
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(30, 52, 65, 90, 60, 90);  // -200 300 h200
  M00(40, 52, 28, 37, 145, 90); // -200 300 h20
  G11();
  M00(30, 52, 65, 90, 60, 90);  // -200 300 h200
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(40, 135, 45, 90, 72, 90); // 200 300 h180
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(30, 15, 65, 90, 60, 90);  // -300 100 h200
  M00(40, 15, 40, 3, 165, 90);  // -300 100 h20
  G11();
  M00(30, 15, 65, 90, 60, 90);  // -300 100 h200
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(40, 135, 45, 92, 82, 90); // 200 300 h220
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200

  G28();
}

void G103() // uzivatelska funkce - ukazkova uloha 103 - Stavba pyramidy
{
  Serial.println("G103");

  LEDmajak('Y');

  M00(30, 15, 65, 90, 60, 90); //-300 100 h200
  G10();
  M00(40, 15, 70, 10, 135, 90); //-300 100 h3
  G11();
  M00(30, 15, 65, 90, 60, 90);  //-300 100 h200
  M00(30, 101, 65, 90, 60, 90); // 20 300 h200
  M00(40, 101, 40, 1, 165, 90); // 20 300 h1
  G12(65);
  M00(30, 101, 65, 90, 60, 90); // 20 300 h200
  M00(30, 15, 65, 90, 60, 90);  //-300 100 h200
  G10();
  M00(40, 15, 50, 10, 150, 90); //-300 100 h2
  G11();
  M00(30, 15, 65, 90, 60, 90); //-300 100 h200
  M00(30, 85, 65, 90, 60, 90); //-20 300 h200
  M00(40, 85, 40, 1, 165, 90); //-20 300 h1
  M00(40, 92, 40, 1, 165, 90); //-20 300 h1
  G12(65);
  M00(30, 90, 65, 90, 60, 90); //-20 300 h200
  M00(30, 15, 65, 90, 60, 90); //-300 100 h200
  G10();
  M00(40, 15, 40, 3, 165, 90); //-300 100 h1
  G11();
  M00(30, 15, 65, 90, 60, 90); //-300 100 h200
  M00(30, 97, 65, 90, 60, 90); // 0 300 h200
  M00(40, 97, 62, 1, 145, 90); // 0 300 h200
  G10();
  M00(30, 97, 65, 90, 60, 90); //
  G28();
}

void G104() // uzivatelska funkce - ukazkova uloha 104 - Dve veze
{
  Serial.println("G104");

  LEDmajak('Y');

  M00(30, 86, 65, 90, 60, 90); // 0 300 h200
  G10();
  M00(30, 86, 80, 45, 90, 90); // 0 300 h6
  G11();
  M00(30, 86, 65, 90, 60, 90);   // 0 300 h200
  M00(30, 135, 65, 90, 60, 90);  // 200 300 h200
  M00(30, 135, 25, 45, 135, 90); // 200 300 h1
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h1
  M00(30, 86, 65, 90, 60, 90);  // 0 300 h200
  M00(30, 86, 85, 28, 100, 90); // 0 300 h5
  G11();
  M00(30, 86, 65, 90, 60, 90);  // 0 300 h200
  M00(30, 52, 65, 90, 60, 90);  //-200 300 h200
  M00(30, 52, 25, 45, 135, 90); //-200 300 h1
  G10();
  M00(30, 52, 65, 90, 60, 90);  //-200 300 h200
  M00(30, 94, 65, 90, 60, 90);  // 0 300 h200 P
  M00(30, 94, 85, 10, 120, 90); // 0 300 h4 P
  G11();
  M00(30, 94, 65, 90, 60, 90);   // 0 300 h200 P
  M00(30, 135, 65, 90, 60, 90);  // 200 300 h200
  M00(30, 135, 40, 33, 132, 90); // 200 300 h2
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(30, 86, 65, 90, 60, 90);  // 0 300 h200
  M00(30, 86, 75, 1, 140, 90);  // 0 300 h3
  G11();
  M00(30, 86, 65, 90, 60, 90);  // 0 300 h200
  M00(30, 52, 65, 90, 60, 90);  //-200 300 h200
  M00(30, 52, 35, 30, 145, 90); //-200 300 h2
  G10();
  M00(30, 52, 65, 90, 60, 90); //-200 300 h200
  M00(30, 94, 65, 90, 60, 90); // 0 300 h200 P
  M00(30, 94, 60, 1, 150, 90); // 0 300 h2 P
  G11();
  M00(30, 94, 65, 90, 60, 90);   // 0 300 h200 P
  M00(30, 135, 65, 90, 60, 90);  // 200 300 h200
  M00(30, 135, 50, 35, 125, 90); // 200 300 h3
  G10();
  M00(30, 135, 65, 90, 60, 90); // 200 300 h200
  M00(30, 86, 65, 90, 60, 90);  // 0 300 h200
  M00(30, 86, 40, 1, 170, 90);  // 0 300 h1
  G11();
  M00(30, 86, 65, 90, 60, 90);  // 0 300 h200
  M00(30, 52, 65, 90, 60, 90);  //-200 300 h200
  M00(30, 52, 50, 35, 125, 90); //-200 300 h3
  G10();
  M00(30, 52, 65, 90, 60, 90); //-200 300 h200
  G28();
}

void G105() // uzivatelska funkce - ukazkova uloha 105 - Obihani
{
  Serial.println("G105");

  LEDmajak('Y');

  G11();
  M00(30, 86, 65, 90, 60, 90);

  M00(20, 86, 49, 95, 33, 90);
  M00(20, 86, 73, 50, 60, 90);
  M00(20, 86, 80, 5, 130, 90);
  M00(20, 86, 60, 1, 165, 90);
  M00(20, 86, 50, 1, 180, 90);

  delay(1000);

  M00(20, 86, 60, 1, 165, 90);
  M00(20, 86, 80, 5, 130, 90);
  M00(20, 86, 73, 50, 60, 90);
  M00(20, 86, 49, 95, 33, 90);

  delay(1000);

  M00(20, 86, 73, 50, 60, 90);
  M00(20, 86, 80, 5, 130, 90);
  M00(20, 86, 60, 1, 165, 90);
  M00(20, 86, 50, 1, 180, 90);

  delay(1000);

  M00(20, 86, 60, 1, 165, 90);
  M00(20, 86, 80, 5, 130, 90);
  M00(20, 86, 73, 50, 60, 90);
  M00(20, 86, 49, 95, 33, 90);

  G28();
}

// *** Systemove procedury ***

void setup()
{
  LEDmajak_init(); // inicializace majaku
  LEDmajak('X');   // rozsviceni vsech barev
  USB_init();      // inicializace usb
  Servo_init();    // inicializace servomotoru
  CS_init();       // inicializace central-stop tlacitka

  delay(2000);

  // SOFTSTART ramene
  M00(30, 91, 101, 96, 96, 91);
}

void loop()
{

  LEDmajak('G');
  delay(10);

  flag = 1; // pomocna flag
  central_stop_flag = false;

  // Cteni uzivatelskeho vstupu
  // osetreno pouze 1 vypisu
  if (flag == 1)
  {
    Serial.println("\n═══════════════════════════════════════");
    Serial.print("Zadejte G-code: ");
    flag = 0;
  }

  // cteni bufferu
  while (true)
  {
    usb_message = 0;
    USB_read();
    gcode = usb_message;
    if (usb_message != 0)
    {
      flag = 1;
      break;
    }
    delay(1);
  }

  delay(100);

  // *VYBER INSTRUKCE DLE ZADANEHO GCODE*
  switch (gcode)
  {

  case 10:
    G10();
    break;

  case 11:
    G11();
    break;

  case 12:
    user_G12();
    break;

  case 90:
    G90();
    break;

  case 92:
    G92();
    break;

  case 28:
    G28();
    break;

  case 101:
    G101();
    break;

  case 102:
    G102();
    break;

  case 103:
    G103();
    break;

  case 104:
    G104();
    break;

  case 105:
    G105();
    break;

  case 69:
    Serial.print("G");
    Serial.print(usb_message);
    Serial.print(" => ");
    Serial.print("( ͡° ͜ʖ ͡° )");
    break;

    case 420:
    Serial.print("G");
    Serial.print(usb_message);
    Serial.print(" => ");
    Serial.print("BLAZE IT 🌿🚬");
    break;

  default:
    Serial.print("G");
    Serial.print(usb_message);
    Serial.print(" => Neplatný G-code :/");
    break;
  }
}