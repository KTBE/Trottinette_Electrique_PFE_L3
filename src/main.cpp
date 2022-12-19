/************************************************************************************/
/*--------------------Code Pour Trottinettes Intelligente---------------------------*/
/************************************************************************************/
/*--------------------Edité par KIEMA Teewendé Boris Elisée-------------------------*/
/************************************************************************************/
/*---------------------------Projet de Fin d'Etudes---------------------------------*/
/************************************************************************************/
/*------------------------------------2020------------------------------------------*/

#include "BluetoothSerial.h"

/***************************************Mutex****************************************/
SemaphoreHandle_t xMutex; // declaration de la semaphore de type mutex
// les mutex permettent de faire une exclusion
// lorsqu'on utilise une ressource partagée

/*********************************Declaration dutube**********************************/
QueueHandle_t tube_envoie;

/*****************************Déclaration desvariables********************************/
//////////////////////////////////////////pour la vitesse:
#define emetteur 19             // led IR emettrice
const byte recepteur = 35;      // photo-diode receptrice
const float rayonRoue = 0.105;  // à modifier selon le raton de la roue à utiliser
volatile int compteur;          // compteur
int nombreDeFois = 0;           // nombre de fois que la partie blanche du rayon passe
int nombreDeRotationCroix;      //
float vitesseAngulaire;         //
float vitesse;                  //
long tempsDeFonctionnement;     //
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

///////////////////////////////////////////////pour le moteur
#define resistor A13            // relié au volant
#define motor 22                // relié au moteur pour regler la vitesse de rotation
#define bp1 23                  // bouton demarré
#define bp2 21                  // bouton arrêt
int k=0;                        //
int k1=0;                       //
                                //
uint32_t nombreTourRoue = 0;    //variable pour la distance parcourrue
uint32_t distance;              //
                                //
boolean variable;               //variable qui sera vraie (true) si l'on
                                //reçoit une donnée du tube

//////////////////////////////////////////******parametres du pwm
const int frequency = 5000;     // frequence du pwm
const int ledChannel = 0;       // numero du canal utilisé
const int resolution = 8;       // nombre de bits sur lequel le pwm est codé

///////////////////////////////////////////////pour eclairage automatique
#define ldr A12                 // gpio2
#define lampe 32                // gpio32
const int seuil_lumiere = 300 ; // valeur en dessous de laquelle, on allume la lampe

/////////////////////////////////////////////pour batterie
#define batterie A14                   // gpio13 (relié à la batterie)
const uint16_t niveau_Haut = 3900;    // correspond à une tension de 24v
const uint16_t niveau_Milieu = 3656;  // correspond à une tension de 22.5v
const uint16_t niveau_Bas = 3494;     // correspond à une tension de 21.5v
uint16_t etatBat = 0;

//////////////////////////////////////////pour Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

/******************************************************************************************/
/*----------------------------Différentes tâches------------------------------------------*/
/******************************************************************************************/
void IRAM_ATTR handleInterrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  compteur++;
  portEXIT_CRITICAL_ISR(&mux);
}

void tache_vitesse(void *pvParameter)
{
  while(1)
  {
    digitalWrite (emetteur, HIGH);
    if ( compteur > 0 )
    {
      portENTER_CRITICAL(&mux);
      compteur--;
      portEXIT_CRITICAL(&mux);
      nombreDeFois = 1;
        while( (millis() - tempsDeFonctionnement) < 1998 )
        {
            if ( compteur > 0 )
            {
              portENTER_CRITICAL(&mux);
              compteur--;
              portEXIT_CRITICAL(&mux);
              nombreDeFois++; // nombre de tours en 2secondes
            }
        }
    }
    tempsDeFonctionnement = millis(); // à envoyer au zigbee

    //rotation par minute (rpm)de la roue
    nombreDeRotationCroix = (nombreDeFois * 30); //nombre de tour en 2seconde
                                                 //multiplié par 30 pour avoir
                                                 //le rpm
    nombreTourRoue += nombreDeFois;
    nombreDeFois = 0;
    //convertion de rpm (rotation per minute) en km/h
    vitesseAngulaire = (2*3.14*nombreDeRotationCroix)/60; // calcul de la vitesse angulaire
    //uint8_t c=10;
    vitesse = vitesseAngulaire * rayonRoue; //+= c;
    //distance parcourue
    distance = nombreTourRoue * rayonRoue;

    // envoie de la vitesse et temps de fonctionnement
    xSemaphoreTake(xMutex, portMAX_DELAY);
    SerialBT.println("vitesse=" + String (vitesse));
    delay(15);
    SerialBT.println("distance=" + String (distance));
    xSemaphoreGive(xMutex);
    vTaskDelay(5);
  }
}

void tache_moteur(void *pvParameter)
{
  for(;;)
  {
    int start = digitalRead(bp1);
    int Stop = digitalRead(bp2);
    uint8_t item = 0;
    boolean variable;
    Serial.println(start);
    Serial.println(Stop);

    if(xQueueReceive(tube_envoie,&item, 1000) == pdPASS)
    {
      variable = true;
    }

    if( variable == true )
    {
      if(start == 0)
      {
        k=1;
        k1=0;
      }

      if(Stop == 0)
      {
        k=0;
        k1=1;
      }
      
      if((k==1)&&(k1==0))
      {
        int value = analogRead(resistor);
        delay(100);
        Serial.println(value);
        ledcWrite(0, value);
      }
      else
      {
        ledcWrite(0, 0);
      }
    }
  }
}

void tache_batterie(void *pvParameter)
{
  for(;;)
  {
    uint16_t valBatterie = analogRead (batterie);
    etatBat = map (valBatterie, 0, 4095, 0, 100); // pour changer d'echelle et ramener sur
                                                  // l'echelle [0-100]
    xSemaphoreTake(xMutex, portMAX_DELAY);
    SerialBT.println("batterie=" + String(etatBat));
    xSemaphoreGive(xMutex);
    delay(60000); // attente 1 min
  }
  vTaskDelay (1); //
}

void eclairage(void *pvParameter)
{
  int etat_ldr = analogRead(ldr);
  boolean etat_lampe;
  etat_lampe = digitalRead(lampe);
  for (;;)
  {
    if ( etat_ldr <= seuil_lumiere)
    {
      digitalWrite(lampe, HIGH);
      xSemaphoreTake(xMutex, portMAX_DELAY);
      SerialBT.println("lampe=ON");
      xSemaphoreGive(xMutex);
    }

    if (etat_ldr>seuil_lumiere)
    {
      digitalWrite(lampe, LOW);
      xSemaphoreTake(xMutex, portMAX_DELAY);
      SerialBT.println("lampe=OFF");
      xSemaphoreGive(xMutex);
    }

    if(etat_lampe == LOW)
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      SerialBT.println("1111111lampe=OFF");
      xSemaphoreGive(xMutex);
    }
    
    if(etat_lampe == HIGH)
    {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      SerialBT.print("lampe=ON");
      xSemaphoreGive(xMutex);
    }
    
    delay(30000);
  }
}

void tache_xbee(void *pvParameter)
{
  for(;;)
  {
    String message;
    message = SerialBT.readString();
    if(message.length()>0)
    {
      if (message == "ok ")
      {
        uint8_t x = 1;
        xQueueSend(tube_envoie, &x, portMAX_DELAY);
      }
      else
      {
        Serial.println(message);
      }
   }
   vTaskDelay(1);
  }
}

void setup ()
{
  Serial.begin(115200);

  //vitesse
  pinMode(recepteur, INPUT);
  attachInterrupt(digitalPinToInterrupt(recepteur), handleInterrupt, FALLING);
  
  //moteur
  ledcSetup(ledChannel, frequency, resolution);
  ledcAttachPin(motor, ledChannel);
  pinMode (bp1, INPUT_PULLUP);
  pinMode (bp2, INPUT_PULLUP);
  
  //eclairage
  pinMode (lampe, OUTPUT);
  digitalWrite(lampe, LOW);
  
  //Creation de tube
  tube_envoie = xQueueCreate(2, sizeof(uint8_t));

  //bluetooth
  SerialBT.begin("Trottinette");// nom du Bluetooth
  
  //Creation de Mutex
  xMutex = xSemaphoreCreateMutex();
  
  //Creation de tâches
  xTaskCreatePinnedToCore(tache_vitesse, "vitesse", 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tache_batterie, "batterie", 10240, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(tache_moteur, "moteur", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(eclairage, "eclairage", 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tache_xbee, "localisation", 1024, NULL, 4, NULL,1);

}

void loop()
{
}
//FIN;
//by KTBE;
//THANKS GOD;
//2020