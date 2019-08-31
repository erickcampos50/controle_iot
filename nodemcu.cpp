//Para testes de recepção de infravermelho
#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#if DECODE_AC
#include <ir_Daikin.h>
#include <ir_Fujitsu.h>
#include <ir_Gree.h>
#include <ir_Haier.h>
#include <ir_Kelvinator.h>
#include <ir_Midea.h>
#include <ir_Toshiba.h>
#endif  // DECODE_AC

#define RECV_PIN 14 //Equivale ao D5 do nodemcu
#define BAUD_RATE 115200
#define CAPTURE_BUFFER_SIZE 1024

#if DECODE_AC
#define TIMEOUT 50U  // Some A/C units have gaps in their protocols of ~40ms.
                     // e.g. Kelvinator
                     // A value this large may swallow repeats of some protocols
#else  // DECODE_AC
#define TIMEOUT 15U  // Suits most messages, while not swallowing many repeats.
#endif  // DECODE_AC
#define MIN_UNKNOWN_SIZE 100

IRrecv irrecv(RECV_PIN, CAPTURE_BUFFER_SIZE, TIMEOUT, true);
decode_results results;  // Somewhere to store the results
void dumpACInfo(decode_results *results) {
  String description = "";
#if DECODE_DAIKIN
  if (results->decode_type == DAIKIN) {
    IRDaikinESP ac(0);
    ac.setRaw(results->state);
    description = ac.toString();
  }
#endif  // DECODE_DAIKIN
#if DECODE_FUJITSU_AC
  if (results->decode_type == FUJITSU_AC) {
    IRFujitsuAC ac(0);
    ac.setRaw(results->state, results->bits / 8);
    description = ac.toString();
  }
#endif  // DECODE_FUJITSU_AC
#if DECODE_KELVINATOR
  if (results->decode_type == KELVINATOR) {
    IRKelvinatorAC ac(0);
    ac.setRaw(results->state);
    description = ac.toString();
  }
#endif  // DECODE_KELVINATOR
#if DECODE_TOSHIBA_AC
  if (results->decode_type == TOSHIBA_AC) {
    IRToshibaAC ac(0);
    ac.setRaw(results->state);
    description = ac.toString();
  }
#endif  // DECODE_TOSHIBA_AC
#if DECODE_GREE
  if (results->decode_type == GREE) {
    IRGreeAC ac(0);
    ac.setRaw(results->state);
    description = ac.toString();
  }
#endif  // DECODE_GREE
#if DECODE_MIDEA
  if (results->decode_type == MIDEA) {
    IRMideaAC ac(0);
    ac.setRaw(results->value);  // Midea uses value instead of state.
    description = ac.toString();
  }
#endif  // DECODE_MIDEA
#if DECODE_HAIER_AC
  if (results->decode_type == HAIER_AC) {
    IRHaierAC ac(0);
    ac.setRaw(results->state);
    description = ac.toString();
  }
#endif  // DECODE_HAIER_AC
#if DECODE_HAIER_AC_YRW02
  if (results->decode_type == HAIER_AC_YRW02) {
    IRHaierACYRW02 ac(0);
    ac.setRaw(results->state);
    description = ac.toString();
  }
#endif  // DECODE_HAIER_AC_YRW02
  // If we got a human-readable description of the message, display it.
  if (description != "")  Serial.println("Mesg Desc.: " + description);
}

//===================================================================//


//Utilizado somente pra avaliar o total de ram disponivel
#include "user_interface.h"

// Import required libraries
#include <ESP8266WiFi.h> //Necessária para o Wifi, mesmo com o Wifimanager sendo utilizado
#include <PubSubClient.h>
#include <aREST.h>

//Utilizado para gerar um dominio para acesso alternativo ao IP, assim é possível gerenciar melhor o acesso em redes cujo IP possa variar (ainda mais considerando que ele vai resevar a cada vez que a memoria baixar de 5.000 bytes)
#include <ESP8266mDNS.h>        // Include the mDNS library

//Bibliotecas referentes a emissão de IR
#include <IRremoteESP8266.h>
#include <IRsend.h>

//Gerenciamento do WIFI
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager Padrão de ativação do wifi http://192.168.4.1/wifisave?s=SSID&p=senha

//Biblioteca para viabilizar atualizações OTA
#include <ArduinoOTA.h>

//Biblioteca para converter strings para array de modo fácil e permitir a entrada de codigos diversos de AC sem precisar inserir no codigo compilado
#include <StringSplitter.h> //essa função foi editada aumentando o MAX de 5 para 200

//para simplificar a função de sinalização do LED
#include <Ticker.h> //https://github.com/esp8266/Arduino/blob/master/libraries/Ticker/examples/TickerBasic/TickerBasic.ino
Ticker ticker;
#define LED_Status 0 //LED de sinalização para os eventos (D3)

#define porta_reset 15 //porta para realizar o reset das configurações do Wifi sem depender da IDE ou do funcionamento da rede (D8)
//Porta utilizada para emissão de IR, essa é a melhor até o momento. Não usar em hipotese alguma D4 ou irá causar um bug fisico e o ESP entrará em loop
#define IR_LED 4  // ESP8266 GPIO pin to use for IR. Recommended: 4 (D2).

IRsend irsend(IR_LED);  // Set the GPIO to be used to sending the message.
// Clients
WiFiClient espClient;
PubSubClient client(espClient);

// Create aREST instance
aREST rest = aREST(client);

// Unique ID to identify the device for cloud.arest.io
char* device_id = "node03";

// Variables to be exposed to the API
//String local_ip = "";  //Variável criada para expor o IP local da placa para o utilizador na internet. Mas foi descartada porque a interface do Arest não funcinoa para emissão de códigos de função
String ir_recv = ""; //Variável criada para expor o último código de IR recebido para a comunicação web
String reg = ""; //Variável criada para armazenar o registro de data, hora e estado do ultimo acionamento

//Variavel utilizada pra quebrar o loop while quando houver travamento em !clientlocal.avaliable()
int timeout = 0;


// The port to listen for incoming TCP connections
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Functions
void callback(char* topic, byte* payload, unsigned int length);

  void tick() //Utilizada para piscar complementar o LED de sinalização, ainda em testes
{
  //toggle state
  int state = digitalRead(LED_Status);  // get the current state of GPIO1 pin
  digitalWrite(LED_Status, !state);     // set pin to the opposite state
}

//====================================================================//
//Modelo inicial de acionamento quando ainda nao era possivel enviar comandos completos como parametro para a função

//Variaveis armazenando os codigos das funcoes
//#define RAW_DATA_LEN2 198 //Valor exclusivo para o AC Rheem
//uint16_t rheem_power[RAW_DATA_LEN2] = {6100, 7400, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 1700, 550, 580, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 1700, 550, 580, 550, 1700, 550, 1700, 550, 580, 550, 580, 550, 580, 550, 580, 550, 580, 550, 1700, 550, 580, 550, 580, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 580, 550, 1700, 550, 1700, 550, 7400, 550};

//int rheem_power_on(String b) {
//  irsend.sendRaw(rheem_power, RAW_DATA_LEN2, 38); //Pass the buffer,length, optionally frequency
//  Serial.print(millis());
//  Serial.println(": rheem_power_on");
//  ticker.attach(0.05, tick);
//  delay(300);
//  ticker.detach();
//  uint32_t free = system_get_free_heap_size();
//  Serial.println(free);
//  return 0;
//}
//====================================================================//


int ir_send(String a) {

  Serial.println("O sinal recebido é " + a);
  StringSplitter *splitter = new StringSplitter(a, ',', 200);  // new StringSplitter(string_to_split, delimiter, limit)
  int RAW_DATA_LEN = splitter->getItemCount();
  //Serial.println("191");
  Serial.print("A variavel RAW_DATA_LEN 1 é:");
  Serial.println(RAW_DATA_LEN);
  uint16_t ir[RAW_DATA_LEN-1];
  //Serial.println("195");
  
  reg = splitter->getItemAtIndex(RAW_DATA_LEN-1); //Registra o primeiro elemento como data, hora e estado da ativação (YYYYMMDDHHMMSSEstado)
  //reg = splitter->getItemCount();
  //Serial.println("198");
  Serial.print("Registro da ultima ativação:");
  Serial.println(reg);
  //Serial.println("201");
  


  for (int i = 0; i < RAW_DATA_LEN -1; i++) { //Foi adaptado para utilizar o primeiro elemento como indicador de data, hora e estado da ultima ativação e por isso o codigo do ar condicionado é considerado somente a partir do segundo elemento (1)
    //Serial.println("206");
    ir[i] = splitter->getItemAtIndex(i).toInt();
    Serial.print(ir[i]);
    Serial.print(",");

  }

//  Serial.print("A variavel RAW_DATA_LEN 2 é:"); //Inserido aqui para testar se há distorção no sinal quando é enviado junto o estado
//  Serial.println(RAW_DATA_LEN);
  irsend.sendRaw(ir, RAW_DATA_LEN-1, 38); //Pass the buffer,length, optionally frequency
  Serial.println("");
  Serial.print(millis());
  Serial.println(": Sinal enviado com sucesso");


  ticker.attach(0.05, tick);
  delay(300);
  ticker.detach();

  delete splitter;
  splitter = NULL;

  //Serve pra exibir a quantidade de memoria livre apos a execução. Util pra avaliar vazamentos.

  int free2 = system_get_free_heap_size(); //utilizado somente para avaliar consumo de memoria do loop()
  if (free2 < 1000) { //o reset travou algumas vezes com os pinos D2 e D3 ligados e não travou nenhuma vez com eles desplugados, estudar isso
    Serial.println("Memoria inferior a 1Kb.\nResetando...");
    delay(0); //https://github.com/esp8266/Arduino/issues/1017#issuecomment-352107597
    ESP.reset();
    delay(0);
  }

  return free2;

}



////gets called when WiFiManager enters configuration mode
//void configModeCallback (WiFiManager *myWiFiManager) {
//  Serial.println("Entered config mode");
//  Serial.println(WiFi.softAPIP());
//  //if you used auto generated SSID, print it
//  Serial.println(myWiFiManager->getConfigPortalSSID());
//  //entered config mode, make led toggle faster
//  ticker.attach(0.5, tick);
//}

int esp_reset (String a) { //para resetar o ESP via Wifi
  Serial.println("Reiniciando o módulo");
  ticker.attach(0.2, tick);
  delay(2000);
  ESP.reset();
  delay(0);
  return 0;
}

int wifi_reset (String a) { //para limpar as configurações salvas de credenciais e forcar o modo AP
  Serial.println("Resetando as configurações de credenciais de Wifi");
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  esp_reset("string");
  return 0;
}

void setup(void)
{
  // Start Serial
  Serial.begin(115200);

//==========================================================================//

//Seção para recepção de comandos de IR
 Serial.println();
  Serial.print("IRrecvDumpV2 is now running and waiting for IR input on Pin ");
  Serial.println(RECV_PIN);
  Serial.println(WiFi.macAddress());

#if DECODE_HASH
  // Ignore messages with less than minimum on or off pulses.
  irrecv.setUnknownThreshold(MIN_UNKNOWN_SIZE);
#endif  // DECODE_HASH
  irrecv.enableIRIn();  // Start the receiver

  //=========================================================================//

  // Set callback aRest
  client.setCallback(callback);

  //LED de sinalização para os eventos
  pinMode(LED_Status, OUTPUT);
  pinMode(porta_reset, INPUT);

  //Ativa o LED para indicar que está executando o Setup. Se o LED continuar aceso depois de alguns segundos é uma indicação clara de que entrou no modo AP
  digitalWrite(LED_Status,HIGH);
  //Ativando o emissor de IR
  irsend.begin();

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id(device_id);
  rest.set_name("node03");

  //Declares the function in the Arduino sketch for Arest
  //rest.function("rheem_power_on", rheem_power_on);
  rest.function("ir_send", ir_send);
  rest.function("wifi_reset", wifi_reset);
  rest.function("esp_reset", esp_reset);


  // Init variables and expose them to REST API
  //rest.variable("local_ip", &local_ip); //local_ip foi retirada devido a sua função que conflita com as bibliotecas adicionadas para captar IR
  rest.variable("ir_recv", &ir_recv);
  rest.variable("reg",&reg);
  


  WiFiManager wifiManager;  //funcao que irá iniciar a configuração do wifi baseado nas ultimas credenciais fornecidas

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("node03", "nodemcu02")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ticker.attach(0.5, tick);
    delay(2000);
    ticker.detach();
    delay(0);//https://github.com/esp8266/Arduino/issues/1017#issuecomment-352107597
    ESP.reset();
    delay(0);
  }
  //Desativa o LED para indicar que saiu do modo AP e tambem para permitir que o sequenciamento baseado no Ticker funcione.
  digitalWrite(LED_Status,LOW);
  //if you get here you have connected to the WiFi
  Serial.println("Conectado a uma rede WiFi com sucesso. Prosseguindo.");
  ticker.attach(0.5, tick);
  delay(3000);
  ticker.detach();

  // Iniciar servidor local
  server.begin();
  Serial.println("Local server started on IP:");

  // Print the IP address
  Serial.println(WiFi.localIP());
  //local_ip = ipToString(WiFi.localIP()); //variavel associada à função iptostring que apresentava conflito com as bibliotecas de recepção de IR

  //Exibe informações importantes para criação de interface e rede wireless
  Serial.print("Device id: ");
  Serial.println(device_id);

  // Set output topic
  char* out_topic = rest.get_topic();

  //Seção referente ao OTA
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  //  ArduinoOTA.setPassword((const char *)"nodemcu02"); //Esse é o password para fazer uplado via OTA, deve ser o mesmo do AP para evitar confusões

  ArduinoOTA.begin();

  if (!MDNS.begin("node03")) {             // Start the mDNS responder for esp8266.local
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.println("mDNS responder started at node03.local");

}


void loop() {
  
  //=================================================//
//Seção para recepção de IR
  // Check if the IR code has been received.
  if (irrecv.decode(&results)) {
    ticker.attach(0.15,tick);
    delay(600);
    ticker.detach();
    // Display a crude timestamp.
    uint32_t now = millis();
    Serial.printf("Timestamp : %06u.%03u\n", now / 1000, now % 1000);
    if (results.overflow)
      Serial.printf("WARNING: IR code is too big for buffer (>= %d). "
                    "This result shouldn't be trusted until this is resolved. "
                    "Edit & increase CAPTURE_BUFFER_SIZE.\n",
                    CAPTURE_BUFFER_SIZE);
    // Display the basic output of what we found.
    Serial.print(resultToHumanReadableBasic(&results));
    dumpACInfo(&results);  // Display any extra A/C info if we have it.
    yield();  // Feed the WDT as the text output can take a while to print.

    // Display the library version the message was captured with.
    Serial.print("Library   : v");
    Serial.println(_IRREMOTEESP8266_VERSION_);
    Serial.println();

    // Output RAW timing info of the result.
    Serial.println(resultToTimingInfo(&results));
    yield();  // Feed the WDT (again)

    // Output the results as source code
    Serial.println(resultToSourceCode(&results));
    Serial.println("");  // Blank line between entries
    yield();  // Feed the WDT (again)

    ir_recv = resultToSourceCode(&results); //atribui o resultado à variável que vai ser exposta na interface web
  }
//=================================================//  
  //Serial.println("linha 267");
  //Função referente ao OTA. Deve ser declarada antes do gerenciamento de chamadas ou não funcionará
  ArduinoOTA.handle();
  //Serial.println("linha 270");
  // Connect to the cloud
  //rest.handle(client);
  //Serial.println("linha 273");
  
  // GERENCIAR CHAMADAS LOCAIS
  WiFiClient clientLocal = server.available();
  //Serial.println("linha 276");
  
  //Serial.println("linha 278");
  if (!clientLocal) {
    //Serial.println("linha 280");
    return;
  }
  //Serial.println("linha 283");
   while (!clientLocal.available()) { //Esse laço apresentou repetidos travamentos ao ser utilizado com Chrome e as vezes até com Firefox, por isso foi implementado uma logica que vai forçar a saída do laço quando ele permanecer por um longo periodo. O
    Serial.println("linha 285");
        delay(1);   
      timeout++;
      if(timeout>3000) {Serial.print("INFINITE LOOP BREAK!");  break;}
      }
  timeout=0;
 
  
  //Serial.println("linha 288");
  rest.handle(clientLocal);
  //Serial.println("linha 290");
  if ( digitalRead(porta_reset) == 1) {
  Serial.println("Porta D8 em nivel alto. Realizando a limpeza das credenciais de Wifi e reiniciando");
  ticker.attach(0.5, tick);
  delay(3000);
  ticker.detach();
  wifi_reset("Reset");//Essa string é só pra poder chamar a função atendendo o parametro de chamada obrigatorio
  }
  

}

// Handles message arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {
  rest.handle_callback(client, topic, payload, length);
}


// Convert IP address to String //Essa função apresentava conflito com algum método que é necessário para a a recepção simultanea de sinais de IR, por isso ela foi descartada e a variável local_ip que dependia dela foi deixada em branco
//String ipToString(IPAddress address)
//{
//  return String(address[0]) + "." +
//         String(address[1]) + "." +
//         String(address[2]) + "." +
//         String(address[3]);
//}