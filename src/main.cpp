#include <ESP8266WiFi.h>
#include <LiquidCrystal.h>
#include <ModbusIP_ESP8266.h>

#include <PID_v1.h>

#define ROTACAO_MCP       0
#define POS_ATUADOR       1

#define TOTAL_HREG        2

#define ESTADO_MCP        2
#define COMANDO_ONOFF_MCP 3
#define TOTAL_COIL        2

#define TOTAL_VAR         (TOTAL_HREG + TOTAL_COIL)

// Variáveis PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Modbus Registers Offsets
const int OFFSET_HREG = 0;
const int OFFSET_COIL = TOTAL_HREG;

unsigned int valoresRegistros[TOTAL_VAR] = {0, 0, 0, 0};  // Valores dos Registros Lidos
const char* visorRegistros[TOTAL_VAR] = {"ROTACAO: ", "ATUADOR: ", "ESTADO:", "COMANDO:"};

// variables will change:
int buttonState;
int buttonPressBSET = 0,buttonPressBDIS = 0, refreshVisor = 0;         // variable for reading the pushbutton status
int visor = 0, visorAnt = 0;

// Network parameters
const char* ssid = "pontes";
const char* password = "12345678";

// Set web server at port 80
WiFiServer server(80);

// Storing the HTTP request
String header;

// Button PIN
const int BSET = D1;
const int BDIS = D0;

// LCD PINs
const int RS = D2, EN = D3, d4 = D5, d5 = D6, d6 = D7, d7 = D8; 

// ModbusIP object
ModbusIP mb;//, mb2;

// LCD object
LiquidCrystal lcd(RS, EN, d4, d5, d6, d7);

// Funções PID
void iniciaMCP(){
  //initialize the variables we're linked to
  Input = valoresRegistros[ROTACAO_MCP];
  Setpoint = 50;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  valoresRegistros[COMANDO_ONOFF_MCP] = HIGH;
  valoresRegistros[ESTADO_MCP] = HIGH;
}

void paraMCP(){

  myPID.SetMode(MANUAL);

  valoresRegistros[COMANDO_ONOFF_MCP] = LOW;
  valoresRegistros[ESTADO_MCP] = LOW;
}

void processaPID(){
  Input = valoresRegistros[ROTACAO_MCP];

  myPID.Compute();

  valoresRegistros[POS_ATUADOR] = Output;
}
//------------------------------------

// Função de escrita nos Registros
uint16_t writePOS_ATUADOR(TRegister* reg, uint16_t val) {
    valoresRegistros[ POS_ATUADOR ] = val;
    //refreshVisor = HIGH;
  return val;
}
uint16_t writeROTACAO_MCP(TRegister* reg, uint16_t val) {
    valoresRegistros[ ROTACAO_MCP ] = val;
    //refreshVisor = HIGH;
  return val;
}

// Função para partir o MCP
uint16_t writeCoilComando(TRegister* reg, uint16_t val){
  if( val != LOW )
    iniciaMCP();
  else  
    paraMCP();

  return val;
}

// Função de atualizar dos Registros
void updateReg(){
  //for (int i = 0; i <= TOTAL_HREG; i++) {
    mb.Hreg(OFFSET_HREG + ROTACAO_MCP, valoresRegistros[ROTACAO_MCP] );
    mb.Hreg(OFFSET_HREG + POS_ATUADOR, valoresRegistros[POS_ATUADOR] );

    mb.Coil(OFFSET_COIL, (bool) valoresRegistros[ESTADO_MCP] );
    mb.Coil(OFFSET_COIL + 1, (bool) valoresRegistros[COMANDO_ONOFF_MCP] );
  //}
}

void serverTask() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if a new client connects,
    Serial.println("New Client.");
    String currentLine = "";                // 'currentLine' stores the current line of the request

    while (client.connected()) {
      if (client.available()) {             // checks if there are unread characters from the request

        char c = client.read();             // c stores the current character we are reading
        Serial.write(c);
        header += c;                        // we'll store the entire request in 'header'

        if (c == '\n') {
          if (currentLine.length() == 0) { 
            /* Note that we'll only enter this conditional with a double line break
            ('\n' on empty line) which signifies the end of an http request */

            /* HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            and a content-type, followed by a blank line */
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // if the request does not contain 'GET /xxx/' req will be set to -1
            int req = header.indexOf("GET /cmd/");
            if (req >= 0) {
              // header.substring(9, 12).toInt();
              char cmd = header[9] - 0x30;     // convert chars 9 to number
              if( cmd == 1 )
                iniciaMCP();
              else
                paraMCP();
            }
            // test -----------------------------------
            req = header.indexOf("GET /app"); 
            if (req >= 0) {
              for (int i = 0; i < TOTAL_VAR; i++) {
                client.print(valoresRegistros[i]);
                client.print(", ");
              }
              client.print("n");
            }
            else {
              // displaying HTML webpage
              client.println("<!DOCTYPE html><html>");
              client.println("<head><title>LCD_ESP TEST</title></head>");
              client.println("<body>");
                // test -----------------------------------
              client.print("<p>STATUS: ");
              for (int i = 0; i < TOTAL_VAR; i++) {
                client.print(valoresRegistros[i]);
                client.print(" ");
              }
              // test -----------------------------------
              client.println("</p>");
              client.println("</body>");
              client.println("</html>");
              client.println();           // http response ends with blank line
            }
            break;
          } 
          else 
            currentLine = "";   //currentLine == "";

        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }

    // Ends connection and clears the header
    header = "";
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

void updateVisor() {
    
  if( ( visor != visorAnt ) || ( refreshVisor == HIGH ) ){
    unsigned int valorR;

    refreshVisor = LOW;
    visorAnt = visor;
    
    lcd.setCursor(0, 1);
    lcd.print( visorRegistros[visor] );
    
    valorR = valoresRegistros[visor];
    if( visor == ESTADO_MCP ){
      if ( valorR != LOW ) lcd.print("on");
      else                 lcd.print("off");
    }
    else if( visor == COMANDO_ONOFF_MCP){
      if ( valorR != LOW ) lcd.print("stop");
      else                 lcd.print("start");
    }
    else{
      lcd.print( valorR );
    }
    lcd.print("   ");
  }
}

void processaTeclaBSET(){

  buttonState = digitalRead(BSET);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
      buttonPressBSET = HIGH;
  }
  else{
    if (buttonPressBSET == HIGH) {
      buttonPressBSET = LOW;
      refreshVisor = HIGH;
      if(visor < COMANDO_ONOFF_MCP)
        visor++;      
      else  
        visor=0;
    }
  }
}

void processaTeclaBDIS(){

  buttonState = digitalRead(BDIS);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
      buttonPressBDIS = HIGH;
  }
  else {
    if (buttonPressBDIS == HIGH) {
      int var = valoresRegistros[visor];
      buttonPressBDIS = LOW;
      refreshVisor = HIGH;
      if( visor == COMANDO_ONOFF_MCP ){
        var = ( var == LOW ) ? HIGH : LOW;
      }
      else if( visor == ROTACAO_MCP ){
        if( var <= 90 )   var += 10;
        else              var = 0;
      }
      valoresRegistros[visor] = var;
    }
  }
}

void setup() {

  int tryCount = 0;

  // initialize the pushbutton pin as an input:
  pinMode(BSET, INPUT);
  pinMode(BDIS, INPUT);
  
  //TEST*************************
  Serial.begin(9600);
  Serial.println("Teste");
  //TEST*************************

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    if(tryCount == 0){
      lcd.clear();
      lcd.print("Conectando"); 
      lcd.setCursor(0, 1);    // (note: line 1 is the second row, since counting begins with 0):
    }
    lcd.print(".");
    tryCount++;
    if( tryCount >= 16 )
      tryCount = 0;
    
    delay(500);
  }

  lcd.clear();
  lcd.print("IP:");
  lcd.print(WiFi.localIP());
  lcd.setCursor(0, 1);
  lcd.print("conectado");
  visor = ROTACAO_MCP;

  server.begin();

  mb.server();

  //mb2.client();

  // Add Registros HREG
  mb.addHreg(OFFSET_HREG + ROTACAO_MCP, valoresRegistros[ROTACAO_MCP]);
  mb.addHreg(OFFSET_HREG + POS_ATUADOR, valoresRegistros[POS_ATUADOR]);
  // Add Registros COIL
  mb.addCoil(OFFSET_COIL, (bool) valoresRegistros[ESTADO_MCP]);
  mb.addCoil(OFFSET_COIL + 1, (bool) valoresRegistros[COMANDO_ONOFF_MCP]);

  // Add Função de escrita aos Registros HREG
  mb.onSetHreg(OFFSET_HREG + ROTACAO_MCP, writeROTACAO_MCP);
  mb.onSetHreg(OFFSET_HREG + POS_ATUADOR, writePOS_ATUADOR);
  // Add Função de escrita aos Registros COIL
  //mb.onSetCoil(OFFSET_COIL, writeCoil);
  mb.onSetCoil(OFFSET_COIL + 1, writeCoilComando);
}

void loop() {

  refreshVisor = HIGH;

  //Atualiza LCD
  updateVisor();

  updateReg();

  //Server port 80 process
  serverTask();

  //Call once inside loop() - all magic here
  mb.task();

  //mb2.task();
  
  //Função PID 
  processaPID();

  //Processa Teclas
  processaTeclaBSET();
  processaTeclaBDIS();

  //delay(10);
}