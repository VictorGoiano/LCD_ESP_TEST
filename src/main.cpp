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
const char* ssid = "Tim Live Victor";
const char* password = "pontes309vpm";

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
  Input = valoresRegistros[ROTACAO_MCP];    //Input = analogRead(PIN_INPUT);
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
  Input = valoresRegistros[ROTACAO_MCP];      //Input = analogRead(PIN_INPUT);

  myPID.Compute();

  valoresRegistros[POS_ATUADOR] = Output;     //analogWrite(PIN_OUTPUT, Output);
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

  updateReg();    //mb.Hreg(LEME_HREG, valorLeme);

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