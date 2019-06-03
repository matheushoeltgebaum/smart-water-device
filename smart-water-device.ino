#include <FlowMeter.h>
#include <SoftwareSerial.h>

//Configura canal de comunicação serial entre o ESP e o módulo Sigfox.
SoftwareSerial mySerial(13, 15);

//Configura as propriedades de calibração do sensor de fluxo.
//FlowSensorProperties MySensor = {30.0f, 7.5f, {0.74, 0.74, 0.76, 0.76, 0.74, 0.74, 0.71, 0.71, 0.7, 0.7}};
FlowSensorProperties MySensor = {30.0f, 5.5f, {0.6, 0.6, 0.5, 0.5, 0.6, 0.6, 0.5, 0.5, 0.56, 0.56}};
//FlowSensorProperties MySensor = {30.0f, 4.5f, {0.45, 0.45, 0.45, 0.45, 0.43, 0.43, 0.43, 0.43, 0.46, 0.46}};
FlowMeter Meter = FlowMeter(2, MySensor);

//Variáveis de controle de tempo (rotina do sensor de fluxo)
long periodFlowRoutine = 1000;   //1 segundo (em milisegundos)
long lastTimeFlowRoutine = 0;

//Variável de controle para volume enviado pelo Sigfox.
int lastVolumeSent = 0;

//Variáveis de controle de tempo (rotina de comunicação Sigfox)
long periodSigfoxRoutine = 60000; //1 minuto (em milisegundos)
long lastTimeSigfoxRoutine = 0;

float pressureSensorVoltage;
float pressureSensorValue;
float pressureSensorValueBars;

//Código executado durante a interrupção realizada pelo sensor de fluxo.
void MeterISR() {
    //Incrementa a quantidade de pulsos gerados.
    pressureSensorVoltage = analogRead(0) * 5.00 / 1024;
    /*pressureSensorValue = (pressureSensorVoltage / 5.0 * 0.75) - 0.1;*/
    pressureSensorValueBars = ((3.0*((float)pressureSensorVoltage-0.47))*1000000.0) / 10e5;

    if (pressureSensorValueBars >= 0.84) {
      Meter.count();
    }
}

void setup() {
    //Inicializa o Serial
    Serial.begin(9600);
    while (!Serial) {}
    Serial.println("Connected");

    //Habilita a interrupção gerada pelo sensor de fluxo, quando o sinal gerado for de BAIXO para ALTO.
    attachInterrupt(digitalPinToInterrupt(4), MeterISR, RISING);

    //Zera as variáveis de controle de fluxo.
    Meter.reset();
}

void loop() {
    long currentTime = millis();
    long durationFlowRoutine = currentTime - lastTimeFlowRoutine;
    long durationSigfoxRoutine = currentTime - lastTimeSigfoxRoutine;

    if (durationFlowRoutine >= periodFlowRoutine) {
      flowMeterRoutine(durationFlowRoutine);
      lastTimeFlowRoutine = currentTime;
    }

    if (durationSigfoxRoutine >= periodSigfoxRoutine) {
      sigfoxRoutine();
      lastTimeSigfoxRoutine = currentTime;
    }
}

//Rotina responsável pelo controle de fluxo.
void flowMeterRoutine(long duration) {
    //Realiza o processamento dos pulsos obtidos pelo sensor de fluxo.
    Meter.tick(duration);
    
    //Mostra os resultados obtidos na saída.
    Serial.println("FlowMeter - current flow rate: " + String(Meter.getCurrentFlowrate()) + " l/min, " +
                   "nominal volume: " + String(Meter.getTotalVolume()) + " l, " +
                   "compensated error: " + String(Meter.getCurrentError()) + " %, " +
                   "duration: " + String(Meter.getTotalDuration() / 1000) + " s.");

    float v = analogRead(0) * 5.00 / 1024;
    float p = (v / 5.0 * 0.75) - 0.1;
    float pressure_pascal = (3.0*((float)v-0.47))*1000000.0;
    float pressure_bar = pressure_pascal/10e5;

    Serial.println("WaterPressure - voltage: " + String(v) + " V, " +
                   "pressure: " + String(p) + ", " +
                   "pressure (Pascal): " + String(pressure_pascal) + ", " +
                   "pressure (Bars): " + String(pressure_bar) + ".");
    Serial.println();
}

//Rotina responsável pela comunicação através do Sigfox.
void sigfoxRoutine() {
    testConnection();
    sendMessageRCZ4();
}

//Testa a conexão.
void testConnection() {
    serialWrite("AT\r", "AT: ", 100);
}

//Envia mensagem da zona RCZ4
void sendMessageRCZ4() {
    String response = serialWriteReturn("AT$GI?\n", "Config RCZ4: ", 100);
    
    //5 caracteres -> x,y\r\n
    if (response.length() == 5) {
      int x = response.charAt(0) - '0';
      int y = response.charAt(2) - '0';
  
      if (x == 0 || y < 3) {
        serialWrite("AT$RC\n", "Set RCZ4 Zone: ", 100);
      }
    }

    int value = (Meter.getTotalVolume() * 100) - lastVolumeSent;
    lastVolumeSent += value;

    String payload = calculatePayload(value);
    Serial.println(payload);
    serialWrite("AT$SF=" + payload + "\r", "Response: ", 5000);
}

//Processa o valor a ser enviado pelo módulo Sigfox.
String calculatePayload(int value) {
  String payload = String(value);
  int payloadLength = payload.length();

  //Caso o tamanho da String seja ímpar, adiciona um zero na frente.
  if (payloadLength % 2 != 0) {
    payload = "0" + payload;
  }

  return payload;
}

//Executa um comando na serial do módulo Sigfox.
void serialWrite(String command, String output, int timeout) {
    char cmd[50];
    command.toCharArray(cmd, 50);
    char out[50];
    output.toCharArray(out, 50);
    //Escreve o comando no serial entre o ESP e o módulo, aguardando um tempo de resposta.
    mySerial.write(cmd);
    delay(timeout);
    Serial.print(out);
  
    if (mySerial.available() > 0) {
      while (mySerial.available() > 0) {
        Serial.write(mySerial.read());
      }
  
      Serial.println("");
    }
    
    delay(500);
}

//Executa um comando na serial do módulo Sigfox, retornando a resposta.
String serialWriteReturn(String command, String output, int timeout) {
    char cmd[50];
    command.toCharArray(cmd, 50);
    char out[50];
    output.toCharArray(out, 50);
    //Escreve o comando no serial entre o ESP e o módulo, aguardando um tempo de resposta.
    mySerial.write(cmd);
    delay(timeout);
    Serial.print(out);
  
    String response = "";
  
    if (mySerial.available() > 0) {
      while (mySerial.available() > 0) {
        char c = mySerial.read();
        Serial.write(c);
        response.concat(c);
      }
  
      Serial.println("");
    }
    
    delay(500);
  
    return response;
}
