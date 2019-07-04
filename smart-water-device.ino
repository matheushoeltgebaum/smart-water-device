#include <FlowMeter.h>
#include <SoftwareSerial.h>

//Configura canal de comunicação serial entre o ESP e o módulo Sigfox.
SoftwareSerial mySerial(13, 15);

//Configura as propriedades de calibração do sensor de fluxo.
//FlowSensorProperties MySensor = {30.0f, 5.5f, {0.6, 0.6, 0.5, 0.5, 0.6, 0.6, 0.5, 0.5, 0.56, 0.56}};
FlowSensorProperties MySensor = {30.0f, 8.8f, {0.9, 0.9, 0.9, 1, 1, 1, 1, 1.05, 1.05, 1.05}};
FlowMeter Meter = FlowMeter(2, MySensor);

//Variáveis de controle de tempo (rotina do sensor de fluxo)
long periodFlowRoutine = 1000;   //1 segundo (em milisegundos)
long lastTimeFlowRoutine = 0;

//Variáveis de controle de tempo (rotina de comunicação Sigfox)
long periodSigfoxRoutine = 120000; //2 minutos (em milisegundos)
//long periodSigfoxRoutine = 3600000; //1 hora (em milisegundos)
long lastTimeSigfoxRoutine = 0;

//Saída do sensor de pressão.
float pressureSensorVoltage;

//Variável de controle para volume enviado pelo Sigfox.
int lastVolumeSent = 0;

//Variável que armazena o volume utilizado pela biblioteca (quantidade total gasta).
float totalVolume = 0.0;
//Variável que armazena o volume corrigido, com base no sensor de pressão.
float correctedTotalVolume = 0.0;

//Variável de volume total acumulado e enviado.
float accumulatedVolumeSent = 0.0;

//Código executado durante a interrupção realizada pelo sensor de fluxo.
void MeterISR() {
    //Incrementa a quantidade de pulsos gerados.
    Meter.count();
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

    //Calcula quanto foi consumido entre a última iteração e esta iteração.
    float currentVolume = Meter.getTotalVolume() - totalVolume;
    totalVolume += currentVolume;
    
    //Razão entre o fluxo e a pressão.
    float ratio = Meter.getCurrentFlowrate() / v;
    //Calcula o quanto deve alterar do volume atual lido.
    if (ratio > 0 && ratio < 2.5) {
      float diff = 2.5 - ratio;
      float correction = diff / 100;
      currentVolume += correction - 0.005;
    }
    correctedTotalVolume += currentVolume;

    Serial.println("WaterPressure - voltage: " + String(v) + " V.");
    Serial.println();

    Serial.println("Calculated Volume - total: " + String(totalVolume) + " l " +
                   "corrected: " + String(correctedTotalVolume) + " l.");
    Serial.println();
}

//Rotina responsável pela comunicação através do Sigfox.
void sigfoxRoutine() {
    int volumeToSend = (correctedTotalVolume - accumulatedVolumeSent) * 100;

    if (checkPreviousResponse() && lastVolumeSent > 0) {
      volumeToSend -= lastVolumeSent;
      accumulatedVolumeSent += lastVolumeSent;
    }
    
    testConnection();
    sendMessageRCZ4(volumeToSend);
    lastVolumeSent = volumeToSend;
}

//Verifica se a última mensagem foi enviada com sucesso.
bool checkPreviousResponse() {
  String downlinkResp = getDownlinkRespose();
    if (downlinkResp.length() == 32) {
      Serial.println("Enviou com sucesso a mensagem!");
      return true;
    } else {
      Serial.println("Não conseguiu enviar a mensagem!");
      return false;
    }
}

//Testa a conexão.
void testConnection() {
    serialWrite("AT\r", "AT: ", 100);
}

//Envia mensagem da zona RCZ4
void sendMessageRCZ4(int volume) {
    String response = serialWriteReturn("AT$GI?\n", "Config RCZ4: ", 100);
    
    //5 caracteres -> x,y\r\n
    if (response.length() == 5) {
      int x = response.charAt(0) - '0';
      int y = response.charAt(2) - '0';
  
      if (x == 0 || y < 3) {
        serialWrite("AT$RC\n", "Set RCZ4 Zone: ", 100);
      }
    }

    String payload = calculatePayload(volume);
    Serial.println(payload);
  
    //Deve alterar para um serialWriteReturn
    serialWrite("AT$SF=" + payload + ",1\r", "Frame Sent", 5000);
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

String getDownlinkRespose() {
  String response = "";

  if (mySerial.available() > 0) {
    while (mySerial.available() > 0) {
      Serial.write("\n");
      char c = mySerial.read();
      Serial.write(c);
      response.concat(c);
    }

    Serial.println("");
  }

  delay(500);
  return response;
}
