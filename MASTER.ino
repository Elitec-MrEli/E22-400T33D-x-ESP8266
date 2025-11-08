// Código para ESP8266 Transmissor (Master)

  /*NODEMCU ESP8266          E22-400T33D
    3.3V/VIN --------> VCC (5V recomendado) ⭐
    GND      --------> GND
    GPIO13   --------> RXD (do E22)
    GPIO15   --------> TXD (do E22) 
    GPIO12   --------> M0
    GPIO14   --------> M1
    GPIO2    --------> AUX (opcional)
  */
   
// Estrutura para dados da mensagem
struct Message {
  uint8_t senderID;
  uint8_t receiverID;
  uint32_t messageID;
  char text[32];
  int8_t rssi;
};

// Pinos para o ESP8266
#define LORA_M0 12
#define LORA_M1 14
#define LORA_AUX 2

void setup() {
  Serial.begin(115200);
  
  // Configurar pins de controle do E22
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  
  // Configurar módulo para modo normal (M0=0, M1=0)
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  
  // Inicializar Serial1 para o módulo LoRa
  Serial1.begin(9600, SERIAL_8N1);
  Serial1.swap(); // Usa GPIO13(RX) e GPIO15(TX) para Serial1
  
  Serial.println();
  Serial.println("ESP8266 Transmissor E22-400T33D");
  Serial.println("Aguardando inicialização do módulo...");
  
  waitForModuleReady();
  setupLoRaModule();
  Serial.println("Transmissor pronto!");
}

void waitForModuleReady() {
  Serial.print("Aguardando módulo ficar pronto");
  for (int i = 0; i < 50; i++) { // Timeout de 5 segundos
    if (digitalRead(LORA_AUX) == HIGH) {
      Serial.println("\nMódulo pronto!");
      return;
    }
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nTimeout! Módulo pode não estar respondendo.");
}

void setupLoRaModule() {
  // Entrar em modo de configuração (M0=1, M1=1)
  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);
  delay(100);
  
  // PARÂMETROS OTIMIZADOS PARA MÁXIMA DISTÂNCIA
  uint8_t configCommand[] = {
    0xC0,                    // Comando de escrita
    0x00,                    // Endereço inicial
    0x06,                    // Número de parâmetros (6 bytes)
    0x00,                    // ADDH - Endereço alto
    0x01,                    // ADDL - Endereço baixo (ID deste dispositivo)
    0x00,                    // NETID - ID da rede
    0x68,                    // REG0 - 9600bps, 8N1, taxa aérea 2.4kbps (MAIS LENTA = MAIS DISTÂNCIA)
    0x17,                    // REG1 - Potência 33dBm (MÁXIMA), subpacote 32 bytes
    0x00                     // REG2 - Canal 0 (433.125MHz - melhor penetração)
  };
  
  Serial1.write(configCommand, sizeof(configCommand));
  delay(2000);
  
  // Voltar para modo normal
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(100);
  
  Serial.println("Módulo LoRa configurado para MÁXIMA DISTÂNCIA!");
}

void loop() {
  static uint32_t lastSendTime = 0;
  static uint32_t messageCount = 0;
  
  if (millis() - lastSendTime >= 10000) { // Envia a cada 10 segundos
    sendMessage(messageCount);
    messageCount++;
    lastSendTime = millis();
  }
  
  // Verifica se há mensagens recebidas
  checkForMessages();
  
  delay(100);
}

void sendMessage(uint32_t msgID) {
  Message msg;
  msg.senderID = 0x01;        // ID do transmissor
  msg.receiverID = 0x02;      // ID do receptor
  msg.messageID = msgID;
  msg.rssi = 0;               // RSSI será preenchido pelo receptor
  
  sprintf(msg.text, "Msg #%lu do ESP8266", msgID);
  
  // Envia a mensagem
  Serial1.write((uint8_t*)&msg, sizeof(msg));
  Serial1.flush();
  
  Serial.print("Mensagem enviada: ");
  Serial.print(msg.text);
  Serial.print(" | ID: ");
  Serial.println(msgID);
}

void checkForMessages() {
  if (Serial1.available() >= sizeof(Message)) {
    Message receivedMsg;
    for (int i = 0; i < sizeof(Message); i++) {
      ((uint8_t*)&receivedMsg)[i] = Serial1.read();
    }
    
    if (receivedMsg.receiverID == 0x01) { // Mensagem é para este dispositivo
      Serial.print("Mensagem recebida: ");
      Serial.print(receivedMsg.text);
      Serial.print(" | De: 0x");
      Serial.print(receivedMsg.senderID, HEX);
      Serial.print(" | RSSI: ");
      Serial.print(receivedMsg.rssi);
      Serial.println(" dBm");
    }
  }
}
