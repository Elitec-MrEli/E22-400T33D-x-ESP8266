// Código para ESP8266 Receptor (Slave)

  /*NODEMCU ESP8266          E22-400T33D
    3.3V/VIN --------> VCC (5V recomendado) ⭐
    GND      --------> GND
    GPIO13   --------> RXD (do E22)
    GPIO15   --------> TXD (do E22) 
    GPIO12   --------> M0
    GPIO14   --------> M1
    GPIO2    --------> AUX (opcional)
  */  

// Estrutura para dados da mensagem (MESMA do transmissor)
struct Message {
  uint8_t senderID;
  uint8_t receiverID;
  uint32_t messageID;
  char text[32];
  int8_t rssi;
};

// Pinos para o ESP8266 (MESMOS pinos)
#define LORA_M0 12
#define LORA_M1 14
#define LORA_AUX 2

void setup() {
  Serial.begin(115200);
  
  // Configurar pins de controle do E22 (MESMA configuração)
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
  Serial.println("ESP8266 Receptor E22-400T33D");
  Serial.println("Aguardando inicialização do módulo...");
  
  waitForModuleReady();
  setupLoRaModule();
  Serial.println("Receptor pronto!");
}

void waitForModuleReady() {
  // ⭐ MESMA função do transmissor
  Serial.print("Aguardando módulo ficar pronto");
  for (int i = 0; i < 50; i++) {
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
  // ⭐ MODIFICAÇÃO IMPORTANTE: ID diferente do transmissor
  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);
  delay(100);
  
  uint8_t configCommand[] = {
    0xC0,                    // Comando de escrita
    0x00,                    // Endereço inicial
    0x06,                    // Número de parâmetros (6 bytes)
    0x00,                    // ADDH - Endereço alto
    0x02,                    // ⭐ ADDL - ID DIFERENTE: 0x02 (transmissor é 0x01)
    0x00,                    // NETID - MESMA rede
    0x68,                    // REG0 - MESMA configuração
    0x17,                    // REG1 - MESMA configuração  
    0x00                     // REG2 - MESMO canal
  };
  
  Serial1.write(configCommand, sizeof(configCommand));
  delay(2000);
  
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(100);
  
  Serial.println("Módulo LoRa configurado como RECEPTOR!");
}

void loop() {
  // ⭐ SLAVE só recebe mensagens (não envia periodicamente)
  checkForMessages();
  
  // ⭐ OPCIONAL: Pode adicionar envio por botão ou condição específica
  if (Serial.available()) {
    String input = Serial.readString();
    if (input == "enviar") {
      sendResponse();
    }
  }
  
  delay(100);
}

void checkForMessages() {
  if (Serial1.available() >= sizeof(Message)) {
    Message receivedMsg;
    for (int i = 0; i < sizeof(Message); i++) {
      ((uint8_t*)&receivedMsg)[i] = Serial1.read();
    }
    
    // ⭐ MODIFICAÇÃO: Verifica se a mensagem é para ESTE dispositivo (0x02)
    if (receivedMsg.receiverID == 0x02) {
      Serial.print("📨 Mensagem recebida: ");
      Serial.print(receivedMsg.text);
      Serial.print(" | De: 0x");
      Serial.print(receivedMsg.senderID, HEX);
      Serial.print(" | ID: ");
      Serial.print(receivedMsg.messageID);
      Serial.print(" | RSSI: ");
      Serial.print(receivedMsg.rssi);
      Serial.println(" dBm");
      
      // ⭐ OPCIONAL: Resposta automática
      sendResponse();
    }
  }
}

// ⭐ FUNÇÃO NOVA: Enviar resposta
void sendResponse() {
  static uint32_t responseCount = 0;
  
  Message responseMsg;
  responseMsg.senderID = 0x02;        // ⭐ ID do RECEPTOR
  responseMsg.receiverID = 0x01;      // ⭐ Responde para o TRANSMISSOR
  responseMsg.messageID = responseCount;
  responseMsg.rssi = 0;
  
  sprintf(responseMsg.text, "ACK #%lu do Receptor", responseCount);
  
  Serial1.write((uint8_t*)&responseMsg, sizeof(responseMsg));
  Serial1.flush();
  
  Serial.print("📤 Resposta enviada: ");
  Serial.println(responseMsg.text);
  
  responseCount++;
}
