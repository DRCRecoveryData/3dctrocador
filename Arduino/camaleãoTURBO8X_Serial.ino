/* * 3DChameleon Turbo 8x - Arduino Uno + CNC Shield
 * Adaptado para 8 Cores (Dual Extruder) - Jan/2026
 * -----------------------------------------------------------
 * COMANDOS SERIAL (Enviados via Klipper/Terminal):
 * * [ SELEÇÃO ]
 * T0 - T7         : Seleciona ferramenta e move seletor físico.
 * HOME            : Calibra seletor no batente físico (Canal 0).
 * IDLE            : Estaciona seletor entre ressaltos (alivia fio).
 * * [ MOVIMENTAÇÃO ]
 * LOAD <mm>       : Carrega até o sensor do bico.
 * LOAD <mm> EX <n>: Carrega até o sensor + <n> mm extras.
 * UNLOAD <mm>     : Recua o filamento na distância definida.
 * STOP_LOAD       : Interrupção imediata do motor de carga.
 * * [ CALIBRAÇÃO E TESTE ]
 * PRESSURE <val>  : Offset do seletor (+/- steps) para ajuste de mola.
 * SPEED <val>     : Velocidade extrusão (Menor = Mais rápido).
 * STRESS          : Ciclo automático de todas as combinações (T0-T7).
 * STATUS          : Diagnóstico de sensores, ferramenta e offsets.
 * * [ AUTOMAÇÃO DO BUFFER ]
 * BUFFER_ON/OFF   : Ativa/Desativa recarga automática (Piloto Automático).
 * PROTECAO_ON/OFF : Ativa/Desativa pausa por filamento preso/acabou.
 * * [ ACESSÓRIOS ]
 * COLETOR_ON/OFF  : Abre/Fecha servo do coletor de purga.
 * * -----------------------------------------------------------
 * MAPA DE ESTACIONAMENTO (IDLE):
 * T0/T4 -> Pos 2 | T1/T5 -> Pos 3 | T2/T6 -> Pos 0 | T3/T7 -> Pos 1
 */
#include <Servo.h>
#include <EEPROM.h>

// =================================================================================
// MAPEAMENTO DE PINOS - CNC SHIELD (MOTORES)
// =================================================================================
// --- Eixo Z: Motor da Extrusora 1 (T0, T1, T2, T3) ---
#define extEnable             8   // Z_ENABLE (Geralmente comum)
#define extStep               4   // Z_STEP
#define extDir                7   // Z_DIR

// --- Eixo Y: Motor da Extrusora 2 (T4, T5, T6, T7) ---
#define ext2Enable            8   // Y_ENABLE
#define ext2Step              3   // Y_STEP
#define ext2Dir               6   // Y_DIR

// --- Eixo X: Motor do Seletor (Tool Selector) ---
#define selEnable             8   // X_ENABLE
#define selStep               2   // X_STEP
#define selDir                5   // X_DIR

// =================================================================================
// PERIFÉRICOS E SENSORES
// =================================================================================
#define SERVO_PIN             11  // Servo do coletor/cortador
#define FILAMENT_SENSOR_PIN   A3  // Sensor no Hotend (Não utilizado)

// --- Sistema de Buffer (Microswitches NC) ---
#define BUFFER_EMPTY_PIN      A1  // Pino ABORT - Sensor de Buffer Vazio
#define BUFFER_FULL_PIN       A0  // Pino HOLD - Sensor de Buffer Cheio

// =================================================================================
// CONFIGURAÇÕES DE TEMPO E MEMÓRIA (EEPROM)
// =================================================================================
#define BUFFER_CHECK_INTERVAL 200     // Intervalo de checagem em ms
#define IDLE_TIMEOUT          300000  // Inatividade para Idle (5 min)

// --- Endereços de Memória Permanente ---
#define EEPROM_EXTRUDER_ADDR  0   // Salva a ferramenta atual (T0-T7)
#define EEPROM_PRESSURE_ADDR  4   // Salva o offset de pressão do seletor

// =================================================================================
// CONFIGURAÇÕES DE HARDWARE E CALIBRAÇÃO
// =================================================================================
const float STEPS_PER_MM      = 151.0;  // Passos por mm (Motores Z e Y)
const int stepsPerRev         = 200;    // Passos nativos do motor
const int microSteps          = 16;     // Microstepping configurado no driver
const int defaultBackoff      = 5;      // Recuo pós-homing para alinhar Canal 0 (5mm)

// --- Tempos e Delays (Velocidade) ---
int speedDelay                = 170;    // Velocidade das extrusoras (Z e Y)
const int selectorSpeedDelay  = 60;     // Velocidade do seletor (X)

// --- Definições de Direção ---
const int counterclockwise    = HIGH;
const int clockwise           = !counterclockwise;

// =================================================================================
// ESTADOS DO SISTEMA E FERRAMENTAS (T0-T7)
// =================================================================================
int currentExtruder           = -1;     // Ferramenta ativa (posição física)

int lastExtruder              = -1;     // Última usada (define motor Z ou Y)

int currentPhysPos            = 0;      // Posição física atual (0–3)
int selectorPressureOffset    = 0;      // Offset global em steps para pressão

// --- Flags de Controle e Lógica ---
bool usarProtecaoFila         = true;   // Erro se filamento der nó ou acabar
bool bufferAtivo              = true;   // Controle de carga automática do buffer
bool isInIdleMode             = false;  // Indica se o seletor está em descanso
bool sensorRemoteStop         = false;  // Controle de parada via Serial (STOP_LOAD)

// =================================================================================
// CONTROLE DO SERVO (COLETOR DE PURGA)
// =================================================================================
Servo filamentCutter;
int cutterPos                 = 0;
bool reverseServo             = true;

const int COLETOR_STOP        = 1500;   // Servo parado (Ponto morto)
const int COLETOR_OPEN_SPEED  = 1300;   // Rotação para abrir
const int COLETOR_CLOSE_SPEED = 1700;   // Rotação para fechar
const int COLETOR_OPEN_TIME   = 300;    // Tempo para atingir abertura
const int COLETOR_CLOSE_TIME  = 180;    // Tempo para atingir fechamento

// =================================================================================
// SENSORES, BUFFER E TIMERS
// =================================================================================
unsigned long lastBufferCheck    = 0;
unsigned long lastBufferActivity = 0;   // Timestamp da última movimentação
bool lastBufferEmptyState        = false; // Memória do sensor de buffer vazio
bool lastBufferFullState         = false; // Memória do sensor de buffer cheio

// =================================================================================
// COMUNICAÇÃO E SISTEMA
// =================================================================================
String serialBuffer           = "";     // Buffer de entrada do Klipper
bool commandReceived          = false;  // Flag de comando completo
int loaderMode                = 1;      // Modo automático (Carregar/Descarregar)


// =================================================================================
// CÁLCULOS DE DISTÂNCIA
// =================================================================================
long distance                 = 10;     // Valor base para rotações
long unloadDistance           = (long)stepsPerRev * microSteps * distance;
long loadDistance             = (long)unloadDistance * 1.1; // +10% de margem
// --- GESTÃO DE MEMÓRIA (EEPROM) ---
void savePressureOffset(int offset) { 
  EEPROM.put(EEPROM_PRESSURE_ADDR, offset);  // update() só grava se o valor for diferente
}
void loadPressureOffset() {
  int saved;
  EEPROM.get(EEPROM_PRESSURE_ADDR, saved);
  // Se valor for maior que 150 ou lixo (EEPROM limpa retorna -1 ou 255), reseta para 0
  selectorPressureOffset = (abs(saved) > 150) ? 0 : saved;
}
void saveCurrentExtruder(int extruder) { 
  EEPROM.update(EEPROM_EXTRUDER_ADDR, extruder + 1); 
}
int loadSavedExtruder() { 
  int saved = EEPROM.read(EEPROM_EXTRUDER_ADDR);
  // Retorna a ferramenta (0-7) ou -1 se o valor salvo for inválido/vazio
  return (saved > 0 && saved <= 8) ? (saved - 1) : -1;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("STARTUP: BUSY")); // Notifica o Klipper
  const int pins[] = {extEnable, extStep, extDir, ext2Enable, ext2Step, ext2Dir, selEnable, selStep, selDir};
  for (int i = 0; i < 9; i++) pinMode(pins[i], OUTPUT);

  digitalWrite(extEnable, HIGH);
  digitalWrite(ext2Enable, HIGH);
  digitalWrite(selEnable, LOW);
  // --- CONFIGURAÇÃO DE PINOS (SENSORES) ---
  pinMode(FILAMENT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUFFER_EMPTY_PIN, INPUT_PULLUP);
  pinMode(BUFFER_FULL_PIN, INPUT_PULLUP);
  filamentCutter.attach(SERVO_PIN);
  filamentCutter.writeMicroseconds(COLETOR_STOP);
  loadPressureOffset(); // Carrega o ajuste de mola da EEPROM
  homeSelector();   // O homeSelector agora calibra, lê a EEPROM e já move para a ferramenta correta.
  // --- INICIALIZAÇÃO DE ESTADOS DO BUFFER ---
  lastBufferEmptyState = (digitalRead(BUFFER_EMPTY_PIN) == HIGH);
  lastBufferFullState = (digitalRead(BUFFER_FULL_PIN) == HIGH);
  lastBufferActivity = millis();
  isInIdleMode = false;
  // --- INTERFACE DE USUÁRIO ---
  Serial.println(F("\n--- 3DCHAMELEON TURBO 8X READY ---"));
  Serial.println(F("[SEL] T0-T7, HOME, IDLE, STRESS"));
  Serial.println(F("[EXT] LOAD, UNLOAD, STOP_LOAD"));
  Serial.println(F("[CFG] PRESSURE, SPEED, STATUS, HELP"));
  Serial.println(F("[SYS] BUFFER_ON/OFF, PROT_ON/OFF, COL_ON/OFF"));
  Serial.println(F("------------------------------------------"));
  Serial.println(F("READY")); 
}

void loop() {
  while (Serial.available() > 0) { // --- LEITURA SERIAL ---
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) commandReceived = true;
      break;
    }
    serialBuffer += c;
  }
  // --- EXECUÇÃO DE COMANDOS ---
  if (commandReceived) {
    Serial.print(F("CMD: ")); Serial.println(serialBuffer);
    Serial.println(F("BUSY")); // Avisa o Klipper que está processando
    lastBufferActivity = millis();
    if (isInIdleMode) {
      isInIdleMode = false;
      Serial.println(F("WAKE_UP: OK"));
    }
    processSerialCommand(serialBuffer);
    serialBuffer = "";
    commandReceived = false;
    Serial.println(F("READY")); // Pronto para o próximo
  }
  // --- MONITORAMENTO AUTOMÁTICO ---
  monitorBufferStateChanges(); // Notifica mudanças nos sensores
  unsigned long currentTime = millis();
  if (currentTime - lastBufferCheck >= BUFFER_CHECK_INTERVAL) {
    maintainBuffer(); // Gerencia carga automática (Z ou Y)
    lastBufferCheck = currentTime;
  }
  checkIdleTimeout(currentTime); // Entra em IDLE se ocioso
  delay(5); // Estabilidade do clock
}

void coletorOn() { // --- FUNÇÕES DO COLETOR DE PURGA (Servo 360°) ---
  filamentCutter.writeMicroseconds(COLETOR_OPEN_SPEED); // Aciona motor
  delay(COLETOR_OPEN_TIME);                             // Aguarda curso
  filamentCutter.writeMicroseconds(COLETOR_STOP);       // Para
  Serial.println(F("COL: OPEN_OK"));
}
void coletorOff() {
  filamentCutter.writeMicroseconds(COLETOR_CLOSE_SPEED); // Aciona motor inverso
  delay(COLETOR_CLOSE_TIME);                             // Aguarda curso
  filamentCutter.writeMicroseconds(COLETOR_STOP);        // Para
  Serial.println(F("COL: CLOSE_OK"));
}

void monitorBufferStateChanges() { // --- MONITORAMENTO DOS SENSORES (A0/A1) --- BUFFER
  bool currentEmpty = (digitalRead(BUFFER_EMPTY_PIN) == HIGH);
  bool currentFull  = (digitalRead(BUFFER_FULL_PIN) == HIGH);
  if (currentEmpty != lastBufferEmptyState) {  // Detecta mudança no Vazio (A1)
    lastBufferEmptyState = currentEmpty;
    lastBufferActivity = millis();
    isInIdleMode = false;
    Serial.print(F("BUF_EMPTY: ")); Serial.println(currentEmpty);
  }
  if (currentFull != lastBufferFullState) { // Detecta mudança no Cheio (A0)
    lastBufferFullState = currentFull;
    lastBufferActivity = millis();
    isInIdleMode = false;
    Serial.print(F("BUF_FULL: ")); Serial.println(currentFull);
  }
}

void maintainBuffer() { // --- PILOTO AUTOMÁTICO DE RECARGA ---
  if (!bufferAtivo) return;
  bool bufferEmpty = (digitalRead(BUFFER_EMPTY_PIN) == LOW);
  unsigned long now = millis();
  if (bufferEmpty) {
    
    if (!lastBufferEmptyState) {  // Só inicia se detectou a borda de queda (vazio agora, mas estava cheio antes)
      Serial.println(F("BUFFER: RECHARGING"));
      lastBufferActivity = now;
      // Sincroniza posição física se necessário
      if ((currentExtruder % 4) != (lastExtruder % 4)) {
        gotoExtruder(currentPhysPos, lastExtruder % 4);
        currentPhysPos = lastExtruder % 4;

      }
      feedBuffer(); 
    }
  }
  checkIdleTimeout(now);
}

// --- GESTÃO DE REPOUSO (IDLE) ---
void checkIdleTimeout(unsigned long currentTime) {
  if (isInIdleMode) return;
  if ((currentTime - lastBufferActivity) >= IDLE_TIMEOUT) { // Se ocioso por mais de IDLE_TIMEOUT (5 min)
    Serial.println(F("IDLE: STARTING"));
    int idlePos = (lastExtruder + 2) % 4;  // Lógica 8 cores: Estaciona no vão entre ressaltos (last + 2)
    bool filamentInHotend = (digitalRead(FILAMENT_SENSOR_PIN) == HIGH);   // Pré-carga de segurança antes de relaxar o seletor
    bool bufferFull       = (digitalRead(BUFFER_FULL_PIN) == HIGH);
    if (filamentInHotend && !bufferFull) {
      if (currentExtruder != lastExtruder) {
        gotoExtruder(currentExtruder, lastExtruder);
        currentExtruder = lastExtruder;
      }
      feedBuffer();
    }
    gotoExtruder(currentPhysPos, idlePos);  // Move seletor para posição física de descanso (0-3)
    currentPhysPos = idlePos; 
    isInIdleMode = true;
    Serial.println(F("IDLE: SLEEPING"));
  }
}
// --- ALIMENTAÇÃO DO BUFFER (MOTORES Z/Y) ---
void feedBuffer() {
  int stepPin     = (lastExtruder < 4) ? extStep   : ext2Step;
  int dirPin      = (lastExtruder < 4) ? extDir    : ext2Dir;
  int enPin       = (lastExtruder < 4) ? extEnable : ext2Enable;
  bool direction  = (lastExtruder % 4 < 2) ? clockwise : counterclockwise;
  digitalWrite(enPin, LOW); 
  digitalWrite(dirPin, direction);
  long stepsFed = 0;
  long maxSteps = (long)(400.0 * STEPS_PER_MM); // Limite 400mm
  Serial.println(F("FEED: BUSY"));
  while (digitalRead(BUFFER_FULL_PIN) == HIGH) {
    if (usarProtecaoFila && (stepsFed > maxSteps)) {
      Serial.println(F("PAUSE")); 
      Serial.println(F("ERROR: FILAMENT_STUCK"));
      break; 
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedDelay);
    stepsFed++;
  }
  digitalWrite(enPin, HIGH); // Desativa motor para economizar energia
  Serial.print(F("FEED: OK (")); 
  Serial.print((float)stepsFed / STEPS_PER_MM); 
  Serial.println(F("mm)"));
}
void printHelp() {
  Serial.println(F("\n========= MANUAL DE COMANDOS 3DCHAMELEON ========="));
  Serial.println(F("T0-T7          : Seleciona ferramenta e move seletor"));
  Serial.println(F("HOME           : Calibra seletor no batente fisico"));
  Serial.println(F("IDLE           : Estaciona entre canais p/ aliviar fio"));
  Serial.println(F("STRESS         : Executa ciclo de testes automatico"));
  Serial.println(F("STATUS         : Diagnostico de sensores e variaveis"));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("LOAD 100 EX 5  : Carrega 100mm (ou ate sensor) + 5mm"));
  Serial.println(F("UNLOAD 120     : Recua 120mm de filamento"));
  Serial.println(F("STOP_LOAD      : Para o motor de carga imediatamente"));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("PRESSURE -20   : Ajusta o aperto do seletor (steps)"));
  Serial.println(F("SPEED 150      : Define velocidade (menor = +rapido)"));
  Serial.println(F("BUFFER_ON/OFF  : Ativa/Desativa recarga automatica"));
  Serial.println(F("PROTECAO_ON/OFF: Ativa/Desativa pausa se o fio prender"));
  Serial.println(F("COLETOR_ON/OFF : Abre/Fecha servo do cortador"));
  Serial.println(F("==================================================\n"));
}
//---------------------------------------------------------------------------------------------------
void processSerialCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command == F("STOP_LOAD")) { sensorRemoteStop = true; Serial.println(F("STOP: OK")); return; }
  if (command == F("HELP") || command == F("?")) { printHelp(); return; }
  if (command == F("STATUS")) {
    Serial.print(F("T:"));          Serial.println(currentExtruder);
    Serial.print(F("BF_FULL:"));    Serial.println(digitalRead(BUFFER_FULL_PIN));
    Serial.print(F("BF_EMPTY:"));   Serial.println(digitalRead(BUFFER_EMPTY_PIN));
    Serial.print(F("HOTEND:"));     Serial.println(digitalRead(FILAMENT_SENSOR_PIN));
    Serial.print(F("SPD:"));        Serial.println(speedDelay);
    Serial.print(F("PRESS:"));      Serial.println(selectorPressureOffset);
    Serial.print(F("PROT:"));       Serial.println(usarProtecaoFila ? 1 : 0);
    Serial.print(F("BUF_AUTO:"));   Serial.println(bufferAtivo ? 1 : 0);
    return;
  }
  if (command.startsWith(F("PRESSURE "))) {
    int val = command.substring(9).toInt();
    if (val >= -150 && val <= 150) {
      selectorPressureOffset = val;
      savePressureOffset(val); 
      Serial.print(F("PRESSURE: OK ")); Serial.println(val);
    } else Serial.println(F("ERROR: RANGE"));
    return;
  }
  if (command.startsWith(F("SPEED "))) {
    int v = command.substring(6).toInt();
    if (v >= 50) { speedDelay = v; Serial.print(F("SPEED: OK ")); Serial.println(v); }
    else Serial.println(F("ERROR: VALUE"));
    return;
  }
  if (command == F("PROTECAO_ON"))  { usarProtecaoFila = true;  Serial.println(F("PROT: 1")); return; }
  if (command == F("PROTECAO_OFF")) { usarProtecaoFila = false; Serial.println(F("PROT: 0")); return; }
  if (command == F("BUFFER_ON"))   { bufferAtivo = true;  Serial.println(F("BUF: 1"));  return; }
  if (command == F("BUFFER_OFF"))  { bufferAtivo = false; Serial.println(F("BUF: 0"));  return; }
  if (command.startsWith(F("T"))) {  // --- MOVIMENTAÇÃO (T0-T7) ---
    int tool = command.substring(1).toInt();
    if (tool >= 0 && tool <= 7) { 
      selectTool(tool); 
      //Serial.print(F("T: OK ")); Serial.println(tool); 
    } else Serial.println(F("ERROR: T_RANGE"));
    return;
  }
  if (command.startsWith(F("LOAD"))) {  // --- CARGA E DESCARGA ---
    float d = 0, e = 0;
    int ex = command.indexOf(F("EX"));
    if (ex > 4) d = command.substring(5, ex).toFloat();
    else if (ex == -1 && command.length() > 4) d = command.substring(5).toFloat();
    if (ex > 0) {
      String p = command.substring(ex);
      p.replace(F("EXT"), ""); p.replace(F("EX"), "");
      e = p.toFloat();
    }
    if (currentExtruder >= 0) executeLoadUnload(currentExtruder, true, d, e);
    else Serial.println(F("ERROR: NO_T"));
    return;
  }
  if (command.startsWith(F("UNLOAD "))) {
    float d = command.substring(7).toFloat();
    if (d > 0 && currentExtruder >= 0) executeLoadUnload(currentExtruder, false, d, 0);
    else Serial.println(F("ERROR: UNLOAD_PARAMS"));
    return;
  }
  // --- SISTEMA ---
  if (command == F("HOME"))         { homeSelector(); Serial.println(F("HOME: OK")); return; }
  if (command == F("IDLE"))         { moveToIdle();   Serial.println(F("IDLE: OK")); return; }
  if (command == F("COLETOR_ON"))   { coletorOn();    return; }
  if (command == F("COLETOR_OFF"))  { coletorOff();   return; }
  if (command == F("STRESS"))       { stressTest();   return; }
  Serial.println(F("ERROR: UNKNOWN_CMD"));
}





//---------------------------------------------------------------------------------------------------


void stressTest() {
  Serial.println(F("STRESS_TEST: START"));
  
  // Sequência estratégica para validar saltos e trocas de motor
  int seq[] = {
    0, 1, 0, 2, 0, 3,         // Motor Z (0-3)
    4, 5, 4, 6, 4, 7,         // Motor Y (4-7)
    0, 4, 1, 5, 2, 6, 3, 7,   // Troca de motor (paridade)
    3, 0, 7, 4,               // Retornos longos
    0, 7, 3, 4                // Cruzamentos extremos
  };
  int total = sizeof(seq) / sizeof(seq[0]);
  for (int i = 0; i < total; i++) {
    int t = seq[i];
    Serial.print(F("STEP: ")); Serial.print(i + 1);
    Serial.print(F("/"));      Serial.print(total);
    Serial.print(F(" | TARGET: T")); Serial.println(t);
    selectTool(t);
    bool dir = (t % 4 < 2) ? clockwise : counterclockwise;
    rotateExtruder(t, dir, (long)(1.5 * STEPS_PER_MM));
    delay(2000); 
  }
  Serial.println(F("STRESS_TEST: DONE"));
  selectTool(0);
}




void selectTool(int toolNumber) { // --- SELEÇÃO DE FERRAMENTA (T0-T7) ---
  int fromPos = (currentExtruder >= 0) ? (currentExtruder % 4) : 0;
  int targetPos = toolNumber % 4;
  gotoExtruder(fromPos, targetPos);
  currentExtruder = toolNumber;
  lastExtruder = toolNumber;
  currentPhysPos = targetPos;
  saveCurrentExtruder(currentExtruder);
  Serial.print(F("T: OK ")); Serial.println(toolNumber);
}
void moveToIdle() { // --- MODO DE DESCANSO (IDLE) ---
  if (isInIdleMode) return;
  int idlePos = (lastExtruder + 2) % 4; 
  int physNow = currentExtruder % 4;
  if (physNow != idlePos) {
    Serial.print(F("IDLE: MOVING ")); Serial.print(physNow);
    Serial.print(F("->")); Serial.println(idlePos);
    gotoExtruder(physNow, idlePos);
    currentPhysPos = idlePos; 
    isInIdleMode = true;
    Serial.println(F("IDLE: OK"));
  }
}
long loadUntilSensor(int tool, bool dir, float dist_mm, float extra_mm) { // --- CARGA COM INTERRUPÇÃO REMOTA (STOP_LOAD) ---
  sensorRemoteStop = false;
  int stepPin = (tool < 4) ? extStep   : ext2Step;
  int dirPin  = (tool < 4) ? extDir    : ext2Dir;
  int enPin   = (tool < 4) ? extEnable : ext2Enable;
  digitalWrite(enPin, LOW); 
  digitalWrite(dirPin, dir);
  long steps = 0;
  long maxSteps = (dist_mm > 0) ? (long)(dist_mm * STEPS_PER_MM) : (long)(3000.0 * STEPS_PER_MM); 
  const int minWait = speedDelay;       
  const int maxWait = speedDelay / 5;   
  long ramp = 50.0 * STEPS_PER_MM; 
  long decelStart = (extra_mm > 0) ? -1 : (maxSteps - ramp);
  Serial.println(F("LOAD: BUSY"));
  while (steps < maxSteps && !sensorRemoteStop) {
    if (digitalRead(FILAMENT_SENSOR_PIN) == LOW) { 
      Serial.println(F("SENSOR: DETECTED"));
      break; 
    }
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'S') {
        String cmd = Serial.readStringUntil('\n');
        if (cmd == F("TOP_LOAD")) sensorRemoteStop = true;
      }
    }
    if (sensorRemoteStop) break;
    int wait = maxWait;
    if (steps < ramp) { // Aceleração
      wait = minWait - (int)((minWait - maxWait) * ((float)steps / ramp));
    } else if (decelStart > 0 && steps > decelStart) { // Desaceleração
      wait = maxWait + (int)((minWait - maxWait) * ((float)(steps - decelStart) / ramp));
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(wait);
    steps++;
  }
  if (extra_mm > 0 && !sensorRemoteStop) {
    long extraSteps = (long)(extra_mm * STEPS_PER_MM);
    long eDecel = max(0L, extraSteps - ramp); 
    Serial.println(F("LOAD: EXTRA_PUSH"));
    for (long i = 0; i < extraSteps; i++) {
      int eWait = (i > eDecel) ? 
                  maxWait + (int)((minWait - maxWait) * ((float)(i - eDecel) / ramp)) : 
                  maxWait;
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(eWait);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(eWait);
      steps++;
    }
  }
  digitalWrite(enPin, HIGH);
  Serial.print(F("LOAD: OK (")); 
  Serial.print((float)steps / STEPS_PER_MM); 
  Serial.print(F("mm)"));
  if (sensorRemoteStop) Serial.print(F(" [STOPPED]"));
  Serial.println();
  return steps;
}

void executeLoadUnload(int tool, bool isLoad, float dist, float extra) { // --- CARGA E DESCARGA (8 CORES) ---
  Serial.print(isLoad ? F("CMD_LOAD: T") : F("CMD_UNLOAD: T"));
  Serial.print(tool); Serial.println(F(" BUSY"));
  if (currentExtruder != tool) {
    gotoExtruder(currentExtruder % 4, tool % 4);
    currentExtruder = tool;
  }
  bool dir = (tool % 4 < 2) ? clockwise : counterclockwise;  // LOAD: T0,T1 (CW) | T2,T3 (CCW) --- UNLOAD: Inverte tudo
  if (!isLoad) dir = !dir;
  if (isLoad) {
    loadUntilSensor(tool, dir, dist, extra);
  } else {
    rotateExtruder(tool, dir, (long)(dist * STEPS_PER_MM));
    Serial.println(F("UNLOAD: OK"));
  }
  lastExtruder = tool;
  saveCurrentExtruder(currentExtruder);
}


// --- MOVIMENTAÇÃO DO SELETOR (EIXO X) ---
void gotoExtruder(int from, int to) {
  int physFrom = from % 4;
  int physTo = to % 4;
  if (selectorPressureOffset != 0) { // 1. ZERAR OFFSET (VOLTA AO CENTRO)
    rotateSelector(selectorPressureOffset > 0, abs(selectorPressureOffset));
    delay(20); 
  }
  if (physFrom == physTo) return;
  int diff = physTo - physFrom;  // 2. CÁLCULO DE MOVIMENTO
  bool dir = (diff > 0) ? counterclockwise : clockwise;
  long stepsPerPos = (stepsPerRev * microSteps) / 4; // 800 steps por posição (1/4 de volta)
  long totalSteps = (long)abs(diff) * stepsPerPos;
  totalSteps += selectorPressureOffset;  // 3. APLICAÇÃO DO OFFSET DE PRESSÃO
  Serial.print(F("SEL: ")); Serial.print(physFrom);
  Serial.print(F("->")); Serial.print(physTo);
  Serial.print(F(" [")); Serial.print(totalSteps); Serial.println(F(" steps]"));
  rotateSelector(dir, totalSteps);
  isInIdleMode = false; // Se moveu, não está mais em repouso
}


void rotateExtruder(int tool, bool dir, long steps) { // --- MOVIMENTAÇÃO DOS EXTRUSORES (Z ou Y) ---
  if (steps <= 0) return; 
  int stepPin = (tool < 4) ? extStep   : ext2Step;
  int dirPin  = (tool < 4) ? extDir    : ext2Dir;
  int enPin   = (tool < 4) ? extEnable : ext2Enable;
  digitalWrite(enPin, LOW); 
  digitalWrite(dirPin, dir);
  const int minWait = speedDelay;
  const int maxWait = speedDelay / 6;
  if ((float)steps / STEPS_PER_MM < 100.0) { // Decisão de rampa baseada na distância (Limite 100mm)
    for (long x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH); delayMicroseconds(minWait);
      digitalWrite(stepPin, LOW);  delayMicroseconds(minWait);
    }
  } else {
    long accelLimit = steps * 0.08; // Movimento longo: Rampa 8% / 82% / 10%
    long decelLimit = steps * 0.90;
    for (long x = 0; x < steps; x++) {
      int wait = maxWait;
      if (x < accelLimit) { // Acelera
        wait = minWait - (int)((minWait - maxWait) * ((float)x / accelLimit));
      } 
      else if (x > decelLimit) { // Desacelera
        wait = maxWait + (int)((minWait - maxWait) * ((float)(x - decelLimit) / (steps - decelLimit)));
      }
      digitalWrite(stepPin, HIGH); delayMicroseconds(wait);
      digitalWrite(stepPin, LOW);  delayMicroseconds(wait);
    }
  }
  digitalWrite(enPin, HIGH); // Sleep motor
}



void rotateSelector(bool dir, int steps) { // --- MOVIMENTAÇÃO DO SELETOR (EIXO X) ---
  digitalWrite(selEnable, LOW); // Mantém energizado/travado
  digitalWrite(selDir, dir);
  for (int x = 0; x < steps; x++) {
    digitalWrite(selStep, HIGH);
    delayMicroseconds(selectorSpeedDelay);
    digitalWrite(selStep, LOW);
    delayMicroseconds(selectorSpeedDelay);
  }
}

void homeSelector() // --- HOMING DO SELETOR ---
{
  // 1. Move até o batente físico (sentido horário)
  rotateSelector(clockwise, (int)(stepsPerRev * microSteps * 1.3));
  delay(100);
  // 2. Recua para o centro exato do canal 0
  rotateSelector(counterclockwise, defaultBackoff * microSteps);
  delay(50);
  // 3. Estado físico REAL após homing
  currentPhysPos = 0;        // posição física conhecida
  isInIdleMode   = false;    // acabou de mover, não está em idle
  int saved = loadSavedExtruder();  // 4. Restaura a última ferramenta válida salva
  if (saved < 0 || saved > 7) saved = 0;

  lastExtruder    = saved;
  currentExtruder = saved;
  Serial.print(F("HOME: OK | Phys=0 | T"));
  Serial.println(currentExtruder);
}
