// ESPNOW_Rotore_HTTP.ino - ricevitore ESP32 per il rotore az/el satellitare (antenna Arrow-style)
// COPIA DI LAVORO di ESPNOW_Rotore.ino: aggiunge SOLO un nuovo endpoint HTTP (/api/passload)
// per caricare un passaggio completo dal telefono (al posto del Cardputer via ESP-NOW).
// Tutto il resto (ESP-NOW, pagina web manuale, passTick/servoSlewTick, limiti di sicurezza)
// e' IDENTICO all'originale, non toccato: la nuova funzione hPassLoad() si limita a riempire
// le stesse variabili globali che oggi riempie handlePacket() per RCMD_PASS_LOAD, cosi' tutto
// il resto dell'esecuzione del passaggio (passTick) funziona esattamente come gia' testato.
// --------------------------------------------------------------------
// Stesso schema della Loop (OLR_LoopRX): ESP-NOW dal Cardputer + Access Point/pagina web di scorta
// per puntamento manuale sul campo. Protocollo indipendente da quello della Loop (RotorProto.h):
// MAC e pairing separati, cosi' le due ESP32 si agganciano/sganciano senza interferire.
// --------------------------------------------------------------------
// Scheda: ESP32-WROOM-32 (stessa famiglia della Loop)
// Servi: 2x DS3230 Pro 270 gradi, direct drive (nessun riduttore/cinghia)
//
// Cablaggio:
//   Servo AZ segnale -> GPIO 32
//   Servo EL segnale -> GPIO 33
//   Servi: alimentazione 5-6.8V DEDICATA (power bank / BEC), NON dai 5V della ESP32
//   GND servi in comune con GND ESP32
//
// Libreria richiesta (Library Manager): "ESP32Servo" di Kevin Harrington / madhephaestus
//
// Filosofia: il rotore e' "muto". Riceve due angoli SERVO gia' pronti (0..270.0 gradi,
// RCMD_GOTO) dal Cardputer, che fa tutta l'astronomia e la calibrazione del centro azimut.
// RCMD_PARK porta entrambi i servi al centro (135 gradi = riferimento fisico di calibrazione)
// e spegne il PWM. Se il link cade (nessun comando per ROTOR_TIMEOUT_MS durante
// l'aggancio) il rotore si centra da solo per sicurezza.
//
// NUOVO: /api/passload (HTTP POST) - stesso ruolo di RCMD_PASS_LOAD, ma per un "cervello"
// alternativo al Cardputer (es. un'app web sul telefono) che parla via WiFi/HTTP invece di
// ESP-NOW. Chi manda il pacchetto deve aver gia' fatto calibrazione/centro-arco/eventuale
// flip, esattamente come fa oggi il Cardputer prima di un RCMD_PASS_LOAD: il rotore resta
// "muto" anche qui, riceve solo angoli servo gia' pronti nel tempo.
//
// NUOVO: /app (HTTP GET) - serve direttamente dal rotore la pagina web satellitare completa
// (TLE, calcolo passaggio con satellite.js, DEMO, calibrazione, invio a /api/passload). Evita
// tutti i problemi di aprire un file .html locale sul telefono (specialmente iPhone): basta
// collegarsi al WiFi del rotore e andare su http://192.168.4.1/app con Safari. Il calcolo di
// un passaggio REALE richiede comunque che satellite.js e il TLE siano scaricabili (serve
// internet, es. dati cellulare attivi mentre si e' collegati al WiFi del rotore - su iPhone/
// Android questo passa di solito in automatico sui dati mobili se il WiFi non ha internet).
// Senza internet in quel momento resta sempre disponibile il TLE caricato da file locale e
// il bottone DEMO (passaggio fittizio, nessun calcolo orbitale necessario).
//
// Primo avvio:
//   - LED lampeggia lento = AP attivo, pronto per pairing
//   - Dal telefono collegati alla rete "OLR_Rotor" (pass: "rotoresat")
//   - Apri http://192.168.4.1 nel browser per puntamento manuale
//   - Apri http://192.168.4.1/app per l'app satellitare completa
//   - Dal Cardputer: schermata ROTORE -> "Cerca ricevitore"

#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <ESP32Servo.h>
#include <math.h>
#include "RotorProto.h"

// ============ Pin ============
constexpr int PIN_SERVO_AZ = 32;
constexpr int PIN_SERVO_EL = 33;
constexpr int LED_PIN = 2;

// ============ Taratura impulso servo (verificare sul proprio DS3230 Pro 270) ============
// Molti DS3218/25/30 "Pro 270" usano 500-2500us per 0-270 gradi: punto di partenza,
// da affinare se a fine corsa il servo sbatte o non arriva al fondo.
constexpr int SERVO_US_MIN = 500;    // corrisponde a 0.0 gradi
constexpr int SERVO_US_MAX = 2500;   // corrisponde a 270.0 gradi
constexpr int32_t SERVO_CENTER_DECIDEG = 1350;   // 135.0 gradi: posizione di park/calibrazione
constexpr int32_t SERVO_MIN_DECIDEG = 0;
constexpr int32_t SERVO_MAX_DECIDEG = 2700;
constexpr int32_t EL_MAX_DECIDEG_DEFAULT = 2250;  // 225 servo = 90 gradi di elevazione sopra l'orizzonte (135=0)

constexpr unsigned long ROTOR_TIMEOUT_MS = 10000;   // se agganciato ma senza comandi: park di sicurezza
constexpr unsigned long PARK_SETTLE_MS = 700;       // attesa che il servo arrivi al centro prima di spegnere il PWM

// ============ Movimento dolce ============
// Con l'antenna montata sul boom, uno scatto istantaneo (es. il park a fine passaggio, o un
// GOTO lontano dalla posizione attuale) e' uno strappo meccanico inutile. Tutti i movimenti
// passano quindi da qui: si fissa solo la posizione "target", e servoSlewTick() ci si avvicina
// gradualmente a velocita' limitata. 30 gradi/secondo e' abbastanza lento da essere dolce ma
// resta ben sopra la velocita' angolare tipica di un passaggio LEO (anche allo zenit), quindi
// non introduce un ritardo percepibile nel tracciamento vero e proprio.
constexpr int32_t SERVO_SLEW_DECIDEG_PER_S = 300;   // 30.0 gradi/secondo
constexpr unsigned long SERVO_SLEW_TICK_MS = 20;

// ============ Stato ============
Preferences prefs;
WebServer http(80);
Servo servoAz, servoEl;

int32_t curAz = SERVO_CENTER_DECIDEG, curEl = SERVO_CENTER_DECIDEG;  // decimi di grado, posizione ATTUALE (reale)
int32_t targetAz = SERVO_CENTER_DECIDEG, targetEl = SERVO_CENTER_DECIDEG;  // decimi di grado, posizione desiderata: servoSlewTick() ci si avvicina gradualmente
unsigned long slewLastMs = 0;
int32_t elMaxDeciDeg = EL_MAX_DECIDEG_DEFAULT;   // limite fisico di sicurezza (boom contro la forcella): mai superato, calibrabile dalla pagina web, persiste in flash
bool servoAttached = false;
bool parked = true;
unsigned long parkDueMs = 0;   // quando spegnere il PWM dopo un park (0 = non in attesa)

char apSsid[33] = "OLR_Rotor";
char apPass[64] = "rotoresat";

uint8_t masterMac[6] = {0,0,0,0,0,0};
bool paired = false;
uint32_t lastSeq = 0;
unsigned long lastCmdMs = 0, ledMs = 0;
bool ledState = false;
constexpr int AP_CHANNEL = 1;

// Coda RX (come la Loop): la callback ESP-NOW gira nel task WiFi e deve fare
// SOLO una copia veloce in coda. Chiamare da li' dentro altre API esp_now
// (send/add_peer/del_peer), scrivere su flash (Preferences) o muovere i servi
// e' esattamente quello che la doc ESP-IDF sconsiglia - ed e' il sospetto
// principale per cui il pairing non arrivava mai a buon fine: lo stack
// ESP-NOW puo' restare instabile/silenzioso dopo una callback "pesante"
// come quella che c'era qui prima. Dichiarata qui, PRIMA di qualunque
// funzione, perche' il generatore di prototipi di Arduino inserisce le sue
// dichiarazioni anticipate subito dopo gli include: se RxItem arrivasse piu'
// in basso (dopo altre funzioni), quelle dichiarazioni anticipate non la
// conoscerebbero ancora e la compilazione fallisce.
constexpr int ROTOR_RX_BUF = sizeof(RotorPassCmd);   // il pacchetto piu' grande possibile (passaggio completo)
struct RxItem { uint8_t mac[6]; uint16_t len; uint8_t data[ROTOR_RX_BUF]; };
QueueHandle_t rotorRxQueue = nullptr;

// ============ Passaggio autonomo (RCMD_PASS_LOAD / /api/passload) ============
// Una volta caricato (da ESP-NOW O da HTTP, e' lo stesso stato condiviso), il rotore lo segue
// da solo interpolando nel tempo col proprio clock, senza bisogno di altri comandi per tutta
// la durata del passaggio.
bool     passActive = false;
unsigned long passStartMs = 0;       // millis() al momento della ricezione del pacchetto (riferimento t=0)
uint32_t passStartDelayMs = 0;       // quanto manca all'AOS da passStartMs
uint32_t passDurationMs = 0;         // durata del passaggio (AOS -> LOS)
uint8_t  passCount = 0;
RotorPassPoint passPts[ROTOR_PASS_MAX_PTS];
unsigned long passLastTickMs = 0;

// ============ Servo ============
static int32_t clampDeg(int32_t d) {
    if (d < SERVO_MIN_DECIDEG) d = SERVO_MIN_DECIDEG;
    if (d > SERVO_MAX_DECIDEG) d = SERVO_MAX_DECIDEG;
    return d;
}
static void servoWrite(Servo& s, int32_t decideg) {
    int us = SERVO_US_MIN + (int)((long)(SERVO_US_MAX - SERVO_US_MIN) * decideg / SERVO_MAX_DECIDEG);
    s.writeMicroseconds(us);
}
static void attachServos() {
    if (servoAttached) return;
    servoAz.setPeriodHertz(50);
    servoEl.setPeriodHertz(50);
    servoAz.attach(PIN_SERVO_AZ, SERVO_US_MIN, SERVO_US_MAX);
    servoEl.attach(PIN_SERVO_EL, SERVO_US_MIN, SERVO_US_MAX);
    servoAttached = true;
}
static void detachServos() {
    if (!servoAttached) return;
    servoAz.detach();
    servoEl.detach();
    servoAttached = false;
}
static void moveTo(int32_t az, int32_t el) {
    attachServos();
    targetAz = clampDeg(az);
    // el non va MAI oltre elMaxDeciDeg, qualunque sia la fonte del comando (ESP-NOW dal Cardputer,
    // HTTP dal telefono, o slider della pagina web): e' il fine corsa software che protegge il
    // boom dalla forcella.
    targetEl = clampDeg(el);
    if (targetEl > elMaxDeciDeg) targetEl = elMaxDeciDeg;
    // non tocca curAz/curEl: ci arriva gradualmente servoSlewTick(). Annulla uno spegnimento
    // PWM eventualmente programmato, visto che si sta per lasciare la posizione di park.
    parkDueMs = 0;
    parked = (curAz == targetAz && curEl == targetEl && targetAz == SERVO_CENTER_DECIDEG && targetEl == SERVO_CENTER_DECIDEG);
}
static void parkServos() {
    moveTo(SERVO_CENTER_DECIDEG, SERVO_CENTER_DECIDEG);
    // "parked" e lo spegnimento del PWM li decide servoSlewTick() quando i servi arrivano
    // DAVVERO al centro, non subito: altrimenti con lo scatto istantaneo di prima il PWM si
    // sarebbe spento mentre il servo era ancora a meta' corsa.
}
// Avvicina gradualmente i servi alla posizione "target" (SERVO_SLEW_DECIDEG_PER_S al secondo),
// invece del salto istantaneo: protegge meccanicamente il boom con l'antenna montata sopra.
// Chiamata ad ogni giro di loop(); si autolimita internamente con SERVO_SLEW_TICK_MS.
static void servoSlewTick() {
    unsigned long now = millis();
    unsigned long dt = now - slewLastMs;
    if (dt < SERVO_SLEW_TICK_MS) return;
    slewLastMs = now;
    int32_t step = (int32_t)((int64_t)SERVO_SLEW_DECIDEG_PER_S * dt / 1000);
    if (step < 1) step = 1;
    if (curAz != targetAz) {
        int32_t d = targetAz - curAz;
        if (d > step) d = step; else if (d < -step) d = -step;
        curAz += d;
        servoWrite(servoAz, curAz);
    }
    if (curEl != targetEl) {
        int32_t d = targetEl - curEl;
        if (d > step) d = step; else if (d < -step) d = -step;
        curEl += d;
        servoWrite(servoEl, curEl);
    }
    bool atTarget = (curAz == targetAz && curEl == targetEl);
    bool isParkTarget = (targetAz == SERVO_CENTER_DECIDEG && targetEl == SERVO_CENTER_DECIDEG);
    bool nowParked = atTarget && isParkTarget;
    if (nowParked && !parked) parkDueMs = now + PARK_SETTLE_MS;   // appena arrivato al centro: spegni il PWM dopo l'assestamento
    parked = nowParked;
}

// ============ Preferences ============
void savePrefs() {
    prefs.putBytes("mac", masterMac, 6);
    prefs.putBool("paired", paired);
    prefs.putString("ssid", apSsid);
    prefs.putString("pass", apPass);
}
void loadPrefs() {
    size_t sz = prefs.getBytes("mac", masterMac, 6);
    paired = prefs.getBool("paired", false) && sz == 6;
    String s = prefs.getString("ssid", apSsid);  strncpy(apSsid, s.c_str(), sizeof(apSsid)-1); apSsid[sizeof(apSsid)-1]=0;
    String p = prefs.getString("pass", apPass);  strncpy(apPass, p.c_str(), sizeof(apPass)-1); apPass[sizeof(apPass)-1]=0;
    elMaxDeciDeg = prefs.getInt("elmax", EL_MAX_DECIDEG_DEFAULT);
}

// ============ ESP-NOW ============
void addPeer(const uint8_t* mac) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = AP_CHANNEL; p.encrypt = false;
    p.ifidx = WIFI_IF_AP;      // la scheda e' solo Access Point: senza questo l'invio fallisce (ESP_ERR_ESPNOW_IF)
    esp_now_del_peer(mac);
    esp_now_add_peer(&p);
}
void sendStatus() {
    if (!paired) return;
    RotorStatus st{};
    st.magic = ROTOR_MAGIC; st.type = RRSP_STATUS; st.proto = ROTOR_PROTO;
    st.flags = (paired ? RFLAG_PAIRED : 0) | (parked ? RFLAG_PARKED : 0) | (passActive ? RFLAG_PASS : 0);
    st.az = (int16_t)curAz; st.el = (int16_t)curEl;
    st.fwVersion = 100;
    st.lastSeq = lastSeq;
    esp_err_t r = esp_now_send(masterMac, (uint8_t*)&st, sizeof(st));
    Serial.printf("[ROTORE-TX] sendStatus r=%d to=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  (int)r, masterMac[0], masterMac[1], masterMac[2], masterMac[3], masterMac[4], masterMac[5]);
}

void onRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
    if (!rotorRxQueue || len <= 0 || len > ROTOR_RX_BUF) return;
    RxItem it;
    memcpy(it.mac, info->src_addr, 6);
    it.len = (uint16_t)len;
    memcpy(it.data, data, len);
    xQueueSend(rotorRxQueue, &it, 0);
}

void handlePacket(const RxItem& it) {
    const uint8_t* mac = it.mac;
    const uint8_t* data = it.data;
    int len = it.len;
    Serial.printf("[ROTORE-RX] len=%d (atteso %d o %d) magic=%02X type=%02X from=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  len, (int)sizeof(RotorCmd), (int)sizeof(RotorPassCmd), len > 0 ? data[0] : 0, len > 1 ? data[1] : 0,
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Pacchetto "carica passaggio" (piu' grande del comando normale: lo distinguiamo dalla lunghezza)
    if (len == sizeof(RotorPassCmd)) {
        RotorPassCmd pc; memcpy(&pc, data, sizeof(pc));
        if (pc.magic != ROTOR_MAGIC || pc.type != RCMD_PASS_LOAD) return;
        if (!paired || memcmp(mac, masterMac, 6) != 0) return;
        if (pc.seq != 0 && pc.seq == lastSeq) { sendStatus(); return; }  // ritrasmissione: solo risposta
        lastSeq = pc.seq;
        lastCmdMs = millis();
        passCount = pc.count > ROTOR_PASS_MAX_PTS ? ROTOR_PASS_MAX_PTS : pc.count;
        memcpy(passPts, pc.pts, passCount * sizeof(RotorPassPoint));
        passStartDelayMs = pc.startDelayMs;
        passDurationMs = pc.durationMs;
        passStartMs = millis();
        passLastTickMs = 0;
        passActive = (passCount >= 2);
        Serial.printf("[ROTORE-PASS] caricato: punti=%u delay=%lums durata=%lums\n",
                      passCount, (unsigned long)passStartDelayMs, (unsigned long)passDurationMs);
        sendStatus();
        return;
    }

    if (len != sizeof(RotorCmd)) return;
    RotorCmd p; memcpy(&p, data, sizeof(p));
    if (p.magic != ROTOR_MAGIC) return;
    if (p.type == RCMD_PAIR) {
        memcpy(masterMac, mac, 6); paired = true;
        addPeer(masterMac); savePrefs();
        passActive = false;           // nuovo aggancio: cancella un eventuale passaggio residuo
        parkServos();                 // appena agganciato: centro di riferimento, come da taratura
        sendStatus();
        return;
    }
    if (!paired || memcmp(mac, masterMac, 6) != 0) return;
    if (p.seq != 0 && p.seq == lastSeq) { sendStatus(); return; }  // ritrasmissione: solo risposta
    lastSeq = p.seq;
    lastCmdMs = millis();
    switch (p.type) {
        case RCMD_GOTO: passActive = false; moveTo(p.az, p.el); break;   // un GOTO manuale interrompe il tracciamento autonomo
        case RCMD_PARK: passActive = false; parkServos(); break;         // STOP/PARK: cancella il passaggio e centra
        case RCMD_STOP: passActive = false; targetAz = curAz; targetEl = curEl; break;   // ferma davvero il movimento in corso (non solo il passaggio), resta dove e'
        case RCMD_GET:  break;
    }
    sendStatus();
}

// Segue il passaggio caricato interpolando tra i punti in base al tempo trascorso dal proprio
// clock (millis()): non ha bisogno di altri comandi dal Cardputer/telefono per tutta la durata.
void passTick() {
    if (!passActive) return;
    unsigned long now = millis();
    if (now - passLastTickMs < 1000) return;   // aggiorna una volta al secondo, come il GOTO normale
    passLastTickMs = now;

    unsigned long elapsed = now - passStartMs;
    if (elapsed < passStartDelayMs) {
        // in attesa dell'AOS: si posiziona gia' sul primo punto del passaggio (dove sorgera'
        // il satellite) invece di restare al centro - cosi' si vede a vista che il passaggio
        // e' stato caricato bene, e l'antenna e' gia' pronta quando arriva l'AOS.
        if (passCount >= 1) moveTo(passPts[0].az, passPts[0].el);
        Serial.printf("[PASS-DEBUG] attesa AOS: elapsed=%lu delay=%lu (restano %lums) pt0 az=%d el=%d\n",
                      elapsed, (unsigned long)passStartDelayMs, (unsigned long)passStartDelayMs - elapsed,
                      passPts[0].az, passPts[0].el);
        return;
    }
    unsigned long t = elapsed - passStartDelayMs;
    if (t >= passDurationMs || passCount < 2) {
        Serial.printf("[PASS-DEBUG] FINE passaggio: t=%lu durata=%lu passCount=%u -> park\n",
                      t, (unsigned long)passDurationMs, passCount);
        passActive = false;           // fine passaggio: torna in park
        parkServos();
        return;
    }
    float frac = (float)t / (float)passDurationMs * (passCount - 1);
    int idx = (int)frac;
    if (idx >= (int)passCount - 1) idx = (int)passCount - 2;
    if (idx < 0) idx = 0;
    float localFrac = frac - idx;
    int32_t az = passPts[idx].az + (int32_t)lroundf((passPts[idx + 1].az - passPts[idx].az) * localFrac);
    int32_t el = passPts[idx].el + (int32_t)lroundf((passPts[idx + 1].el - passPts[idx].el) * localFrac);
    Serial.printf("[PASS-DEBUG] tracking: t=%lu/%lu idx=%d/%u az=%d el=%d (elMax=%ld)\n",
                  t, (unsigned long)passDurationMs, idx, passCount, (int)az, (int)el, (long)elMaxDeciDeg);
    moveTo(az, el);        // il clamp di elMaxDeciDeg si applica comunque, qui come ovunque
    lastCmdMs = now;       // conta come attivita' (ridondante col bypass esplicito sotto, ma innocuo)
}

// ============ Pagina web (puntamento manuale di scorta) ============
const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="it"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>OLR Rotore</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}
body{margin:0;padding:12px;font-family:system-ui,sans-serif;background:#111;color:#eee;max-width:640px;margin:auto}
h1{margin:0 0 10px;font-size:18px;color:#8f8}
.card{background:#1b1b1b;border:1px solid #333;border-radius:8px;padding:12px;margin-bottom:10px}
label{display:block;margin:6px 0 2px;font-size:12px;color:#aaa}
.row{display:flex;gap:6px;margin-top:8px}
.row>*{flex:1}
button{background:#2a2a2a;color:#fff;border:1px solid #444;border-radius:6px;padding:14px 6px;font-size:16px;font-weight:bold;cursor:pointer;min-height:48px}
button:active{background:#444}
button.red{background:#833;color:#fff}
input[type=range]{width:100%}
input[type=number]{background:#2a2a2a;color:#fff;border:1px solid #444;border-radius:6px;font-size:22px;text-align:center;padding:10px 4px;width:100%}
.big{font-size:42px;font-weight:bold;text-align:center;color:#8f8}
.lbl{text-align:center;color:#aaa;font-size:12px}
</style></head><body>
<h1>OLR Rotore az/el</h1>
<div class="card">
  <div class="lbl">AZIMUT servo</div>
  <div class="big" id="azv">--</div>
  <input type="range" id="az" min="0" max="270" step="0.5" oninput="azv.textContent=this.value" onchange="send()">
  <div class="lbl">ELEVAZIONE servo</div>
  <div class="big" id="elv">--</div>
  <input type="range" id="el" min="0" max="270" step="0.5" oninput="elv.textContent=this.value" onchange="send()">
  <div class="row">
    <button onclick="park()">CENTRA (135/135)</button>
  </div>
</div>
<div class="card">
  <div class="lbl">TEST VERSO ROTAZIONE AZIMUT (guarda dall'alto, verifica il verso reale)</div>
  <div class="row">
    <button onclick="nudgeAz(-10)">&#9664; ANTIORARIO</button>
    <button onclick="nudgeAz(10)">ORARIO &#9654;</button>
  </div>
</div>
<div class="card">
  <div class="lbl">LIMITE MASSIMO ELEVAZIONE (mai superato)</div>
  <div class="big" id="elmaxv">--</div>
  <div class="lbl" style="margin-top:10px">Scrivi il grado esatto da provare (lo slider sopra si "aggiorna" ogni secondo e non e' comodo per i test fini) e premi VAI. Quando sei vicino al punto fisico sicuro, premi IMPOSTA QUI.</div>
  <div class="row">
    <input type="number" id="eltest" min="0" max="270" step="0.5" value="135">
    <button onclick="gotoElTest()">VAI</button>
  </div>
  <div class="row">
    <button onclick="setElMax()">IMPOSTA QUI</button>
    <button class="red" onclick="resetElMax()">DEFAULT (90°)</button>
  </div>
</div>
<script>
function send(){
  let az=document.getElementById('az').value, el=document.getElementById('el').value;
  fetch('/api/goto?az='+az+'&el='+el).then(poll);
}
function park(){ fetch('/api/park').then(poll); }
function nudgeAz(delta){
  // sposta l'azimut di 'delta' gradi rispetto alla posizione ATTUALE, per verificare a vista
  // (guardando il rotore dall'alto) se "ORARIO" fa davvero girare in senso orario sulla bussola
  let az = parseFloat(document.getElementById('az').value) + delta;
  if (az < 0) az = 0; if (az > 270) az = 270;
  let el = document.getElementById('el').value;
  fetch('/api/goto?az='+az+'&el='+el).then(poll);
}
function gotoElTest(){
  // usa l'azimut ATTUALE (dall'ultimo stato letto), cambia solo l'elevazione al valore scritto a mano
  let az = document.getElementById('az').value;
  let el = document.getElementById('eltest').value;
  fetch('/api/goto?az='+az+'&el='+el).then(poll);
}
function setElMax(){
  if(!confirm('Impostare il limite massimo di elevazione alla posizione ATTUALE del servo?')) return;
  fetch('/api/setelmax').then(r=>{ if(!r.ok) r.text().then(t=>alert(t)); }).then(poll);
}
function resetElMax(){ if(confirm('Ripristinare il limite di elevazione al default (90 gradi)?')) fetch('/api/resetelmax').then(poll); }
function poll(){
  fetch('/api/ping');   // tiene "vivo" il rotore (resetta il timeout di sicurezza) mentre la pagina resta aperta
  fetch('/api/status').then(r=>r.json()).then(s=>{
    document.getElementById('az').value=s.az; document.getElementById('azv').textContent=s.az.toFixed(1);
    document.getElementById('el').value=s.el; document.getElementById('elv').textContent=s.el.toFixed(1);
    document.getElementById('elmaxv').textContent=s.elmax.toFixed(1)+'°';
  });
}
setInterval(poll,1000);poll();
</script></body></html>
)HTML";

const char APP_HTML[] PROGMEM = R"APPPAGE(
<!DOCTYPE html>
<html lang="it">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0">
<title>OLR Rotore Satellitare</title>
<script src="https://cdn.jsdelivr.net/npm/satellite.js@5/dist/satellite.min.js"></script>
<style>
  :root{
    --bg:#0f1720; --panel:#17222c; --panel2:#1d2a36; --text:#e6edf3; --muted:#8aa0b2;
    --accent:#3fa7ff; --ok:#3ecf8e; --warn:#f0a93e; --err:#ff5c5c; --border:#263544;
  }
  *{box-sizing:border-box;}
  body{
    margin:0; padding:0; background:var(--bg); color:var(--text);
    font-family:-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;
    font-size:16px; line-height:1.4;
  }
  header{padding:16px; background:var(--panel2); border-bottom:1px solid var(--border);}
  header h1{margin:0; font-size:1.2em;}
  header p{margin:4px 0 0; color:var(--muted); font-size:0.85em;}
  main{padding:12px; max-width:720px; margin:0 auto;}
  section{
    background:var(--panel); border:1px solid var(--border); border-radius:10px;
    padding:14px; margin-bottom:14px;
  }
  section h2{margin:0 0 10px; font-size:1em; color:var(--accent);}
  label{display:block; margin:10px 0 4px; font-size:0.9em; color:var(--muted);}
  input[type=text], input[type=number], select{
    width:100%; padding:10px; border-radius:8px; border:1px solid var(--border);
    background:var(--panel2); color:var(--text); font-size:1em;
  }
  input[type=checkbox]{width:auto; transform:scale(1.3); margin-right:8px; vertical-align:middle;}
  .row{display:flex; gap:10px;}
  .row > div{flex:1;}
  button{
    display:inline-block; padding:12px 16px; margin:6px 4px 0 0; border:none; border-radius:8px;
    background:var(--accent); color:#06141f; font-weight:600; font-size:0.95em; cursor:pointer;
  }
  button.secondary{background:var(--panel2); color:var(--text); border:1px solid var(--border);}
  button.ok{background:var(--ok); color:#06241a;}
  button.warn{background:var(--warn); color:#271a00;}
  button:disabled{opacity:0.4; cursor:not-allowed;}
  .pill{display:inline-block; padding:2px 8px; border-radius:999px; font-size:0.78em; font-weight:600;}
  .pill.ok{background:rgba(62,207,142,0.15); color:var(--ok);}
  .pill.warn{background:rgba(240,169,62,0.15); color:var(--warn);}
  .pill.err{background:rgba(255,92,92,0.15); color:var(--err);}
  .muted{color:var(--muted); font-size:0.85em;}
  table{width:100%; border-collapse:collapse; font-size:0.82em; margin-top:8px;}
  th,td{padding:4px 6px; text-align:right; border-bottom:1px solid var(--border);}
  th:first-child, td:first-child{text-align:left;}
  #log{
    background:#0a1118; border:1px solid var(--border); border-radius:8px; padding:8px;
    max-height:160px; overflow-y:auto; font-family:monospace; font-size:0.78em; white-space:pre-wrap;
  }
  .log-err{color:var(--err);} .log-ok{color:var(--ok);} .log-warn{color:var(--warn);}
  details summary{cursor:pointer; color:var(--muted); font-size:0.85em; margin-top:6px;}
  .trackbox{max-height:220px; overflow-y:auto;}
</style>
</head>
<body>
<header>
  <h1>OLR &middot; Rotore Satellitare</h1>
  <p>Calcolo passaggi e invio al rotore &middot; <span id="libStatus" class="pill warn">libreria orbitale: caricamento...</span></p>
</header>

<main>

  <section id="sec-tle">
    <h2>1. TLE</h2>
    <div class="row">
      <div><button id="btnFetchTle">Scarica da Celestrak</button></div>
      <div><button class="secondary" id="btnPickFile">Carica file locale</button>
        <input type="file" id="fileTle" accept=".txt" style="position:absolute; width:1px; height:1px; opacity:0; overflow:hidden; pointer-events:none;"></div>
    </div>
    <p class="muted" id="tleStatus">Nessun TLE caricato.</p>
    <label for="selSat">Satellite</label>
    <select id="selSat"><option value="">-- carica un TLE --</option></select>
    <div id="satInfo" class="muted" style="margin-top:8px; display:none">
      NORAD <span id="infoNorad">-</span> &middot; Inclinazione <span id="infoIncl">-</span>&deg; &middot; Periodo orbitale <span id="infoPeriod">-</span> min
    </div>
  </section>

  <section id="sec-obs">
    <h2>2. Posizione osservatore</h2>
    <label for="inLocator">Locatore (es. JN45so)</label>
    <input type="text" id="inLocator" placeholder="6 caratteri" maxlength="6">
    <details>
      <summary>Inserisci lat/lon/quota manualmente invece</summary>
      <div class="row">
        <div><label for="inLat">Lat (&deg;)</label><input type="number" id="inLat" step="0.0001"></div>
        <div><label for="inLon">Lon (&deg;)</label><input type="number" id="inLon" step="0.0001"></div>
      </div>
    </details>
    <label for="inAlt">Quota (m)</label>
    <input type="number" id="inAlt" value="0">
    <p class="muted" id="obsStatus">Nessuna posizione impostata.</p>
  </section>

  <section id="sec-pass">
    <h2>3. Passaggio</h2>
    <div class="row">
      <div><button id="btnCalcPass">Calcola prossimo passaggio</button></div>
      <div><button class="warn" id="btnDemo">DEMO (test rotore)</button></div>
    </div>
    <div id="countdownBox" style="display:none; margin:10px 0; text-align:center;">
      <div class="muted">Prossimo passaggio (<span id="cdSatName"></span>) tra</div>
      <div id="countdownText" style="font-size:1.9em; font-weight:700; color:var(--accent);">--:--:--</div>
    </div>
    <p class="muted" id="passStatus">Nessun passaggio calcolato.</p>
    <details id="trackDetails" style="display:none">
      <summary>Mostra i punti del tracciato</summary>
      <div class="trackbox">
        <table id="trackTable">
          <thead><tr><th>t (s)</th><th>Az&deg;</th><th>El&deg;</th><th>Servo Az</th><th>Servo El</th></tr></thead>
          <tbody></tbody>
        </table>
      </div>
    </details>
  </section>

  <section id="sec-align" style="display:none">
    <h2>4. Allineamento (come sul Cardputer)</h2>
    <p>Centro arco di questo passaggio: <b id="alignCenterArc">-</b>&deg;</p>
    <p class="muted">Porta il rotore al centro meccanico, poi ruota il <b>treppiede</b> (non i servi) finch&eacute; il boom non punta esattamente a questa direzione sulla bussola.</p>
    <button class="secondary" id="btnGoCenter">1. Porta il rotore al centro (135&deg;/135&deg;)</button>
    <div class="row" style="margin-top:10px">
      <div><label><input type="checkbox" id="chkAligned"> Ho ruotato il treppiede e il boom punta a <span id="alignCenterArc2">-</span>&deg;</label></div>
    </div>
    <p class="muted" id="alignStatus">Calcola un passaggio per vedere il centro arco da allineare.</p>
  </section>

  <section id="sec-calib">
    <h2>5. Calibrazione</h2>
    <label><input type="checkbox" id="chkReverse"> Inverti senso azimut (azReverse)</label>
    <div class="row">
      <div><label for="inTrimAz">Trim Az (&deg;)</label><input type="number" id="inTrimAz" value="0"></div>
      <div><label for="inTrimEl">Trim El (&deg;)</label><input type="number" id="inTrimEl" value="0"></div>
    </div>
    <button class="secondary" id="btnSaveCalib">Salva calibrazione</button>
    <p class="muted" id="calibStatus"></p>
  </section>

  <section id="sec-send">
    <h2>6. Invio al rotore</h2>
    <label for="inRotorIp">IP / host rotore</label>
    <input type="text" id="inRotorIp" value="192.168.4.1">
    <button class="ok" id="btnSend" disabled>2. Invia passaggio al rotore</button>
    <p class="muted" id="sendStatus">Calcola un passaggio, allineati al centro arco e conferma sopra prima di inviare.</p>
  </section>

  <section id="sec-log">
    <h2>Registro</h2>
    <div id="log"></div>
  </section>

</main>

<script>
"use strict";

// ---------- stato ----------
let tleList = [];           // [{name, l1, l2}]
let currentTrack = null;    // {pts:[{t,az,el}], aos, los, centerArc, isDemo}
let calib = {azReverse:true, trimAz:0, trimEl:0};  // default coerente col Cardputer (azimut invertito di serie)

// ---------- utility log ----------
function log(msg, cls){
  const el = document.getElementById('log');
  const line = document.createElement('div');
  if (cls) line.className = 'log-'+cls;
  const t = new Date().toLocaleTimeString();
  line.textContent = `[${t}] ${msg}`;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}

// ---------- libreria satellite.js ----------
function checkLib(){
  const pill = document.getElementById('libStatus');
  if (window.satellite) {
    pill.textContent = 'libreria orbitale: pronta';
    pill.className = 'pill ok';
    log('satellite.js caricata correttamente.', 'ok');
  } else {
    pill.textContent = 'libreria orbitale: NON disponibile (serve internet al primo avvio)';
    pill.className = 'pill err';
    log('satellite.js non caricata: senza internet il calcolo dei passaggi reali non funziona (il DEMO funziona comunque).', 'warn');
  }
}
window.addEventListener('load', () => setTimeout(checkLib, 300));

// ---------- localStorage ----------
function loadPrefs(){
  try{
    const c = JSON.parse(localStorage.getItem('olr_calib') || 'null');
    if (c) calib = c;
  }catch(e){}
  document.getElementById('chkReverse').checked = !!calib.azReverse;
  document.getElementById('inTrimAz').value = calib.trimAz || 0;
  document.getElementById('inTrimEl').value = calib.trimEl || 0;

  const loc = localStorage.getItem('olr_locator');
  if (loc) document.getElementById('inLocator').value = loc;
  const ip = localStorage.getItem('olr_rotor_ip');
  if (ip) document.getElementById('inRotorIp').value = ip;

  try{
    const cache = JSON.parse(localStorage.getItem('olr_tle_cache') || 'null');
    if (cache && cache.list && cache.list.length){
      tleList = cache.list;
      populateSatSelect();
      const age = Math.round((Date.now() - cache.ts)/60000);
      document.getElementById('tleStatus').textContent =
        `TLE in cache (${tleList.length} satelliti, scaricati ${age} min fa).`;
    }
  }catch(e){}
}
function saveCalib(){
  calib.azReverse = document.getElementById('chkReverse').checked;
  calib.trimAz = parseFloat(document.getElementById('inTrimAz').value) || 0;
  calib.trimEl = parseFloat(document.getElementById('inTrimEl').value) || 0;
  localStorage.setItem('olr_calib', JSON.stringify(calib));
  document.getElementById('calibStatus').textContent = 'Calibrazione salvata.';
  log('Calibrazione salvata: reverse=' + calib.azReverse + ' trimAz=' + calib.trimAz + ' trimEl=' + calib.trimEl, 'ok');
}
document.getElementById('btnSaveCalib').addEventListener('click', saveCalib);

// ---------- locatore -> lat/lon ----------
function locatorToLatLon(loc){
  loc = loc.trim().toUpperCase();
  if (loc.length < 4) return null;
  const A = 'A'.charCodeAt(0), ZERO = '0'.charCodeAt(0);
  let lon = (loc.charCodeAt(0)-A)*20 - 180;
  let lat = (loc.charCodeAt(1)-A)*10 - 90;
  lon += (loc.charCodeAt(2)-ZERO)*2;
  lat += (loc.charCodeAt(3)-ZERO)*1;
  if (loc.length >= 6) {
    lon += (loc.charCodeAt(4)-A)*(2/24);
    lat += (loc.charCodeAt(5)-A)*(1/24);
    lon += (2/24)/2; lat += (1/24)/2;
  } else {
    lon += 1; lat += 0.5;
  }
  return {lat, lon};
}
function getObserver(){
  const latManual = document.getElementById('inLat').value;
  const lonManual = document.getElementById('inLon').value;
  let lat, lon;
  if (latManual !== '' && lonManual !== '') {
    lat = parseFloat(latManual); lon = parseFloat(lonManual);
  } else {
    const loc = document.getElementById('inLocator').value;
    const ll = locatorToLatLon(loc);
    if (!ll) return null;
    lat = ll.lat; lon = ll.lon;
  }
  const altM = parseFloat(document.getElementById('inAlt').value) || 0;
  return {lat, lon, heightKm: altM/1000};
}
function updateObsStatus(){
  const obs = getObserver();
  const s = document.getElementById('obsStatus');
  if (obs) {
    s.textContent = `Lat ${obs.lat.toFixed(4)}  Lon ${obs.lon.toFixed(4)}  Quota ${(obs.heightKm*1000).toFixed(0)} m`;
    localStorage.setItem('olr_locator', document.getElementById('inLocator').value);
  } else {
    s.textContent = 'Inserisci un locatore valido (6 caratteri) o lat/lon manuali.';
  }
}
document.getElementById('inLocator').addEventListener('input', () => { updateObsStatus(); recalcPass(true); });
document.getElementById('inLat').addEventListener('input', () => { updateObsStatus(); recalcPass(true); });
document.getElementById('inLon').addEventListener('input', () => { updateObsStatus(); recalcPass(true); });
document.getElementById('inAlt').addEventListener('input', () => { updateObsStatus(); recalcPass(true); });

// ---------- TLE: parsing ----------
function parseTripletsText(text){
  const lines = text.split(/\r?\n/).map(l => l).filter(l => l.trim().length > 0);
  const out = [];
  let i = 0;
  while (i + 2 < lines.length) {
    const name = lines[i], l1 = lines[i+1], l2 = lines[i+2];
    if (l1 && l2 && l1.startsWith('1 ') && l2.startsWith('2 ')) {
      out.push({name: name.trim(), l1, l2});
      i += 3;
    } else {
      i += 1;
    }
  }
  return out;
}
function populateSatSelect(){
  const sel = document.getElementById('selSat');
  sel.innerHTML = '';
  if (!tleList.length) {
    sel.innerHTML = '<option value="">-- carica un TLE --</option>';
    return;
  }
  tleList.forEach((s, idx) => {
    const opt = document.createElement('option');
    opt.value = idx;
    opt.textContent = s.name;
    sel.appendChild(opt);
  });
}
function parseSatInfo(sat){
  const norad = parseInt(sat.l1.substr(2,5), 10);
  const incl = parseFloat(sat.l2.substr(8,8));
  const revDay = parseFloat(sat.l2.substr(52,11));
  const periodMin = revDay > 0 ? 1440 / revDay : null;
  return {norad, incl, periodMin};
}
function showSatInfo(sat){
  const info = parseSatInfo(sat);
  document.getElementById('infoNorad').textContent = isNaN(info.norad) ? '-' : info.norad;
  document.getElementById('infoIncl').textContent = isNaN(info.incl) ? '-' : info.incl.toFixed(1);
  document.getElementById('infoPeriod').textContent = info.periodMin ? info.periodMin.toFixed(1) : '-';
  document.getElementById('satInfo').style.display = 'block';
}
document.getElementById('selSat').addEventListener('change', (e) => {
  const idx = e.target.value;
  if (idx === '') { document.getElementById('satInfo').style.display = 'none'; return; }
  showSatInfo(tleList[idx]);
  recalcPass(true);
});

// ---- caricamento file locale ----
document.getElementById('btnPickFile').addEventListener('click', () => {
  document.getElementById('fileTle').click();
});
document.getElementById('fileTle').addEventListener('change', (ev) => {
  const file = ev.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    const parsed = parseTripletsText(reader.result);
    if (!parsed.length) {
      log('Il file non contiene TLE validi.', 'err');
      document.getElementById('tleStatus').textContent = 'File non valido.';
      return;
    }
    tleList = parsed;
    populateSatSelect();
    localStorage.setItem('olr_tle_cache', JSON.stringify({list: tleList, ts: Date.now()}));
    document.getElementById('tleStatus').textContent =
      `TLE caricati da file: ${tleList.length} satelliti (${file.name}).`;
    log(`Caricati ${tleList.length} satelliti da file locale.`, 'ok');
  };
  reader.readAsText(file);
});

// ---- download da Celestrak ----
const GROUP_URL = 'https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=tle';
const CATNR_URL = (norad) => `https://celestrak.org/NORAD/elements/gp.php?CATNR=${norad}&FORMAT=tle`;
// stessa lista usata da tle_downloader.py / sats.txt del Cardputer
const TARGET_SATS = [
  ["SO-50",27607], ["AO-91",43017], ["AO-123",61781], ["ISS",25544],
  ["PO-101",43678], ["CAS-3H",40908], ["RS95S",67291], ["RS-44",44909],
  ["FO-29",24278], ["AO-7",7530], ["AO-73",39444], ["JO-97",43803], ["TO-108",44881],
];
async function fetchText(url){
  const r = await fetch(url, {cache:'no-store'});
  if (!r.ok) throw new Error('HTTP ' + r.status);
  return await r.text();
}
document.getElementById('btnFetchTle').addEventListener('click', async () => {
  const btn = document.getElementById('btnFetchTle');
  btn.disabled = true;
  document.getElementById('tleStatus').textContent = 'Scarico da Celestrak...';
  log('Scarico gruppo amateur da Celestrak...');
  try {
    const bulkText = await fetchText(GROUP_URL);
    const bulk = parseTripletsText(bulkText);
    const byNorad = {};
    bulk.forEach(s => { byNorad[parseInt(s.l1.substr(2,5),10)] = s; });

    const found = [];
    for (const [name, norad] of TARGET_SATS) {
      if (byNorad[norad]) {
        found.push({name, l1: byNorad[norad].l1, l2: byNorad[norad].l2});
      } else {
        log(`  ${name} (NORAD ${norad}) non nel gruppo bulk, provo singolarmente...`);
        try {
          const single = parseTripletsText(await fetchText(CATNR_URL(norad)));
          if (single.length) found.push({name, l1: single[0].l1, l2: single[0].l2});
        } catch(e) { log(`    errore CATNR ${norad}: ${e}`, 'err'); }
      }
    }
    if (!found.length) throw new Error('nessun satellite trovato');
    tleList = found;
    populateSatSelect();
    localStorage.setItem('olr_tle_cache', JSON.stringify({list: tleList, ts: Date.now()}));
    document.getElementById('tleStatus').textContent =
      `TLE scaricati: ${found.length}/${TARGET_SATS.length} satelliti.`;
    log(`OK: ${found.length}/${TARGET_SATS.length} satelliti scaricati da Celestrak.`, 'ok');
  } catch (e) {
    log('Errore download TLE: ' + e, 'err');
    document.getElementById('tleStatus').textContent = 'Errore download (serve internet). TLE precedenti, se presenti, restano validi.';
  } finally {
    btn.disabled = false;
  }
});

// ---------- SGP4 / ricerca passaggio ----------
function elevationAt(satrec, date, obsLatRad, obsLonRad, obsHeightKm){
  const pv = satellite.propagate(satrec, date);
  if (!pv || !pv.position) return -Math.PI/2;
  const gmst = satellite.gstime(date);
  const ecf = satellite.eciToEcf(pv.position, gmst);
  const observerGd = {longitude: obsLonRad, latitude: obsLatRad, height: obsHeightKm};
  const look = satellite.ecfToLookAngles(observerGd, ecf);
  return look.elevation;
}
function lookAnglesAt(satrec, date, obsLatRad, obsLonRad, obsHeightKm){
  const pv = satellite.propagate(satrec, date);
  const gmst = satellite.gstime(date);
  const ecf = satellite.eciToEcf(pv.position, gmst);
  const observerGd = {longitude: obsLonRad, latitude: obsLatRad, height: obsHeightKm};
  const look = satellite.ecfToLookAngles(observerGd, ecf);
  return {az: look.azimuth * 180/Math.PI, el: look.elevation * 180/Math.PI};
}
function bisect(satrec, t1, t2, latRad, lonRad, hKm, thresholdRad, rising){
  for (let i=0; i<24; i++){
    const tm = new Date((t1.getTime()+t2.getTime())/2);
    const el = elevationAt(satrec, tm, latRad, lonRad, hKm);
    const above = el >= thresholdRad;
    if (rising ? !above : above) t1 = tm; else t2 = tm;
  }
  return new Date((t1.getTime()+t2.getTime())/2);
}
function findNextPass(satrec, latRad, lonRad, hKm, minElevDeg, windowHours, stepSeconds){
  const thresholdRad = minElevDeg * Math.PI/180;
  const start = new Date();
  const end = new Date(start.getTime() + windowHours*3600*1000);
  let t = new Date(start);
  let prevEl = null, prevT = null, aos = null;
  while (t <= end) {
    const el = elevationAt(satrec, t, latRad, lonRad, hKm);
    if (prevEl !== null) {
      if (!aos && prevEl < thresholdRad && el >= thresholdRad) {
        aos = bisect(satrec, prevT, t, latRad, lonRad, hKm, thresholdRad, true);
      }
      if (aos && prevEl >= thresholdRad && el < thresholdRad) {
        const los = bisect(satrec, prevT, t, latRad, lonRad, hKm, thresholdRad, false);
        return {aos, los};
      }
    }
    prevEl = el; prevT = new Date(t);
    t = new Date(t.getTime() + stepSeconds*1000);
  }
  return null;
}
function sampleTrack(satrec, aos, los, latRad, lonRad, hKm, numPts){
  const pts = [];
  const totalMs = los.getTime() - aos.getTime();
  for (let i=0; i<numPts; i++){
    const t = new Date(aos.getTime() + totalMs * i/(numPts-1));
    const la = lookAnglesAt(satrec, t, latRad, lonRad, hKm);
    pts.push({t, az: la.az, el: la.el});
  }
  return pts;
}
function centerOfArc(azDegList){
  const az = azDegList.slice().sort((a,b)=>a-b);
  const n = az.length;
  if (n === 0) return 0;
  let maxGap = -1, gapIdx = 0;
  for (let i=0; i<n; i++){
    const a1 = az[i];
    const a2 = (i+1<n) ? az[i+1] : az[0]+360;
    const gap = a2 - a1;
    if (gap > maxGap) { maxGap = gap; gapIdx = i; }
  }
  const startAngle = az[(gapIdx+1)%n];
  const spanDeg = 360 - maxGap;
  let center = startAngle + spanDeg/2;
  center = ((center % 360) + 360) % 360;
  return center;
}
function angDiff(a, b){
  let d = a - b;
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}
function toServo(azCompass, el, centerArcAz){
  let diff = angDiff(azCompass, centerArcAz);
  if (calib.azReverse) diff = -diff;
  diff = Math.max(-134, Math.min(134, diff));
  const elC = Math.max(0, Math.min(90, el));
  let azServo = 135 + diff + (calib.trimAz || 0);
  let elServo = 135 + elC + (calib.trimEl || 0);
  azServo = Math.max(0, Math.min(270, azServo));
  elServo = Math.max(0, Math.min(270, elServo));
  return {azServo, elServo};
}

function renderTrack(track){
  const tbody = document.querySelector('#trackTable tbody');
  tbody.innerHTML = '';
  const t0 = track.pts[0].t.getTime();
  track.pts.forEach(p => {
    const sv = toServo(p.az, p.el, track.centerArc);
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${Math.round((p.t.getTime()-t0)/1000)}</td><td>${p.az.toFixed(1)}</td><td>${p.el.toFixed(1)}</td><td>${sv.azServo.toFixed(1)}</td><td>${sv.elServo.toFixed(1)}</td>`;
    tbody.appendChild(tr);
  });
  document.getElementById('trackDetails').style.display = 'block';
}

// ---- allineamento (centro arco, come sul Cardputer) ----
function showAlignStep(track){
  document.getElementById('sec-align').style.display = 'block';
  document.getElementById('alignCenterArc').textContent = track.centerArc.toFixed(1);
  document.getElementById('alignCenterArc2').textContent = track.centerArc.toFixed(1);
  document.getElementById('chkAligned').checked = false;
  document.getElementById('alignStatus').textContent = 'In attesa di allineamento e conferma.';
  document.getElementById('btnSend').disabled = true;
}
document.getElementById('chkAligned').addEventListener('change', (e) => {
  document.getElementById('btnSend').disabled = !e.target.checked || !currentTrack;
  document.getElementById('alignStatus').textContent = e.target.checked
    ? 'Allineamento confermato: puoi inviare il passaggio.'
    : 'In attesa di allineamento e conferma.';
});
document.getElementById('btnGoCenter').addEventListener('click', async () => {
  const ip = document.getElementById('inRotorIp').value.trim();
  document.getElementById('alignStatus').textContent = 'Porto il rotore al centro...';
  try {
    const resp = await fetch(`http://${ip}/api/goto?az=135&el=135`);
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    document.getElementById('alignStatus').textContent =
      'Rotore al centro (135°/135°). Ruota il treppiede finché il boom non punta alla direzione indicata, poi conferma qui sotto.';
    log('Rotore portato al centro meccanico per l\'allineamento.', 'ok');
  } catch (e) {
    document.getElementById('alignStatus').textContent = 'Errore collegamento al rotore: ' + e;
    log('Errore portando il rotore al centro: ' + e, 'err');
  }
});

// ---- calcolo passaggio reale ----
let countdownTimer = null;

function fmtDuration(ms){
  if (ms < 0) ms = 0;
  const s = Math.floor(ms/1000);
  const hh = Math.floor(s/3600);
  const mm = Math.floor((s%3600)/60);
  const ss = s%60;
  return (hh>0 ? hh+':' : '') + String(mm).padStart(2,'0') + ':' + String(ss).padStart(2,'0');
}
function updateCountdown(){
  if (!currentTrack) { document.getElementById('countdownBox').style.display = 'none'; return; }
  const box = document.getElementById('countdownBox');
  const txt = document.getElementById('countdownText');
  document.getElementById('cdSatName').textContent = currentTrack.satName;
  const now = Date.now();
  box.style.display = 'block';
  if (now < currentTrack.aos.getTime()) {
    txt.textContent = fmtDuration(currentTrack.aos.getTime() - now);
    txt.style.color = 'var(--accent)';
  } else if (now <= currentTrack.los.getTime()) {
    txt.textContent = 'IN CORSO - termina in ' + fmtDuration(currentTrack.los.getTime() - now);
    txt.style.color = 'var(--ok)';
  } else {
    txt.textContent = 'TERMINATO';
    txt.style.color = 'var(--muted)';
    clearInterval(countdownTimer);
  }
}
function startCountdown(){
  if (countdownTimer) clearInterval(countdownTimer);
  updateCountdown();
  countdownTimer = setInterval(updateCountdown, 1000);
}

function recalcPass(auto){
  if (!window.satellite) { if (!auto) log('satellite.js non disponibile: impossibile calcolare (serve averla caricata con internet almeno una volta in questa sessione).', 'err'); return; }
  const idx = document.getElementById('selSat').value;
  if (idx === '') { if (!auto) log('Seleziona un satellite.', 'err'); return; }
  const obs = getObserver();
  if (!obs) { if (!auto) log('Posizione osservatore non valida.', 'err'); return; }
  const sat = tleList[idx];
  let satrec;
  try { satrec = satellite.twoline2satrec(sat.l1, sat.l2); }
  catch(e) { log('TLE non valido per ' + sat.name + ': ' + e, 'err'); return; }

  const latRad = obs.lat * Math.PI/180;
  const lonRad = obs.lon * Math.PI/180;
  document.getElementById('passStatus').textContent = 'Calcolo in corso...';
  if (!auto) log(`Cerco il prossimo passaggio di ${sat.name}...`);

  const pass = findNextPass(satrec, latRad, lonRad, obs.heightKm, 10, 24, 20);
  if (!pass) {
    document.getElementById('passStatus').textContent = `Nessun passaggio di ${sat.name} sopra 10° nelle prossime 24h.`;
    log(`Nessun passaggio di ${sat.name} trovato nelle prossime 24h.`, 'warn');
    currentTrack = null;
    document.getElementById('btnSend').disabled = true;
    document.getElementById('sec-align').style.display = 'none';
    document.getElementById('countdownBox').style.display = 'none';
    if (countdownTimer) clearInterval(countdownTimer);
    return;
  }
  const pts = sampleTrack(satrec, pass.aos, pass.los, latRad, lonRad, obs.heightKm, 48);
  const centerArc = centerOfArc(pts.map(p => p.az));
  const maxEl = Math.max(...pts.map(p => p.el));
  currentTrack = {pts, aos: pass.aos, los: pass.los, centerArc, isDemo:false, satName: sat.name};

  const durMin = ((pass.los - pass.aos)/60000).toFixed(1);
  document.getElementById('passStatus').innerHTML =
    `<b>${sat.name}</b><br>AOS: ${pass.aos.toLocaleString()}<br>LOS: ${pass.los.toLocaleString()}<br>` +
    `Durata: ${durMin} min &middot; El. max: ${maxEl.toFixed(1)}&deg; &middot; Centro arco: ${centerArc.toFixed(1)}&deg;`;
  renderTrack(currentTrack);
  showAlignStep(currentTrack);
  startCountdown();
  log(`Passaggio trovato: AOS ${pass.aos.toLocaleTimeString()} - LOS ${pass.los.toLocaleTimeString()}, el.max ${maxEl.toFixed(1)}°.`, 'ok');
}
document.getElementById('btnCalcPass').addEventListener('click', () => recalcPass(false));

// ---- DEMO ----
document.getElementById('btnDemo').addEventListener('click', () => {
  const now = Date.now();
  const startDelayMs = 5000;
  const durationMs = 180000; // 3 minuti
  const numPts = 48;
  const pts = [];
  // arco realistico (120 gradi, come un passaggio vero), NON un giro completo 0-360:
  // un giro intero non ci sta fisicamente nei 270 gradi del rotore, qualunque sia il
  // centraggio, e resterebbe "incollato" al limite per una parte del test.
  const azStart = 45, azEnd = 225; // 180 gradi di arco, come il DEMO del Cardputer
  for (let i=0; i<numPts; i++){
    const frac = i/(numPts-1);
    const az = azStart + frac * (azEnd - azStart);
    const el = 5 + 75 * (1 - Math.abs(2*frac - 1));
    pts.push({t: new Date(now + startDelayMs + frac*durationMs), az, el});
  }
  const centerArc = centerOfArc(pts.map(p => p.az));
  currentTrack = {
    pts, aos: new Date(now+startDelayMs), los: new Date(now+startDelayMs+durationMs),
    centerArc, isDemo:true, satName: 'DEMO'
  };
  document.getElementById('passStatus').innerHTML =
    `<b>DEMO</b> &middot; passaggio fittizio per test rotore<br>Durata: 3 min, parte in 5s, arco az ${azStart}&deg;-${azEnd}&deg;, el. 5&deg;-80&deg;.`;
  renderTrack(currentTrack);
  showAlignStep(currentTrack);
  startCountdown();
  log('Passaggio DEMO generato (arco 180°, come il Cardputer, 3 minuti).', 'ok');
});

// ---- invio al rotore ----
document.getElementById('inRotorIp').addEventListener('change', (e) => {
  localStorage.setItem('olr_rotor_ip', e.target.value.trim());
});
document.getElementById('btnSend').addEventListener('click', async () => {
  if (!currentTrack) { log('Nessun passaggio da inviare.', 'err'); return; }
  const ip = document.getElementById('inRotorIp').value.trim();
  const startDelayMs = Math.max(0, currentTrack.aos.getTime() - Date.now());
  const durationMs = currentTrack.los.getTime() - currentTrack.aos.getTime();
  const servoPts = currentTrack.pts.map(p => toServo(p.az, p.el, currentTrack.centerArc));
  const header = `${Math.round(startDelayMs)},${Math.round(durationMs)},${servoPts.length}`;
  const body = header + ';' + servoPts.map(p => `${p.azServo.toFixed(1)},${p.elServo.toFixed(1)}`).join(';');

  document.getElementById('sendStatus').textContent = `Invio a http://${ip}/api/passload ...`;
  log(`Invio passaggio (${currentTrack.satName}, ${servoPts.length} punti) a ${ip}...`);
  try {
    const resp = await fetch(`http://${ip}/api/passload`, {
      method: 'POST',
      headers: {'Content-Type':'text/plain'},
      body
    });
    const text = await resp.text();
    if (resp.ok) {
      document.getElementById('sendStatus').textContent = `Rotore: ${text}`;
      log('Rotore ha risposto OK: ' + text, 'ok');
    } else {
      document.getElementById('sendStatus').textContent = `Errore rotore (${resp.status}): ${text}`;
      log(`Errore rotore (${resp.status}): ${text}`, 'err');
    }
  } catch (e) {
    document.getElementById('sendStatus').textContent = 'Errore di connessione al rotore: ' + e;
    log('Errore di connessione al rotore (sei collegato al suo WiFi?): ' + e, 'err');
  }
});

// ---------- avvio ----------
loadPrefs();
updateObsStatus();
log('App pronta.');
</script>
</body>
</html>

)APPPAGE";

void hStatus() {
    String j = "{";
    j += "\"az\":" + String(curAz/10.0, 1);
    j += ",\"el\":" + String(curEl/10.0, 1);
    j += ",\"parked\":" + String(parked ? "true" : "false");
    j += ",\"elmax\":" + String(elMaxDeciDeg/10.0, 1);
    j += ",\"passActive\":" + String(passActive ? "true" : "false");
    j += "}";
    http.send(200, "application/json", j);
}
void hRoot() { http.send_P(200, "text/html; charset=utf-8", PAGE_HTML); }
void hApp() { http.send_P(200, "text/html; charset=utf-8", APP_HTML); }   // NUOVO: app web satellitare (TLE, calcolo passaggio, invio a /api/passload)
void hGoto() {
    double az = http.hasArg("az") ? http.arg("az").toDouble() : curAz/10.0;
    double el = http.hasArg("el") ? http.arg("el").toDouble() : curEl/10.0;
    lastCmdMs = millis();
    moveTo((int32_t)lround(az*10), (int32_t)lround(el*10));
    http.send(200,"text/plain","OK");
}
void hPark() { lastCmdMs = millis(); parkServos(); http.send(200,"text/plain","OK"); }
void hPing() { lastCmdMs = millis(); http.send(200,"text/plain","OK"); }  // keep-alive dalla pagina web: non muove i servi, resetta solo il timeout di sicurezza
void hSetElMax() {
    // memorizza la posizione ATTUALE del servo elevazione come nuovo limite massimo invalicabile:
    // va chiamato con il boom portato (via slider) fino al punto fisico sicuro, non oltre.
    // Deve restare sopra il centro/orizzonte (135 gradi), altrimenti il park di riferimento
    // (che va sempre al centro) finirebbe bloccato sotto il proprio stesso limite.
    if (curEl <= SERVO_CENTER_DECIDEG) {
        http.send(400, "text/plain", "Troppo basso: deve stare sopra 135 gradi (l'orizzonte)");
        return;
    }
    elMaxDeciDeg = curEl;
    prefs.putInt("elmax", elMaxDeciDeg);
    http.send(200, "text/plain", "OK " + String(elMaxDeciDeg/10.0, 1));
}
void hResetElMax() {
    elMaxDeciDeg = EL_MAX_DECIDEG_DEFAULT;
    prefs.putInt("elmax", elMaxDeciDeg);
    http.send(200, "text/plain", "OK " + String(elMaxDeciDeg/10.0, 1));
}

// ============ NUOVO: carica un passaggio completo via HTTP (equivalente di RCMD_PASS_LOAD) ============
// POST /api/passload
// body (text/plain): "<startDelayMs>,<durationMs>,<count>;<az1>,<el1>;<az2>,<el2>;...;<azN>,<elN>"
//   - startDelayMs/durationMs: stessi campi di RotorPassCmd (vedi RotorProto.h)
//   - az/el: GRADI SERVO (0..270.0), stessa convenzione di RCMD_GOTO - chi genera il
//     passaggio (es. l'app web sul telefono) deve aver gia' fatto calibrazione, centro
//     dell'arco ed eventuale "flip", esattamente come fa oggi il Cardputer prima di un
//     RCMD_PASS_LOAD via ESP-NOW. Il rotore resta "muto" anche con questo endpoint.
// Non richiede "paired": e' un canale di controllo indipendente, come /api/goto manuale.
// Non tocca ESP-NOW ne' handlePacket(): riempie solo le stesse variabili globali che legge
// passTick(), gia' testato e invariato.
void hPassLoad() {
    if (!http.hasArg("plain")) { http.send(400, "text/plain", "body mancante"); return; }
    String body = http.arg("plain");

    int p1 = body.indexOf(';');
    if (p1 < 0) { http.send(400, "text/plain", "formato non valido (manca l'header)"); return; }
    String header = body.substring(0, p1);

    int h1 = header.indexOf(','), h2 = header.indexOf(',', h1 + 1);
    if (h1 < 0 || h2 < 0) { http.send(400, "text/plain", "formato non valido (campi header)"); return; }
    uint32_t newDelay    = (uint32_t) header.substring(0, h1).toInt();
    uint32_t newDuration = (uint32_t) header.substring(h1 + 1, h2).toInt();
    int newCountReq      = header.substring(h2 + 1).toInt();
    uint8_t newCount = (newCountReq > ROTOR_PASS_MAX_PTS) ? ROTOR_PASS_MAX_PTS : (uint8_t)newCountReq;

    RotorPassPoint pts[ROTOR_PASS_MAX_PTS];
    int idx = 0;
    int pos = p1 + 1;
    while (idx < newCount && pos < (int)body.length()) {
        int sep = body.indexOf(';', pos);
        String tok = (sep < 0) ? body.substring(pos) : body.substring(pos, sep);
        int comma = tok.indexOf(',');
        if (comma < 0) break;
        double az = tok.substring(0, comma).toDouble();
        double el = tok.substring(comma + 1).toDouble();
        pts[idx].az = (int16_t) lround(az * 10);
        pts[idx].el = (int16_t) lround(el * 10);
        idx++;
        if (sep < 0) break;
        pos = sep + 1;
    }
    if (idx < 2) { http.send(400, "text/plain", "punti insufficienti (minimo 2)"); return; }

    passCount = (uint8_t) idx;
    memcpy(passPts, pts, passCount * sizeof(RotorPassPoint));
    passStartDelayMs = newDelay;
    passDurationMs   = newDuration;
    passStartMs      = millis();
    passLastTickMs   = 0;
    passActive       = true;
    lastCmdMs        = millis();

    Serial.printf("[ROTORE-PASS-HTTP] caricato: punti=%u delay=%lums durata=%lums\n",
                  passCount, (unsigned long)passStartDelayMs, (unsigned long)passDurationMs);

    http.send(200, "text/plain", "OK " + String(passCount));
}

// ============ Setup / Loop ============
void setup() {
    Serial.begin(115200); delay(200);
    Serial.println("OLR Rotore avvio (versione con /api/passload)");

    pinMode(LED_PIN, OUTPUT);

    prefs.begin("rotorrx", false);
    loadPrefs();

    rotorRxQueue = xQueueCreate(8, sizeof(RxItem));

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSsid, apPass, AP_CHANNEL, 0, 4);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP: "); Serial.print(apSsid); Serial.print("  IP: "); Serial.println(ip);
    Serial.print("MAC: "); Serial.println(WiFi.softAPmacAddress());

    if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init KO"); }
    else {
        esp_now_register_recv_cb(onRecv);
        uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        addPeer(bcast);
        if (paired) addPeer(masterMac);
    }

    parkServos();   // all'avvio: posizione nota, sicura
    parkDueMs = millis() + PARK_SETTLE_MS;   // e' gia' al centro di suo (nessuno scivolamento da fare): spegne comunque il PWM dopo l'assestamento, come sempre

    http.on("/", hRoot);
    http.on("/app", hApp);   // NUOVO: http://192.168.4.1/app serve l'app satellitare direttamente dal rotore
    http.on("/api/status", hStatus);
    http.on("/api/goto", hGoto);
    http.on("/api/park", hPark);
    http.on("/api/ping", hPing);
    http.on("/api/setelmax", hSetElMax);
    http.on("/api/resetelmax", hResetElMax);
    http.on("/api/passload", HTTP_POST, hPassLoad);   // NUOVO: carica un passaggio completo dal telefono
    http.begin();

    Serial.printf("paired=%d\n", paired?1:0);
}

void loop() {
    http.handleClient();

    RxItem it;
    while (rotorRxQueue && xQueueReceive(rotorRxQueue, &it, 0) == pdTRUE) handlePacket(it);

    passTick();   // segue da solo un eventuale passaggio caricato (RCMD_PASS_LOAD o /api/passload)
    servoSlewTick();   // avvicina gradualmente i servi alla posizione richiesta, qualunque sia la fonte

    unsigned long now = millis();

    // spegne il PWM dopo che il park si e' assestato (il servo non deve "scaldare" fermo)
    if (parkDueMs && now > parkDueMs) { detachServos(); parkDueMs = 0; }

    // sicurezza: agganciato ma nessun comando da troppo tempo -> si centra da solo.
    // NON scatta mentre un passaggio autonomo e' caricato: li' il silenzio e' normale e atteso.
    if (paired && !passActive && !parked && lastCmdMs && now - lastCmdMs > ROTOR_TIMEOUT_MS) {
        parkServos();
    }

    unsigned long blinkT = paired ? 1500 : 400;
    if (now - ledMs > blinkT) { ledMs = now; ledState = !ledState; digitalWrite(LED_PIN, ledState); }
}
