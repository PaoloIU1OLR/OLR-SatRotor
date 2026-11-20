// RotorProto.h - protocollo ESP-NOW tra Cardputer (OLR Suite) e la ESP32 del rotore az/el
// Indipendente dal protocollo della Loop (OLR_LoopProto.h): MAC, pairing e canale separati,
// cosi' Loop e Rotore si possono agganciare/sganciare uno senza toccare l'altro.
// Lo stesso identico file va incluso sia nel firmware del Cardputer sia in quello del rotore.
//
// Filosofia: il rotore e' "muto". Tutta l'astronomia (satellite -> az/el), la calibrazione
// (centro azimut scelto per il passaggio) e l'eventuale "flip" (quando l'azimut richiesto
// esce dai 270 gradi del servo) si calcolano sul Cardputer. Il rotore riceve solo angoli
// servo gia' pronti (0..2700 decimi di grado) e li esegue.
//
// RCMD_PASS_LOAD: il Cardputer puo' mandare al rotore l'INTERO passaggio satellitare gia'
// pronto (una serie di punti az/el servo nel tempo) in un unico pacchetto. Da quel momento
// il rotore lo segue da solo, interpolando col proprio clock, senza bisogno di altri comandi
// per tutta la durata del passaggio - cosi' il Cardputer resta libero di usare il WiFi per
// altro (es. il Doppler del 705) senza interrompere il tracciamento.
#pragma once
#include <stdint.h>

#define ROTOR_MAGIC   0xBD     // diverso da LOOP_MAGIC (0xBC): stesso canale ESP-NOW, pacchetti distinguibili
#define ROTOR_PROTO   1

// Comandi (RotorCmd.type)
enum : uint8_t {
    RCMD_PAIR      = 1,   // abbinamento (inviare in broadcast FF:FF:FF:FF:FF:FF)
    RCMD_GOTO      = 2,   // az/el = angolo SERVO in decimi di grado (0..2700), non azimut compass
    RCMD_PARK      = 3,   // vai al centro (1350 = 135.0 gradi), spegni i PWM, e cancella un eventuale passaggio autonomo in corso
    RCMD_GET       = 4,   // chiede solo lo stato
    RCMD_STOP      = 5,   // ferma dove si trova adesso e cancella un eventuale passaggio autonomo in corso
    RCMD_PASS_LOAD = 6    // carica un passaggio satellitare completo (vedi RotorPassCmd): il rotore lo segue da solo
};

// Risposte (RotorStatus.type)
#define RRSP_STATUS   0xA2

// Flag in RotorStatus.flags
#define RFLAG_PAIRED   0x01
#define RFLAG_PARKED   0x02
#define RFLAG_PASS     0x04   // un passaggio autonomo e' attualmente caricato/in corso

struct __attribute__((packed)) RotorCmd {
    uint8_t  magic;      // ROTOR_MAGIC
    uint8_t  type;       // RCMD_*
    uint8_t  proto;      // ROTOR_PROTO
    uint8_t  _pad;
    int16_t  az;         // angolo servo AZ, decimi di grado (0..2700)
    int16_t  el;         // angolo servo EL, decimi di grado (0..2700)
    uint32_t seq;
};  // 12 byte

struct __attribute__((packed)) RotorStatus {
    uint8_t  magic;      // ROTOR_MAGIC
    uint8_t  type;       // RRSP_STATUS
    uint8_t  proto;
    uint8_t  flags;      // RFLAG_*
    int16_t  az;         // ultimo angolo servo AZ comandato
    int16_t  el;         // ultimo angolo servo EL comandato
    uint16_t fwVersion;  // es. 100 = v1.00
    uint32_t lastSeq;    // ultimo seq eseguito (conferma comando)
};  // 16 byte

// ---- Passaggio autonomo (RCMD_PASS_LOAD) ----
// ROTOR_PASS_MAX_PTS deve combaciare col numero di punti calcolati dal Cardputer (TRK_PTS).
// Dimensione pacchetto risultante: 16 + 48*4 = 208 byte, entro il limite ESP-NOW di 250 byte.
constexpr uint8_t ROTOR_PASS_MAX_PTS = 48;

struct __attribute__((packed)) RotorPassPoint {
    int16_t az;   // angolo servo AZ, decimi di grado (0..2700)
    int16_t el;   // angolo servo EL, decimi di grado (0..2700)
};  // 4 byte

struct __attribute__((packed)) RotorPassCmd {
    uint8_t  magic;          // ROTOR_MAGIC
    uint8_t  type;           // RCMD_PASS_LOAD
    uint8_t  proto;          // ROTOR_PROTO
    uint8_t  count;          // numero di punti validi in pts[] (<= ROTOR_PASS_MAX_PTS)
    uint32_t seq;
    uint32_t startDelayMs;   // quanto manca all'AOS, in millisecondi, da quando il rotore riceve il pacchetto
    uint32_t durationMs;     // durata totale del passaggio (AOS -> LOS), in millisecondi
    RotorPassPoint pts[ROTOR_PASS_MAX_PTS];
};  // 16 + 192 = 208 byte
