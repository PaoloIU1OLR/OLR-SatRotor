# OLR-SatRotor

Rotore az/el satellitare portatile per antenna Yagi, pilotabile **dal solo cellulare** via un'app web — nessun dispositivo aggiuntivo richiesto (niente Cardputer, niente PC sul campo). Pensato per attività radioamatoriali in portatile (POTA, SOTA, QRP all'aperto) con satelliti radioamatoriali.

Autore: **IU1OLR**

## Cos'è

Una ESP32 pilota due servo digitali (azimut ed elevazione) per orientare una piccola antenna Yagi satellitare montata su treppiede. L'ESP32 fa da Access Point WiFi e serve direttamente dal suo firmware una pagina web (`/app`): da lì, dal telefono, si scaricano i TLE, si calcola il prossimo passaggio di un satellite, ci si allinea e si invia il passaggio al rotore, che poi lo segue da solo.

Non serve installare nulla sul telefono: basta collegarsi al WiFi del rotore e aprire una pagina nel browser.

## Di cosa hai bisogno (BOM)

- 1× ESP32 (va bene un generico "ESP32 Dev Module", es. WROOM 38 pin)
- 2× servo digitali **DS3230 Pro 270°** (o equivalenti, purché a rotazione 270° e di coppia adeguata al peso dell'antenna) — uno per l'azimut, uno per l'elevazione
- Un treppiede fotografico con attacco standard 1/4"-20 come base
- Le parti stampate in 3D (vedi sotto, cartella [`3d/`](3d/))
- Viti/dadi per fissare i servo nelle tasche stampate (misure nei file 3D)
- Un'antenna Yagi satellitare da montare sul rotore (il progetto non dipende da un modello specifico)
- Alimentazione: ESP32 + servo via power bank USB (o batteria adeguata alla coppia richiesta dai servo sotto carico)

## File 3D

Nella cartella [`3d/`](3d/):

- `rotore_satellite_bil_e_slim_90.stl` — ultima versione (progettata in Fusion 360), pronta per la stampa
- `rotore_boom60.scad`, `rotore_boom90.scad` — varianti precedenti in OpenSCAD (per boom antenna rispettivamente ~60mm e ~90mm), mantenute come riferimento
- `rotore_v4.3mf` — una versione precedente già pronta per lo slicer (formato 3MF)

Il pezzo inferiore si aggancia al treppiede e fa la rotazione orizzontale (azimut); quello superiore, fissato sul corno del servo inferiore, regola l'inclinazione (elevazione). È presente un fine corsa meccanico sulla forcella dell'elevazione, oltre al limite software configurabile via web (vedi sotto), per evitare che il boom dell'antenna vada a sbattere contro la struttura vicino allo zenit.

## Firmware (cartella `firmware/`)

- `ESPNOW_Rotore_HTTP.ino` — sketch principale
- `RotorProto.h` — definizioni condivise (struttura dei comandi)

**Come compilare:**

1. Arduino IDE, con il supporto schede **ESP32** installato (Boards Manager → "esp32" di Espressif)
2. Scheda: **ESP32 Dev Module**
3. Libreria da installare (Library Manager): **[ESP32Servo](https://github.com/madhephaestus/ESP32Servo)** di Kevin Harrington / madhephaestus
4. Apri `ESPNOW_Rotore_HTTP.ino`, compila e carica via USB

**Collegamento servo** (pin di default, modificabili in testa allo sketch):

| Servo | Pin ESP32 |
|---|---|
| Azimut | GPIO 32 |
| Elevazione | GPIO 33 |

Alimenta i servo con una linea dedicata adeguata (non dal solo 5V della USB dell'ESP32 se i servo sotto carico richiedono più corrente di quella che la porta USB può fornire).

## Primo avvio e uso

1. Accendi il rotore: crea una rete WiFi propria, SSID **`OLR_Rotor`**, password **`rotoresat`** (modificabili nel codice)
2. Collega il telefono a quella rete
3. Apri nel browser **`http://192.168.4.1/app`**
4. **Imposta il limite di sicurezza dell'elevazione** (la prima volta, e ogni volta che cambi meccanica): dalla pagina base `http://192.168.4.1/` (senza `/app`), porta con lo slider il boom fino al punto fisico massimo sicuro e salva quel limite — resta memorizzato e viene rispettato automaticamente da qualunque comando, incluso `/app`
5. In `/app`: scarica i TLE da Celestrak (serve internet in quel momento, non quando usi il rotore sul campo) oppure carica un file `tle.txt` già pronto, inserisci la tua posizione (locatore Maidenhead), scegli il satellite
6. Calcola il prossimo passaggio (o usa il bottone **DEMO** per un passaggio fittizio di test, senza aspettare un satellite vero)
7. Allineamento: il rotore va al centro dell'arco calcolato, ruota fisicamente il treppiede nella direzione indicata e conferma
8. Invia il passaggio al rotore: da quel momento lo segue da solo, senza bisogno del telefono acceso o collegato

## Sicurezza

- Il limite massimo di elevazione (punto 4 sopra) è una protezione software contro urti meccanici: **va sempre impostato per primo**, prima di montare l'antenna, e riverificato ogni volta che si modifica la meccanica.
- Il rotore si muove gradualmente (non a scatti) per non stressare la meccanica col peso dell'antenna.
- Prova sempre un movimento completo con il bottone **DEMO** prima di montare l'antenna su una meccanica nuova o modificata.

## Licenza

CC BY-NC-SA 4.0 — vedi [`LICENSE`](LICENSE). Uso e modifica liberi, non commerciale, stessa licenza per i derivati. Se modifichi o migliori il progetto, una pull request o un'issue su questo repository sono il modo più apprezzato per farlo sapere all'autore.

## Crediti

- Calcolo orbitale (SGP4) nell'app web: libreria **[satellite.js](https://github.com/shashwatak/satellite-js)**
- Firmware ESP32: libreria **[ESP32Servo](https://github.com/madhephaestus/ESP32Servo)** di Kevin Harrington / madhephaestus
- Fonte TLE: **[CelesTrak](https://celestrak.org)**

## Progetto collegato

Questo rotore è stato sviluppato come estensione di **[SuiteQRPCardputer](https://github.com/PaoloIU1OLR/SuiteQRPCardputer)**, il firmware per M5Stack Cardputer ADV dell'autore (log QSO, CW Trainer, tracking satelliti con Icom IC-705, Loop Magnetica). Il rotore funziona anche da solo col Cardputer (via ESP-NOW) — questo repository documenta la via standalone via cellulare per chi non possiede un Cardputer.
