# ProjectSETR2 — ESP32 Remote Sensors (Capteurs distants)

Carte ESP32 déportée équipée d'un capteur CO₂ MH-Z19B et d'un détecteur de présence ultrason HC-SR04. Les mesures sont transmises à la carte centrale ProjectSETR via liaison RS232 (UART2).

## Architecture

```
┌────────────────────────────────────────────────────┐
│              ProjectSETR2 (ESP32 distant)           │
│                                                     │
│  ┌──────────────────┐                               │
│  │  MH-Z19B CO₂     │                               │
│  │  ┌──────────┐    │  ┌───────────────┐            │
│  │  │ UART1    │────┼──▶ CO2 Producer  │            │
│  │  │ (GPIO17) │    │  └───────┬───────┘            │
│  │  ├──────────┤    │          │                    │
│  │  │ PWM      │────┼──▶ CO2 Producer              │
│  │  │ (GPIO23) │    │  └───────┬───────┘            │
│  │  └──────────┘    │          │                    │
│  └──────────────────┘          │                    │
│                        ┌───────▼───────┐            │
│  ┌──────────────────┐  │  FreeRTOS     │  ┌────────┴────┐
│  │  HC-SR04         │  │  Queue        │  │ UART2 TX   │
│  │  Ultrason        │──┼───────────────┼──▶ (GPIO27)  │
│  │  TRIG=GPIO33     │  │  Consumer    │  │ à la carte │
│  │  ECHO=GPIO32     │  │  UART Task   │  │ centrale   │
│  └──────────────────┘  └───────────────┘  └────────────┘
```
## Matériel

| Composant | Connexion |
|---|---|
| ESP32 DevKit V4 | Carte distante |
| MH-Z19B CO₂ capteur | UART1 (TX=GPIO16, RX=GPIO17, 9600 bauds) + PWM (GPIO23) |
| HC-SR04 Ultrason | TRIG=GPIO33, ECHO=GPIO32 |
| RS232 vers ProjectSETR | UART2 (TX=GPIO27, RX=GPIO26, 9600 bauds) |

## Tâches FreeRTOS

| Tâche | Rôle | Pile | Priorité |
|---|---|---|---|
| `co2_producer` | Lit CO₂ (UART+PWM) toutes les 4s, envoie dans la queue | 3072 | 5 |
| `presence_producer` | Lit ultrason toutes les 500ms, envoie dans la queue | 4096 | 5 |
| `uart_consumer` | Consomme la queue, envoie trame sur UART2 toutes les 4s | 4096 | 6 |

## Protocole de communication

Trame envoyée à ProjectSETR (format ASCII) :

```
CO2_UART:<ppm>;CO2_PWM:<ppm>;PRES:<0|1>\n
```

| Champ | Source | Description |
|---|---|---|
| `CO2_UART` | MH-Z19B (lecture UART) | Concentration CO₂ en ppm |
| `CO2_PWM` | MH-Z19B (lecture PWM) | Concentration CO₂ en ppm (voie redondante) |
| `PRES` | HC-SR04 | 1 si distance ≤ 50 cm, 0 sinon |

## Fonctionnalités du capteur CO₂

- Calibration zero au boot (air frais ~400 ppm)
- ABC (Automatic Baseline Correction) désactivé par défaut
- Délai de chauffe de 3 minutes avant mesures stables
- Plage de détection : configurable (5000 ppm par défaut)

## Construction et déploiement

```bash
pio run -t upload --upload-port /dev/cu.usbserial-110
pio device monitor --port /dev/cu.usbserial-110
```

> **Note** : Framework ESP-IDF. Vérifier le port série dans `platformio.ini`.
