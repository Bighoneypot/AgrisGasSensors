
# Procedura Reset Device LoRaWAN su AWS IoT Core
## Progetto MYRUMINET - Sensore Gas Stalla

---

### Il Problema

Ogni volta che si carica un nuovo firmware sul RAK3172-E tramite Arduino IDE, il bootloader RUI3 resetta il **DevNonce** (contatore incrementale usato nel Join OTAA LoRaWAN) nella memoria NVM dell'STM32.

AWS IoT Core per LoRaWAN rifiuta silenziosamente qualsiasi Join Request con un DevNonce già visto (protezione anti-replay). Il risultato è che dopo ogni flash il dispositivo non riesce più a fare il Join e non invia dati.

La soluzione è cancellare e ricreare il device su AWS IoT Core prima di ogni flash, così il contatore DevNonce viene resettato lato server.

---

### Soluzione Implementata: Pre-Upload Hook Automatico

Abbiamo automatizzato il reset integrando uno script `.bat` direttamente nel processo di upload di Arduino IDE, così il reset avviene **automaticamente** ad ogni flash senza intervento manuale.

---

### File Coinvolti

**1. Script di reset:**
```
C:\agrisResetDevice\reset_lorawan_device_gas_sensor.bat
```

Questo script:
- Cerca il device `Agris_Lifely_GAS_Sensors_01` su AWS IoT Core (regione `eu-central-1`)
- Lo cancella (resettando il DevNonce lato server)
- Attende 3 secondi per la propagazione su AWS
- Lo ricrea con le stesse credenziali OTAA (DevEUI, AppEUI, AppKey, DeviceProfileId, ServiceProfileId)
- Aggiorna la tabella DynamoDB `ruminet-Devices` con il nuovo Wireless Device ID

**2. Hook Arduino IDE:**
```
C:\Users\gabri\AppData\Local\Arduino15\packages\rak_rui\hardware\stm32\4.2.4\platform.local.txt
```

Contenuto:
```
recipe.hooks.upload.preoptions.1.pattern=cmd /c C:\agrisResetDevice\reset_lorawan_device_gas_sensor.bat
```

Questo file dice ad Arduino IDE di eseguire lo script `.bat` automaticamente **prima** di ogni upload del firmware.

---

### Credenziali e Parametri AWS Utilizzati

| Parametro | Valore |
|-----------|--------|
| Device Name | Agris_Lifely_GAS_Sensors_01 |
| DevEUI | ac1f09fffe0a70f4 |
| AppEUI | ac1f09fff8683172 |
| AppKey | AC1F09FFFE0A70F4AC1F09FFF8683172 |
| DeviceProfileId | 65f2e49c-6716-4f4f-866a-2b6e1edf9342 |
| ServiceProfileId | be7b355e-4d08-4d9d-b831-76624be192d7 |
| Destination | Ruminet-Uplink-Destination |
| Regione AWS | eu-central-1 |
| Tabella DynamoDB | ruminet-Devices |

---

### Flusso Operativo

1. Premi **Upload** in Arduino IDE
2. Arduino IDE esegue automaticamente `reset_lorawan_device_gas_sensor.bat`
3. Lo script cancella il device su AWS IoT Core
4. Lo script ricrea il device con DevNonce pulito
5. Lo script aggiorna DynamoDB con il nuovo Device ID
6. Arduino IDE flasha il firmware sul RAK3172-E
7. Il dispositivo si accende → Join OTAA → funziona al primo colpo

---

### Note Importanti

- Il percorso dello script `.bat` **non deve contenere spazi né caratteri speciali** (come `&`). Per questo è stato scelto `C:\agrisResetDevice\`
- Se si aggiorna la versione della board RAK RUI3 (es. da 4.2.4 a una nuova versione), il file `platform.local.txt` va **ricopiato** nella nuova cartella della versione
- Lo script richiede che AWS CLI sia configurato con credenziali valide (`aws configure`)
- Il prerequisito è avere Python3 installato (usato per il parsing JSON in alcuni comandi)
