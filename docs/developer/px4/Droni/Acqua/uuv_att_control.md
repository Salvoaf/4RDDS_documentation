
# Controller di Attitudine Geo per UAV


## Introduzione

Questo documento fornisce una spiegazione dettagliata del controller di attitudine `control_attitude_geo` utilizzato per controllare l'attitudine di un veicolo aereo non abitato (UAV). Questo controller utilizza una strategia di controllo adattativo per regolare i guadagni proporzionali e derivativi (`P_GAIN` e `D_GAIN`) in risposta agli errori di attitudine, migliorando così la stabilità e la reattività del sistema.

## Struttura del Controller

La funzione principale del controller è `UUVAttitudeControl::control_attitude_geo()`, che esegue le seguenti operazioni:

1. **Inizializzazione dei Parametri di Controllo Adattativo**: Definisce i guadagni iniziali, i tassi di adattamento e i limiti dei guadagni.
2. **Calcolo dell'Intervallo di Tempo (`dt`)**: Determina il tempo trascorso dall'ultima esecuzione del controller.
3. **Calcolo dell'Errore di Attitudine**: Calcola l'errore di attitudine tra l'attitudine desiderata e quella corrente.
4. **Adattamento dei Guadagni**: Aggiorna i guadagni in base alla magnitudine dell'errore di attitudine.
5. **Calcolo dei Torques**: Determina i torques da applicare per correggere l'attitudine in base ai guadagni adattativi.
6. **Applicazione dei Comandi di Controllo**: Applica i comandi di controllo per eseguire l'attitudine desiderata.

## Parametri di Controllo Adattativo

### Guadagni Iniziali

I guadagni iniziali determinano la risposta iniziale del controller prima che avvenga l'adattamento.

- **P_GAIN**: Guadagno proporzionale iniziale.
  ```cpp
  static float P_GAIN = 0.2f;
  ```
  
- **D_GAIN**: Guadagno derivativo iniziale.
  ```cpp
  static float D_GAIN = 5.0f;
  ```

### Tassi di Adattamento

I tassi di adattamento (`gamma_p` e `gamma_d`) controllano la velocità con cui i guadagni si adattano in risposta agli errori.

- **gamma_p**: Tasso di adattamento per `P_GAIN`.
  ```cpp
  const float gamma_p = 0.01f;
  ```
  
- **gamma_d**: Tasso di adattamento per `D_GAIN`.
  ```cpp
  const float gamma_d = 0.01f;
  ```

### Limiti dei Guadagni

Per evitare oscillazioni e garantire la stabilità, i guadagni sono limitati entro un intervallo definito.

- **P_GAIN**
  ```cpp
  const float P_GAIN_MIN = 0.1f;
  const float P_GAIN_MAX = 1.0f;
  ```
  
- **D_GAIN**
  ```cpp
  const float D_GAIN_MIN = 1.0f;
  const float D_GAIN_MAX = 10.0f;
  ```

## Calcolo dell'Intervallo di Tempo (`dt`)

### Cos'è `dt`?

`dt` rappresenta l'intervallo di tempo (in secondi) trascorso tra due esecuzioni consecutive del controller. È fondamentale per garantire che gli aggiornamenti dei guadagni siano proporzionati al tempo effettivo, mantenendo la coerenza del comportamento del controller indipendentemente dalla frequenza di esecuzione.

### Calcolo di `dt` nel Codice

```cpp
static hrt_abstime last_time = 0;
hrt_abstime now = hrt_absolute_time();
float dt = (now - last_time) / 1e6f; // Convertito in secondi
last_time = now;

// Verifica dell'intervallo di tempo
if (dt <= 0.0f || dt > 1.0f) {
    dt = 0.01f; // Valore di default in caso di intervallo non valido
}
```

## Calcolo dell'Errore di Attitudine

L'errore di attitudine rappresenta la differenza tra l'attitudine desiderata e quella attuale del drone. Viene calcolato utilizzando le matrici di rotazione corrispondenti.

```cpp
Matrix3f e_R = (rot_des.transpose() * rot_att - rot_att.transpose() * rot_des) * 0.5f;
```

### Conversione in Vettore tramite Vee-Map

L'errore di attitudine `e_R` viene convertito in un vettore `e_R_vec` utilizzando la Vee-map:

```cpp
e_R_vec(0) = e_R(2, 1);  // Roll
e_R_vec(1) = e_R(0, 2);  // Pitch
e_R_vec(2) = e_R(1, 0);  // Yaw
```

Questo vettore contiene gli errori di rollio, beccheggio e imbardata.

## Adattamento dei Guadagni

Il controller utilizza una strategia di controllo adattativo per aggiornare i guadagni in base all'errore di attitudine.

### Formula per l'Aggiornamento dei Guadagni

L'aggiornamento dei guadagni `P_GAIN` e `D_GAIN` avviene in base alla seguente formula:

- **Aggiornamento di `P_GAIN`:**
  ```cpp
  P_GAIN += gamma_p * attitude_error_magnitude * dt;
  ```

- **Aggiornamento di `D_GAIN`:**
  ```cpp
  D_GAIN += gamma_d * attitude_error_magnitude * dt;
  ```

Dove:
- `gamma_p` e `gamma_d` sono i tassi di adattamento.
- `attitude_error_magnitude` rappresenta la magnitudine dell'errore di attitudine.
- `dt` è l'intervallo di tempo tra le iterazioni.

Entrambi i guadagni sono limitati entro i valori definiti per evitare eccessive oscillazioni.


## Calcolo dei Torques

Una volta aggiornati i guadagni, il controller calcola i torques necessari per correggere l'attitudine utilizzando un controllo proporzionale-derivativo (PD).

- **Controllo Proporzionale (P)**: Basato sull'errore di attitudine `e_R_vec`.
  ```cpp
  torques = -P_GAIN * e_R_vec;
  ```

- **Controllo Derivativo (D)**: Basato sulla velocità angolare attuale `omega`, corretta per la velocità angolare desiderata.
  ```cpp
  torques -= D_GAIN * omega;
  ```

### Estrarre i Comandi di Controllo

I comandi di controllo per rollio, beccheggio e imbardata vengono estratti dai torques calcolati:

```cpp
float roll_u = torques(0);
float pitch_u = torques(1);
float yaw_u = torques(2);
```

Questi valori rappresentano i comandi di attitudine che devono essere applicati per correggere l'orientamento del drone.

## Applicazione dei Comandi di Controllo

Prima di applicare i comandi di controllo, il controller verifica che i valori siano entro i limiti operativi. Vengono quindi applicati i comandi di attitudine per stabilizzare il drone.

```cpp
constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_x, thrust_y, thrust_z);
```

Questo assicura che i comandi non superino i valori massimi impostati per ogni asse.

## Conclusioni

Il controller di attitudine `control_attitude_geo` utilizza guadagni adattativi proporzionali e derivativi per correggere l'orientamento del drone in risposta agli errori di attitudine e di velocità angolare. La strategia di controllo PD, combinata con i tassi di adattamento per i guadagni, garantisce una risposta stabile e reattiva, mantenendo l'attitudine desiderata del drone.

## Contatti

Per ulteriori domande o suggerimenti, si prega di contattare [tuo.email@esempio.com](mailto:tuo.email@esempio.com).
