
# Controller di Posizione 6DoF per UAV

## Indice

1. [Introduzione](#introduzione)
2. [Panoramica del Sistema](#panoramica-del-sistema)
3. [Struttura del Controller](#struttura-del-controller)
4. [Parametri di Controllo Adattativo](#parametri-di-controllo-adattativo)
    - [Guadagni Iniziali](#guadagni-iniziali)
    - [Tassi di Adattamento](#tassi-di-adattamento)
    - [Limiti dei Guadagni](#limiti-dei-guadagni)
5. [Soglie di Errore](#soglie-di-errore)
6. [Calcolo dell'Intervallo di Tempo (`dt`)](#calcolo-dellintervallo-di-tempo-dt)
    - [Cos'è `dt`?](#cosè-dt)
    - [Calcolo di `dt` nel Codice](#calcolo-di-dt-nel-codice)
    - [Importanza di `dt`](#importanza-di-dt)
7. [Calcolo degli Errori](#calcolo-degli-errori)
    - [Errori di Posizione](#errori-di-posizione)
    - [Errori di Velocità](#errori-di-velocità)
8. [Adattamento dei Guadagni](#adattamento-dei-guadagni)
    - [Adattamento di `ZP_GAIN`](#adattamento-di-zp_gain)
    - [Adattamento di `VP_GAIN`](#adattamento-di-vp_gain)
9. [Calcolo della Spinta](#calcolo-della-spinta)
10. [Gestione della Stabilità](#gestione-della-stabilità)
    - [Verifica delle Soglie di Errore](#verifica-delle-soglie-di-errore)
    - [Aggiornamento dei Setpoint](#aggiornamento-dei-setpoint)
11. [Logging e Debug](#logging-e-debug)
12. [Pubblicazione del Setpoint di Spinta](#pubblicazione-del-setpoint-di-spinta)
13. [Constrain della Spinta](#constrain-della-spinta)
14. [Considerazioni sulla Sintonizzazione](#considerazioni-sulla-sintonizzazione)
15. [Conclusioni](#conclusioni)
16. [Licenza](#licenza)
17. [Contatti](#contatti)
18. [Riferimenti](#riferimenti)

---

## Introduzione

Questo documento fornisce una spiegazione dettagliata del controller di posizione a 6 gradi di libertà (`pose_controller_6dof`) utilizzato per il controllo di un veicolo aereo non abitato (UAV). Il controller implementa una strategia di controllo adattativo che regola dinamicamente i guadagni di posizione e velocità in base agli errori correnti, garantendo così la stabilità e l'efficacia nel raggiungimento degli obiettivi di navigazione.

## Panoramica del Sistema

Il controller è progettato per gestire le posizioni e le velocità lungo gli assi X, Y e Z del UAV. Utilizza guadagni adattativi (`ZP_GAIN` e `VP_GAIN`) che si adattano in tempo reale in risposta agli errori di posizione e velocità, consentendo al drone di mantenere la stabilità anche in presenza di perturbazioni o variazioni ambientali.

## Struttura del Controller

La funzione principale del controller è `UUVPOSControl::pose_controller_6dof()`, che esegue le seguenti operazioni:

1. **Inizializzazione dei Parametri di Controllo Adattativo**: Definisce i guadagni iniziali, i tassi di adattamento e i limiti dei guadagni.
2. **Calcolo dell'Intervallo di Tempo (`dt`)**: Determina il tempo trascorso dall'ultima iterazione del controller.
3. **Calcolo degli Errori di Posizione e Velocità**: Calcola gli errori correnti rispetto ai setpoint desiderati.
4. **Adattamento dei Guadagni**: Aggiorna i guadagni in base agli errori calcolati e all'intervallo di tempo.
5. **Calcolo della Spinta**: Determina la spinta necessaria lungo ciascun asse utilizzando i guadagni adattativi.
6. **Gestione della Stabilità**: Verifica se il drone è stabile e aggiorna i setpoint di conseguenza.
7. **Logging e Pubblicazione del Setpoint di Spinta**: Registra i dati per il debug e pubblica i setpoint per l'attuazione.

## Parametri di Controllo Adattativo

### Guadagni Iniziali

I guadagni iniziali determinano la risposta iniziale del controller prima che avvenga l'adattamento.

- **ZP_GAIN**: Guadagno per il controllo della posizione lungo l'asse Z.
  ```cpp
  static float ZP_GAIN = 0.1f;
  ```
  
- **VP_GAIN**: Guadagno per il controllo delle velocità lungo gli assi X e Y.
  ```cpp
  static float VP_GAIN = 0.5f;
  ```

### Tassi di Adattamento

I tassi di adattamento (`gamma_z` e `gamma_v`) controllano la velocità con cui i guadagni si adattano in risposta agli errori.

- **gamma_z**: Tasso di adattamento per `ZP_GAIN`.
  ```cpp
  const float gamma_z = 0.01f;
  ```
  
- **gamma_v**: Tasso di adattamento per `VP_GAIN`.
  ```cpp
  const float gamma_v = 0.01f;
  ```

### Limiti dei Guadagni

Per evitare oscillazioni e garantire la stabilità, i guadagni sono limitati entro un intervallo definito.

- **ZP_GAIN**
  ```cpp
  const float ZP_GAIN_MIN = 0.05f;
  const float ZP_GAIN_MAX = 1.0f;
  ```
  
- **VP_GAIN**
  ```cpp
  const float VP_GAIN_MIN = 0.1f;
  const float VP_GAIN_MAX = 2.0f;
  ```

## Soglie di Errore

Le soglie di errore determinano quando il drone è considerato stabile.

- **Errore di Posizione**
  ```cpp
  const float POSITION_ERROR_THRESHOLD = 0.5f; // in metri
  ```
  
- **Errore di Velocità**
  ```cpp
  const float VELOCITY_ERROR_THRESHOLD = 0.5f; // in m/s
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
