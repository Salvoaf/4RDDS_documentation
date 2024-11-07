# Guida Completa alla Creazione di un Nuovo Oggetto in Gazebo: Comprendere i Fondamenti

Questa guida fornisce una panoramica dettagliata su come creare e inserire un nuovo oggetto (target) in Gazebo, esplorando non solo i passaggi pratici ma anche comprendendo i file e le strutture sottostanti. Inoltre, vedremo come gestire un target con proprietà gassose e come utilizzare immagini per creare materiali e texture personalizzati.

## Introduzione

Gazebo è un potente simulatore che permette di creare ambienti 3D complessi per testare robot, sensori e oggetti in un ambiente realistico. Per sfruttare al meglio Gazebo, è fondamentale comprendere come creare e personalizzare i modelli che popolano le simulazioni.

Questa guida vi accompagnerà attraverso:

- La creazione di un nuovo modello in Gazebo
- La comprensione dei file chiave: `.sdf` e `.dae`
- L'utilizzo di immagini per materiali e texture
- L'implementazione di un target gassoso
- Esempi pratici e consigli utili

## 1. Preparazione della Directory del Modello

Per iniziare, crea una cartella per il tuo nuovo modello nella directory dei modelli di Gazebo. La directory consigliata è:

```bash
/home/fourdds/.gazebo/models
```

**Esempio:**

```bash
mkdir -p /home/fourdds/.gazebo/models/nuovo_target
```

## 2. Creazione del File `model.config`

All'interno della cartella del modello (`nuovo_target`), crea un file chiamato `model.config`. Questo file contiene informazioni basilari sul modello e permette a Gazebo di riconoscerlo correttamente.

**Contenuto di `model.config`:**

```xml
<?xml version="1.0"?>
<model>
  <name>Nuovo Target</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Il Tuo Nome</name>
    <email>tuaemail@example.com</email>
  </author>
  <description>
    Un nuovo target per la simulazione in Gazebo.
  </description>
</model>
```

## 3. Creazione del File `model.sdf`

Il file `model.sdf` (Simulation Description Format) definisce le caratteristiche dettagliate del modello. Qui puoi specificare la geometria, i materiali e le proprietà fisiche.

### 3.1 Cos'è un File `.sdf`?

Il file `.sdf` è un file XML che descrive un modello o un mondo in Gazebo. Definisce la struttura del modello, le proprietà fisiche, i collegamenti (link), le giunzioni (joint), i sensori e altre caratteristiche. Serve come schema per costruire l'oggetto nella simulazione, specificando come deve comportarsi e apparire.

### 3.2 Esempio di `model.sdf` per un Cubo

**Contenuto di `model.sdf`:**

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="nuovo_target">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient> <!-- Colore rosso -->
          <diffuse>1.0 0.0 0.0 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

Questo esempio crea un cubo di dimensioni 1x1x1 metri con un colore rosso.

## 4. Aggiunta di un Target Gassoso

Per creare un target gassoso, possiamo utilizzare una geometria semitrasparente e regolare le proprietà fisiche per simulare un comportamento gassoso.

### 4.1 Modifiche al File `model.sdf` per un Target Gassoso

**Contenuto aggiornato di `model.sdf`:**

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="target_gassoso">
    <static>false</static> <!-- Il target può muoversi -->
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <collide_bitmask>0x00</collide_bitmask> <!-- Evita collisioni -->
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.0 0.5 1.0 0.3</ambient> <!-- Colore azzurro semitrasparente -->
          <diffuse>0.0 0.5 1.0 0.3</diffuse>
          <transparency>0.7</transparency> <!-- Aumenta la trasparenza -->
        </material>
      </visual>
      <inertial>
        <mass>0.1</mass> <!-- Massa ridotta -->
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

**Spiegazione delle Modifiche:**

- **Geometria:** Abbiamo usato una sfera con raggio 0.5 metri.
- **Materiale:** Colori `ambient` e `diffuse` impostati con un valore alpha di `0.3` per la semitrasparenza.
- **Trasparenza:** Il tag `<transparency>` controlla il livello di trasparenza (0.0 opaco, 1.0 completamente trasparente).
- **Proprietà Fisiche:**
  - **`<static>false</static>`:** Permette al modello di muoversi.
  - **Massa e Inerzia:** Valori ridotti per simulare leggerezza.
  - **Collide Bitmask:** Impostato per evitare collisioni con altri oggetti.

### 4.2 Esempio Completo di `model.sdf` per un Target Gassoso

Ecco un esempio completo per un target gassoso che utilizza una combinazione di semitrasparenza e proprietà fisiche leggere:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="nuvola_gassosa">
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>1.0</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <collide_bitmask>0x00</collide_bitmask>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>1.0</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 0.1</ambient>
          <diffuse>0.8 0.8 0.8 0.1</diffuse>
          <transparency>0.9</transparency>
        </material>
      </visual>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

**Caratteristiche:**

- **Aspetto Gassoso:** Elevata trasparenza e colori chiari per simulare una nube.
- **Proprietà Fisiche:** Massa e inerzia molto basse.
- **Assenza di Collisioni:** Il bitmask evita interazioni indesiderate con altri oggetti.

## 5. Utilizzo di Mesh Personalizzate

Se desideri utilizzare una mesh personalizzata (ad esempio, un file `.dae` o `.stl`), posiziona il file nella sottocartella `meshes` all'interno della cartella del modello.

### 5.1 Cos'è un File `.dae`?

Il file `.dae` (Digital Asset Exchange) è un formato per lo scambio di asset digitali, basato sullo standard COLLADA (COLLAborative Design Activity). Contiene dati dettagliati della mesh 3D, inclusi vertici, facce, normali, UV mapping e materiali. Fornisce la geometria dettagliata per la rappresentazione visiva del modello.

### 5.2 Relazione tra File `.sdf` e `.dae`

- Il file `.sdf` fa riferimento al file `.dae` per ottenere la geometria visiva e di collisione del modello.
- Nel `.sdf`, utilizziamo il tag `<mesh>` con il percorso al file `.dae` per includere la mesh.

**Esempio di riferimento a un file `.dae` nel `.sdf`:**

```xml
<geometry>
  <mesh>
    <uri>model://nome_modello/meshes/nome_file.dae</uri>
  </mesh>
</geometry>
```

### 5.3 Struttura della Cartella

Assicurati che la struttura della cartella del modello sia la seguente:

```
/home/fourdds/.gazebo/models/nuovo_target/
├── model.config
├── model.sdf
└── meshes/
    └── tuo_modello.dae
```

### 5.4 Modifiche al File `model.sdf` con Mesh Personalizzata

**Esempio di `model.sdf` con mesh:**

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="nuovo_target_con_mesh">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://nuovo_target/meshes/tuo_modello.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nuovo_target/meshes/tuo_modello.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

**Nota:** Assicurati che il percorso nel tag `<uri>` punti correttamente al file della mesh.

## 6. Uso delle Immagini per Materiali e Texture

Le immagini (ad esempio, `.jpg`, `.png`) sono utilizzate come texture per dare ai modelli un aspetto più realistico o specifico.

### 6.1 Cartella `materials/textures`

- **Posizione:** All'interno della cartella del modello:

```
/home/fourdds/.gazebo/models/nome_modello/materials/textures
```

- **Contenuto:** Contiene le immagini utilizzate come texture.

### 6.2 Definizione dei Materiali nei File `.dae`

- Il file `.dae` può includere riferimenti a materiali e texture.
- Quando crei o modifichi un file `.dae` in un software di modellazione 3D (come Blender), puoi assegnare materiali e texture ai tuoi modelli.
- Questi materiali possono utilizzare le immagini nella cartella `materials/textures` come texture diffuse, speculari, normali, ecc.

### 6.3 Esempio Pratico: Applicare una Texture Personalizzata

Supponiamo di avere un modello di una cassa di legno e vogliamo applicare una texture di legno.

#### Passi:

1. **Preparazione delle Immagini:**

   - Salva l'immagine della texture di legno come `legno.jpg` nella cartella:

     ```
     /home/fourdds/.gazebo/models/cassa_legno/materials/textures/legno.jpg
     ```

2. **Creazione della Mesh con Materiali in Blender:**

   - Modella la cassa in Blender.
   - Applica un materiale al modello.
   - Assegna la texture `legno.jpg` al materiale.
   - Esporta il modello in formato `.dae`, assicurandoti che le opzioni per includere materiali e texture siano selezionate.

3. **Struttura della Cartella del Modello:**

   ```
   /home/fourdds/.gazebo/models/cassa_legno/
   ├── model.config
   ├── model.sdf
   ├── meshes/
   │   └── cassa_legno.dae
   └── materials/
       └── textures/
           └── legno.jpg
   ```

4. **Contenuto di `model.sdf`:**

   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <model name="cassa_legno">
       <static>true</static>
       <link name="link">
         <collision name="collision">
           <geometry>
             <mesh>
               <uri>model://cassa_legno/meshes/cassa_legno.dae</uri>
             </mesh>
           </geometry>
         </collision>
         <visual name="visual">
           <geometry>
             <mesh>
               <uri>model://cassa_legno/meshes/cassa_legno.dae</uri>
             </mesh>
           </geometry>
         </visual>
       </link>
     </model>
   </sdf>
   ```

### 6.4 Note Importanti

- **Percorsi Relativi:** Assicurati che i percorsi alle texture nel file `.dae` siano relativi e non assoluti.
- **Compatibilità delle Texture:** Utilizza formati di immagine compatibili come `.jpg`, `.png`.
- **Esportazione Corretta:** Durante l'esportazione da Blender o altro software, verifica che l'opzione per includere materiali e texture sia abilitata.

## 7. Differenza tra Materiali Definiti in `.sdf` e in `.dae`

- **Materiali nel `.sdf`:**

  - Definiti direttamente nel file `.sdf` usando il tag `<material>`.
  - Utilizzati per oggetti semplici o per specificare rapidamente colori e proprietà di base.
  - Non supportano texture complesse; principalmente usano colori solidi o script materiali.

- **Materiali nel `.dae`:**

  - Definiti nel software di modellazione 3D e salvati nel file `.dae`.
  - Supportano materiali complessi con texture multiple, mapping UV, normali, ecc.
  - Offrono maggiore controllo e dettaglio visivo.

### 7.1 Quando Usare `.sdf` vs `.dae` per Materiali

- **Usa Materiali in `.sdf` Quando:**

  - Hai bisogno di oggetti semplici con colori solidi o trasparenze basiche.
  - Vuoi modificare rapidamente il colore o la trasparenza senza rieditare la mesh.

- **Usa Materiali in `.dae` Quando:**

  - Hai bisogno di dettagli visivi complessi.
  - Vuoi utilizzare texture dettagliate e mapping UV.
  - Stai lavorando con modelli complessi creati in software 3D.

## 8. Caricamento del Modello in Gazebo

Dopo aver creato i file necessari, puoi avviare Gazebo e inserire il tuo nuovo target nella scena.

**Passi:**

1. **Avvia Gazebo:**

   ```bash
   gazebo
   ```

2. **Inserisci il Modello nella Scena:**

   - Nel pannello dei modelli, cerca il tuo nuovo target (ad esempio, `Nuovo Target`) e trascinalo nella scena.

3. **Verifica:**

   - Assicurati che il modello appaia correttamente e che le texture siano visualizzate come previsto.

## 9. Risoluzione dei Problemi Comuni

- **Il Modello non Appare:**

  - Verifica che tutti i tag XML siano correttamente chiusi e che la sintassi sia corretta.
  - Controlla il terminale per eventuali messaggi di errore.
  - Assicurati che i percorsi nei tag `<uri>` siano corretti.

- **Texture Mancanti:**

  - Assicurati che le immagini siano nella cartella `materials/textures`.
  - Verifica che i percorsi alle texture nel file `.dae` siano relativi.
  - Controlla che i formati delle immagini siano supportati (preferibilmente `.jpg` o `.png`).

- **Percorsi Assoluti vs Relativi:**

  - Evita percorsi assoluti nel file `.dae`; utilizza percorsi relativi al modello.
  - In Blender, prima di esportare, imposta il percorso delle texture su relativo.

- **Formati di Immagine Non Supportati:**

  - Utilizza formati comuni come `.jpg` o `.png`.
  - Evita formati non standard che potrebbero non essere riconosciuti da Gazebo.

## 10. Consigli Aggiuntivi

- **Organizzazione delle Cartelle:**

  - Mantieni una struttura chiara con cartelle per `meshes` e `materials/textures`.
  - Questo aiuta Gazebo a trovare facilmente i file necessari e semplifica la gestione del modello.

- **Verifica dei File:**

  - Utilizza un editor XML per controllare la sintassi dei file `.sdf` e `.config`.

- **Materiali Avanzati:**

  - Puoi definire materiali più complessi utilizzando shader o texture, posizionando i file nella cartella `materials`.

## 11. Conclusione

Seguendo questi passaggi e comprendendo la relazione tra i diversi file e le strutture coinvolte, puoi creare e personalizzare nuovi oggetti in Gazebo, adattandoli alle esigenze della tua simulazione. La gestione di target gassosi e l'uso di texture personalizzate richiedono attenzione alle proprietà visive e fisiche per ottenere un comportamento realistico.

**Punti Chiave:**

- **Comprendere i File:** Il file `.sdf` definisce la struttura e le proprietà del modello, mentre il file `.dae` contiene la geometria dettagliata e i materiali avanzati.

- **Utilizzo delle Immagini:** Le immagini sono utilizzate come texture per arricchire l'aspetto visivo del modello.

- **Personalizzazione Avanzata:** L'uso di software di modellazione 3D come Blender ti permette di creare modelli dettagliati con materiali e texture complessi.

**Risorse Utili:**

- [SDF Format Specification](http://sdformat.org/spec)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)

Se riscontri problemi o hai domande specifiche, non esitare a consultare la documentazione ufficiale o le comunità online dedicate a Gazebo.
