# README - Guida per creare nuovi Worlds in Gazebo

Questa guida fornisce le istruzioni per creare e personalizzare i "worlds" in Gazebo, un simulatore 3D utilizzato nella piattaforma 4DDS.

## Avvio di Gazebo

Per iniziare, aprire il software Gazebo utilizzando il comando:

```bash
gazebo
```

Questo comando deve essere eseguito dalla shell PowerShell o dalla WSL. Se si desidera aprire un "world" esistente per modificarlo, aggiungere il percorso del file al comando:

Esempio:

```bash
gazebo /home/fourdds/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/<nome_file>.world
```

## Personalizzazione del World

Una volta aperto Gazebo, è possibile modificare il "world" utilizzando i modelli presenti all'interno di Gazebo. I modelli si trovano nella scheda "Insert" situata in alto a sinistra dell'interfaccia di Gazebo.

Aggiungere, spostare o eliminare gli oggetti presenti nel "world" per creare uno scenario personalizzato.

## Salvataggio del World

Dopo aver apportato le modifiche desiderate, salvare il "world" utilizzando il menu "File -> Save World".

**Percorso consigliato per il salvataggio:**

```bash
/home/fourdds/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
```

Questo è il percorso in cui vengono salvati tutti i "world" di Gazebo, per una facile gestione e utilizzo.

## Esecuzione del World salvato

Per eseguire il "world" salvato, utilizzare il comando seguente:

```bash
./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n <numero_droni> -w <nome_world>
```

Assicurarsi di sostituire `<numero_droni>` con il numero di droni che si desidera simulare e `<nome_world>` con il nome del "world" salvato.

## Esportazione/Importazione del World

Per esportare o importare un "world" da un altro sistema, utilizzare la shell WSL e digitare il comando di copia (`cp`).

**Esempio di comando per esportare un world:**

```bash
cp /home/fourdds/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/find_targets.world /mnt/c/Users/username/Desktop/
```

Questo comando copia il file `find_targets.world` nella directory `Desktop` dell'utente in Windows.

## Consegna dei Worlds creati

Tutti gli scenari creati (file `.world`) durante il lavoro di tesi devono essere consegnati insieme alle classi implementate. Assicurarsi di organizzare i file in modo chiaro per facilitare il processo di revisione e utilizzo da parte degli altri membri del team.
