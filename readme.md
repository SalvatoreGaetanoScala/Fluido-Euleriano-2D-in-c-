# Simulazione di "Fluido Euleriano 2D" (C++17 + raylib — Architettura MVC)

Simulazione interattiva in tempo reale di un fluido incomprimibile e trasporto di fumo su griglia Euleriana 2D, sviluppata in **C++17** con **raylib** secondo un'architettura **Model-View-Controller (MVC)** modulare e scalabile.

---

## Struttura del Progetto

```text
fluid2d_mvc/
├── .gitignore
├── CMakeLists.txt
├── README.md
├── include/
│   ├── model/
│   │   ├── FluidGrid.hpp            # Struttura dati 1D contigua per campi scalari, vettoriali e ostacoli
│   │   └── FluidSolver.hpp          # Risolutore numerico Navier-Stokes (zero dipendenze da raylib)
│   ├── view/
│   │   ├── FluidTexture.hpp         # Wrapper RAII per Image e Texture2D in raylib
│   │   └── SimulationView.hpp       # Rendering campi, frecce vettoriali e HUD interattivo
│   └── controller/
│       └── SimulationController.hpp # Loop principale (fixed timestep) e gestione input utente
└── src/
    ├── main.cpp                     # Entry point e ciclo di vita della finestra
    ├── model/
    │   ├── FluidGrid.cpp
    │   └── FluidSolver.cpp
    ├── view/
    │   ├── FluidTexture.cpp
    │   └── SimulationView.cpp
    └── controller/
        └── SimulationController.cpp
```

### Separazione dei Ruoli (MVC)
* **Model (`fluid_model`):** libreria statica C++ pura, completamente disaccoppiata da `raylib`. Gestisce la memoria dei campi fisici in array monodimensionali linearizzati per massimizzare la *cache locality* e facilita l'integrazione di test unitari.
* **View:** traduce i campi fisici del Model in mappe cromatiche RGBA e li trasferisce alla GPU tramite `FluidTexture` (gestita con idiomi **RAII**).
* **Controller:** coordina l'aggiornamento fisico tramite **fixed timestep con accumulatore** (evitando la *spiral of death* su cali di framerate) e mappa gli input di mouse e tastiera sulle operazioni del risolutore.

---

## Modello Fisico e Leggi Matematiche

Il risolutore implementa una variante stabile (*Stable Fluids* di Jos Stam) delle **equazioni di Navier-Stokes per fluidi incomprimibili** in due dimensioni su una griglia regolare con celle fantasma per le condizioni al contorno.

### 1. Equazione del Momento (Navier-Stokes)
L'evoluzione del campo di velocità $\mathbf{u} = (u_x, u_y)$ è governata da:

$$\frac{\partial \mathbf{u}}{\partial t} = -(\mathbf{u} \cdot \nabla)\mathbf{u} + \nu \nabla^2 \mathbf{u} - \frac{1}{\rho}\nabla p + \mathbf{f}$$

dove:
* $-(\mathbf{u} \cdot \nabla)\mathbf{u}$ rappresenta l'**avvezione** (trasporto del campo lungo se stesso), risolta con metodo **semi-Lagrangiano** (*backtracing* delle particelle nel passato con interpolazione bilineare) per garantire stabilità incondizionata anche con passi temporali grandi.
* $\nu \nabla^2 \mathbf{u}$ rappresenta la **diffusione viscosa** ($\nu$ = viscosità cinematica), risolta tramite schema implicito e iterazioni di **Gauss-Seidel**.
* $-\frac{1}{\rho}\nabla p$ è il gradiente di pressione che impone la conservazione della massa.
* $\mathbf{f}$ raccoglie le forze esterne: impulsi del mouse, **galleggiamento (buoyancy)** proporzionale alla densità del fumo ($f_y \propto -\beta d$), gravità e **confinamento della vorticità**.

### 2. Condizione di Incompressibilità (Proiezione di Helmholtz-Hodge)
Per mantenere il fluido incomprimibile, il campo di velocità deve avere divergenza nulla:

$$\nabla \cdot \mathbf{u} = 0$$

Al termine degli step di diffusione e avvezione, il solver calcola la divergenza del campo intermedio $\mathbf{w}$, risolve l'equazione di Poisson per la pressione $p$:

$$\nabla^2 p = \nabla \cdot \mathbf{w}$$

e sottrae il gradiente di pressione al campo di velocità ($\mathbf{u} = \mathbf{w} - \nabla p$), rendendo il flusso privo di divergenza e facendolo deviare realisticamente attorno agli ostacoli solidi.

### 3. Trasporto di Scalari (Densità del Fumo)
La concentrazione di fumo $d$ viene trasportata dal campo di velocità tramite un'equazione di avvezione-diffusione con dissipazione:

$$\frac{\partial d}{\partial t} = -(\mathbf{u} \cdot \nabla)d + \kappa \nabla^2 d - \alpha d$$

dove $\kappa$ è il coefficiente di diffusione e $\alpha$ il fattore di decadimento nel tempo.

### 4. Confinamento della Vorticità (*Vorticity Confinement*)
Per contrastare la dissipazione numerica introdotta dall'interpolazione bilineare e preservare i piccoli vortici caratteristici del fumo, il solver calcola il rotore scalare in 2D:

$$\omega = \frac{\partial u_y}{\partial x} - \frac{\partial u_x}{\partial y}$$

e applica una forza correttiva lungo il gradiente normalizzato della magnitudine della vorticità:

$$\mathbf{N} = \frac{\nabla |\omega|}{\|\nabla |\omega|\| + \epsilon}, \qquad \mathbf{f}_{\text{vort}} = \varepsilon (\mathbf{N} \times \boldsymbol{\omega})$$

---

## Librerie e Strumenti Utilizzati

* **C++17:** standard del linguaggio (`std::vector`, `std::clamp`, RAII, `[[nodiscard]]`).
* **raylib 5.5:** libreria grafica e multimediale per creazione finestra, gestione input, aggiornamento texture dinamiche (`UpdateTexture`) e disegno HUD 2D. Viene scaricata e compilata automaticamente tramite `CMake FetchContent`.
* **CMake ($\ge$ 3.20):** sistema di build multipiattaforma configurato con flag di warning severi (`-Wall -Wextra -Wpedantic -Werror`) e supporto opzionale ai runtime sanitizer (`ASan` + `UBSan`).

---

## Controlli dell'Applicazione

| Input | Azione |
| :--- | :--- |
| **Click Sinistro + Trascina** | Inietta densità di fumo e applica una spinta nella direzione del movimento del mouse |
| **Click Destro + Trascina** | Disegna ostacoli solidi sul dominio del fluido |
| **Shift + Click Destro** | Cancella gli ostacoli solidi sotto il pennello |
| **Rotella del Mouse** | Aumenta o riduce il raggio del pennello (`2` – `30` celle) |
| **`1`** | Modalità vista: **Densità Fumo** |
| **`2`** | Modalità vista: **Campo Velocità** (direzione codificata in tonalità HSV e intensità) |
| **`3`** | Modalità vista: **Vorticità** (mappa di calore del rotore $|\omega|$) |
| **`4`** | Modalità vista: **Pressione** (mappa di calore del campo $p$) |
| **`V`** | Attiva / disattiva la sovrapposizione delle frecce del campo vettoriale |
| **`S`** | Attiva / disattiva la sorgente continua di fumo dal basso |
| **`SPAZIO`** | Mette in pausa o riprende la simulazione |
| **`N`** | Avanza la simulazione di un singolo passo temporale (*single-step* per debug) |
| **`C`** | Rimuove tutti gli ostacoli solidi mantenendo il fluido |
| **`R`** | Reset completo (azzera campi fisici e rimuove gli ostacoli) |
| **`H`** | Mostra / nasconde il pannello dei controlli nell'HUD |

---

## Compilazione ed Esecuzione

### Requisiti
* Compilatore C++17 (AppleClang, Clang, GCC o MSVC)
* CMake $\ge$ 3.20
* Git (per il download automatico di `raylib 5.5` al primo configure)

### Build Release (Prestazioni Ottimali)
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/fluid2d_mvc
```

### Build di Sviluppo con Sanitizers (ASan + UBSan)
Per rilevare accessi fuori dai limiti o comportamenti indefiniti durante la modifica del solver:
```bash
cmake -S . -B build-san -DENABLE_SANITIZERS=ON
cmake --build build-san -j
./build-san/fluid2d_mvc
```