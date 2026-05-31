# Simulation Environment — Setup & Usage Guide

Instrukcja konfiguracji środowiska symulacyjnego dla projektu autonomicznego drona.
Stack: **ROS2 Jazzy + Gazebo Harmonic + PX4 SITL + uXRCE-DDS**.

Docelowy hardware: **M5Stamp Fly (ESP32-S3)**. Symulacja emuluje ograniczenia tego drona
(kamera 2 FPS, ToF zamiast LiDAR, latencja WiFi), dzięki czemu algorytmy rozwijane
w symulacji mają szansę działać na prawdziwym sprzęcie.

---

## Spis treści

1. [Wymagania systemowe](#1-wymagania-systemowe)
2. [Architektura stacku](#2-architektura-stacku)
3. [Instalacja krok po kroku](#3-instalacja-krok-po-kroku)
4. [Procedura uruchomienia](#4-procedura-uruchomienia)
5. [Troubleshooting](#5-troubleshooting)
6. [Struktura projektu vs. narzędzia lokalne](#6-struktura-projektu-vs-narzędzia-lokalne)
7. [Kluczowe informacje techniczne](#7-kluczowe-informacje-techniczne)

---

## 1. Wymagania systemowe

- **OS:** Ubuntu 24.04 LTS (testowane na 24.04.4)
- **Dysk:** ~30 GB wolnego miejsca (PX4 + Gazebo + ROS2 + build artifacts)
- **RAM:** minimum 8 GB, rekomendowane 16 GB
- **GPU:** karta graficzna z obsługą OpenGL 3.3+ (dla Gazebo)
- **Sterownik GPU:** NVIDIA — zainstalowany driver + CUDA (opcjonalnie); AMD/Intel — mesa drivers

### Czy da się to postawić na Windowsie?

**Krótko: natywnie nie.** ROS2 Jazzy oficjalnie istnieje na Windowsie, ale w statusie
"eksperymentalnym" — wymaga budowania ze źródeł, Visual Studio 2019, i wielu ręcznych
poprawek. Gazebo Harmonic na Windowsie natywnie nie działa. PX4 SITL wymaga Linuxa
(`make px4_sitl` kompiluje z GCC pod Linux). Połączenie tych trzech natywnie na Windowsie
to droga przez mękę — nikt tak nie pracuje.

**Realne opcje dla Windowsa:**

1. **WSL2 (rekomendowane):** Windows Subsystem for Linux 2 to pełny Linux wbudowany
   w Windowsa. Instalacja:
   ```powershell
   # PowerShell jako administrator:
   wsl --install -d Ubuntu-24.04
   ```
   Po instalacji otwiera się terminal Ubuntu. Dalej postępujesz **identycznie**
   jak w tej instrukcji. Wymaga Windows 11 z aktualnym driverem GPU.
   Gazebo działa z GPU passthrough (WSLg). Test:
   ```bash
   glxinfo | grep "OpenGL renderer"
   # Powinno pokazać nazwę karty, nie "llvmpipe"
   ```

2. **Dual boot:** Ubuntu 24.04 obok Windowsa. Pełna wydajność, zero problemów.
   Minus: trzeba restartować komputer, żeby przełączyć się między systemami.

3. **VM (VirtualBox / VMware):** Działa do pisania kodu i testowania logiki ROS2,
   ale Gazebo będzie wolne (brak GPU passthrough). Akceptowalne do developmentu
   node'ów, nie do ciężkich symulacji.

4. **Docker na WSL2:** Istnieją gotowe Dockerfile'e z PX4+ROS2+Gazebo
   (np. [PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)).
   Eliminują problem instalacji, ale wymagają WSL2 pod spodem.

**macOS:** brak wsparcia. PX4 SITL + Gazebo Harmonic nie działają na macOS.
Jedyna opcja to VM z Ubuntu lub zdalny dostęp do maszyny Linux.

---

## 2. Architektura stacku

```
┌─────────────┐   UDP 14550 (MAVLink)    ┌─────────┐
│  QGround    │ ◄─────────────────────── │         │
│  Control    │                           │  PX4    │
│  (GUI)      │                           │  SITL   │ ◄── pxh> shell
└─────────────┘                           │         │
                                          └────┬────┘
┌─────────────┐   UDP 8888 (XRCE-DDS)         │
│ MicroXRCE   │ ◄─────────────────────────────┘
│ Agent       │         ▲
└──────┬──────┘         │ symulowane czujniki
       │                │ symulowane silniki
       │ DDS            ▼
       ▼           ┌─────────┐
┌─────────────┐    │ Gazebo  │
│    ROS2     │    │ Harmonic│
│  (nasze     │    │  (3D)   │
│   node'y)   │    └─────────┘
└─────────────┘
```

### Komponenty

| Komponent | Rola | Port/protokół |
|-----------|------|---------------|
| **PX4 SITL** | Firmware autopilota (EKF2, PID, mixery silników) skompilowany na PC | — |
| **Gazebo Harmonic** | Symulacja fizyki, czujników (IMU, GPS, kamera, ToF) | — |
| **MicroXRCEAgent** | Most PX4 ↔ ROS2 (proxy DDS) | UDP 8888 |
| **ROS2 Jazzy** | Framework robotyczny — nasze node'y (SLAM, planner, control) | DDS |
| **QGroundControl** | Stacja naziemna — podgląd telemetrii, mapa, status | UDP 14550 |
| **px4_msgs** | Definicje typów wiadomości PX4 dla ROS2 (Python/C++) | — |

### Przepływ danych

```
Czujniki Gazebo → PX4 (estymuje pozycję EKF2)
    → klient uxrce-dds (wewnątrz PX4, lekki)
    → UDP port 8888
    → MicroXRCEAgent (proxy na PC)
    → topiki ROS2 (/fmu/out/vehicle_local_position_v1, ...)
    → deserializacja przez px4_msgs
    → nasze node'y Python/C++

Nasze node'y → topiki ROS2 (/fmu/in/trajectory_setpoint, ...)
    → MicroXRCEAgent
    → UDP
    → klient uxrce-dds
    → PX4 (wykonuje setpointy)
    → Gazebo (silniki kręcą, dron się rusza)
```

---

## 3. Instalacja krok po kroku

Cała instalacja zajmuje ~30-45 minut (zależnie od prędkości internetu i dysku).

### 3.1 ROS2 Jazzy

```bash
# Dodaj repozytorium ROS2
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalacja
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Weryfikacja
source /opt/ros/jazzy/setup.bash
ros2 topic list
# Powinno pokazać: /parameter_events, /rosout
```

### 3.2 Gazebo Harmonic + ROS2 bridge

```bash
# Dodaj repozytorium OSRF (Gazebo)
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Instalacja
sudo apt update
sudo apt install -y gz-harmonic ros-jazzy-ros-gz

# Weryfikacja (powinno otworzyć okno 3D z kształtami)
gz sim shapes.sdf
# Ctrl+C żeby zamknąć
```

### 3.3 PX4-Autopilot (SITL)

```bash
# Klonuj PX4 (do katalogu domowego, ~15 GB po buildzie)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Zainstaluj zależności systemowe
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh

# Stwórz dedykowany Python venv dla PX4
python3 -m venv ~/px4-venv
source ~/px4-venv/bin/activate
pip install -r ~/PX4-Autopilot/Tools/setup/requirements.txt
deactivate

# Pierwszy build + test (trwa 5-15 min za pierwszym razem)
source ~/px4-venv/bin/activate
cd ~/PX4-Autopilot
make px4_sitl gz_x500
# Powinno otworzyć Gazebo z dronem x500 i shell pxh>
# W pxh> wpisz: commander takeoff
# Dron powinien wzlecieć
# Wpisz: shutdown (żeby zamknąć)
```

### 3.4 Micro XRCE-DDS Agent

Most między PX4 a ROS2. Klient siedzi wewnątrz PX4 (kompilowany razem z firmware),
agent to osobny proces, który odpalamy na PC.

```bash
cd ~
git clone -b v3.0.1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/

# Weryfikacja
MicroXRCEAgent udp4 -p 8888
# Powinno wypisać "running... | port: 8888"
# Ctrl+C żeby zamknąć
```

> **Uwaga:** wersja v2.4.2 NIE działa (brakujący branch fastdds). Używamy v3.0.1.

### 3.5 px4_msgs (workspace ROS2)

Definicje typów wiadomości PX4 dla ROS2. Bez tego `ros2 topic echo` nie rozumie
danych z PX4.

```bash
# Stwórz workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# WAŻNE: klonuj z brancha "main", NIE "release/1.16"
# PX4 v1.17-alpha używa wersjonowanych nazw topików (_v1, _v4)
# i main branch px4_msgs jest z nimi zsynchronizowany
git clone https://github.com/PX4/px4_msgs.git

# Build
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs

# Weryfikacja (wymaga działającego PX4 SITL + Agenta)
source install/setup.bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
# Powinno pokazać dane pozycji drona (x, y, z, vx, vy, vz...)
```

> **Uwaga dotycząca venv:** `colcon build` musi używać **systemowego Pythona**
> (`/usr/bin/python3`), nie Pythona z `px4-venv` czy innego venv.
> Przed budowaniem upewnij się: `deactivate` i `which python3` → `/usr/bin/python3`.

### 3.6 QGroundControl

Stacja naziemna. PX4 SITL wymaga połączenia z GCS, żeby pozwolić na armowanie
(safety check: "ktoś musi nadzorować lot").

```bash
cd ~/Downloads
wget https://github.com/mavlink/qgroundcontrol/releases/latest/download/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage

# Jeśli AppImage nie chce się uruchomić (błąd "fuse"):
sudo apt install libfuse2 -y

# Test
./QGroundControl-x86_64.AppImage
# Powinno otworzyć okno z mapą
# Przy pierwszym uruchomieniu: kliknij OK na dialogach o units i serial devices
```

### 3.7 Opcjonalne aliasy w ~/.bashrc

```bash
# Dodaj na koniec ~/.bashrc:
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

alias px4env='source ~/px4-venv/bin/activate'
alias px4sim='cd ~/PX4-Autopilot && make px4_sitl gz_x500'
alias px4agent='MicroXRCEAgent udp4 -p 8888'
alias px4qgc='~/Downloads/QGroundControl-x86_64.AppImage'
alias px4-clean='pkill -9 -f "gz sim"; pkill -9 -f px4; pkill -9 -f ruby; rm -rf /tmp/gz-* 2>/dev/null'
```

---

## 4. Procedura uruchomienia

Cztery terminale, **w tej kolejności** (kolejność ma znaczenie):

### Terminal 1 — Agent (most PX4 ↔ ROS2)

```bash
MicroXRCEAgent udp4 -p 8888
# albo: px4agent (jeśli dodałeś alias)
```

Zostaw działający. Agent musi wystartować **przed** PX4, żeby klient w PX4
od razu go znalazł.

### Terminal 2 — PX4 SITL + Gazebo

```bash
source ~/px4-venv/bin/activate
cd ~/PX4-Autopilot
make px4_sitl gz_x500
# albo: px4env && px4sim (jeśli dodałeś aliasy)
```

Otworzy się Gazebo z dronem x500. W terminalu pojawi się shell `pxh>`.
W Terminalu 1 (Agent) powinny pojawić się linie `create_participant`, `create_topic`.

### Terminal 3 — QGroundControl

```bash
~/Downloads/QGroundControl-x86_64.AppImage
# albo: px4qgc
```

QGC automatycznie wykryje SITL (UDP 14550). Po ~10s powinien pokazać drona
na mapie (okolice Zurychu — domyślna pozycja PX4).

### Terminal 4 — ROS2 (twój workspace)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Sprawdź topiki
ros2 topic list | grep fmu

# Odczytaj pozycję drona
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once

# Uruchom node sterujący
python3 packages/control/offboard_hover.py
```

### Zamykanie

```
pxh> shutdown          # zamyka PX4 + Gazebo czysto
Ctrl+C na Agent        # zamyka MicroXRCEAgent
Ctrl+C lub zamknij QGC # zamyka QGroundControl
```

Jeśli coś się zawiesi:

```bash
px4-clean
# albo ręcznie:
pkill -9 -f "gz sim"
pkill -9 -f px4
```

---

## 5. Troubleshooting

### "Arming denied: Resolve system health failures first"

**Przyczyna:** Brak połączenia z GCS (QGroundControl nie jest odpalony).
**Fix:** Odpal QGroundControl. Poczekaj ~10s aż pokaże "Ready To Fly".

### "Arming denied" mimo QGC

**Przyczyna:** EKF jeszcze nie ma GPS lock. SITL potrzebuje ~30s po starcie.
**Fix:** Poczekaj 30-60 sekund, potem `commander check` w `pxh>`.

### PX4 build: "ModuleNotFoundError: No module named 'menuconfig'" / 'kconfiglib'

**Przyczyna:** Zły Python venv jest aktywny (np. `siium` zamiast `px4-venv`),
albo żaden venv nie jest aktywny.
**Fix:**
```bash
deactivate                          # wyjdź z bieżącego venv
source ~/px4-venv/bin/activate      # wejdź do px4-venv
which python3                       # weryfikacja: ~/px4-venv/bin/python3
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

### Gazebo się nie otwiera (PX4 startuje, ale brak okna 3D)

**Przyczyna:** Wiszące procesy z poprzedniej sesji blokują start.
**Fix:**
```bash
pkill -9 -f "gz sim"
pkill -9 -f px4
rm -rf /tmp/gz-* 2>/dev/null
# Odpal ponownie
```

### "ros2: command not found"

**Przyczyna:** Nie zsourceowano ROS2 setup.
**Fix:** `source /opt/ros/jazzy/setup.bash`

### "The message type 'px4_msgs/msg/VehicleLocalPosition' is invalid"

**Przyczyna:** Nie zsourceowano workspace z px4_msgs.
**Fix:** `source ~/ros2_ws/install/setup.bash`

### colcon build failuje z "ModuleNotFoundError" / venv conflicts

**Przyczyna:** Aktywny venv (px4-venv lub inny) koliduje z systemowym ROS2.
**Fix:**
```bash
deactivate
which python3   # musi pokazać /usr/bin/python3
colcon build --packages-select px4_msgs
```

### Agent nie łączy się z PX4 (brak linii create_participant)

**Przyczyna:** Agent odpalony **po** PX4, albo na złym porcie.
**Fix:** Zamknij PX4 (`shutdown` w pxh>), odpal Agenta pierwszy, potem PX4.

---

## 6. Struktura projektu vs. narzędzia lokalne

### Co jest w repozytorium Git (wasz kod)

```
Autonomous-drone/
├── packages/
│   ├── control/            ← node'y sterujące (offboard, landing)
│   ├── perception/         ← SLAM, przetwarzanie obrazu
│   ├── planning/           ← planner trajektorii, return-to-dock
│   ├── communication/      ← mosty komunikacyjne
│   ├── state_estimation/   ← filtry, fuzja czujników
│   ├── mappers/            ← budowanie mapy 3D
│   └── common/             ← wspólne utility
├── configs/                ← pliki konfiguracyjne
├── tests/                  ← testy
├── notebooks/              ← eksperymenty, analizy
├── data/                   ← dane testowe (małe pliki)
├── logs/                   ← logi (gitignore'd)
├── main.py
├── requirements.txt        ← zależności Pythonowe NASZEGO kodu
├── SIM_README.md           ← ten plik
├── .gitignore
└── README.md
```

### Co jest lokalne u każdego developera (NIE na Git)

| Ścieżka | Co to jest | Rozmiar |
|----------|-----------|---------|
| `~/PX4-Autopilot/` | Firmware PX4 (klon z GitHub) | ~15 GB |
| `~/Micro-XRCE-DDS-Agent/` | Agent DDS (klon z GitHub) | ~500 MB |
| `~/px4-venv/` | Python venv z zależnościami PX4 | ~200 MB |
| `~/ros2_ws/` | Workspace ROS2 z px4_msgs | ~500 MB |
| `~/Downloads/QGroundControl-x86_64.AppImage` | Stacja naziemna | ~250 MB |
| `/opt/ros/jazzy/` | ROS2 (zainstalowany z apt) | ~1 GB |
| Gazebo Harmonic | Symulator (zainstalowany z apt) | ~500 MB |

Każdy developer instaluje te narzędzia jednorazowo, postępując według sekcji 3.

---

## 7. Kluczowe informacje techniczne

### Wersje (stan na maj 2026)

| Komponent | Wersja | Uwagi |
|-----------|--------|-------|
| Ubuntu | 24.04.4 LTS | jedyny testowany OS |
| ROS2 | Jazzy Jalisco | LTS, wspierane do 2029 |
| Gazebo | Harmonic | LTS, wspierane do 2028 |
| PX4 | v1.17.0-alpha1 | bleeding edge z main |
| Micro XRCE-DDS Agent | v3.0.1 | v2.4.2 NIE działa |
| px4_msgs | main branch | NIE release/1.16 |
| QGroundControl | v5.0+ | najnowszy stabilny |

### Nazwy topiców PX4

PX4 v1.17-alpha używa **wersjonowanych nazw topiców** (sufiksy `_v1`, `_v4`).
To różni się od tutoriali w internecie, które zakładają v1.14-v1.15.

| Typ danych | Nazwa topiku | Typ wiadomości ROS2 |
|------------|-------------|---------------------|
| Pozycja lokalna (odczyt) | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` |
| Status drona (odczyt) | `/fmu/out/vehicle_status_v4` | `px4_msgs/msg/VehicleStatus` |
| Setpoint trajektorii (zapis) | `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` |
| Tryb offboard (zapis) | `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` |
| Komenda (zapis) | `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` |

Pełna lista: `ros2 topic list | grep fmu` (przy działającym stacku).

### Układ współrzędnych

PX4 używa **NED** (North-East-Down):
- `x` = North (do przodu)
- `y` = East (w prawo)
- `z` = Down (**w dół** — ujemne `z` = w górę!)

Czyli: `trajectory_setpoint.position = [0.0, 0.0, -2.0]` oznacza "leć 2m w górę".

### QoS profil dla komunikacji z PX4

Node'y ROS2 muszą używać profilu QoS kompatybilnego z uXRCE-DDS:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

Użycie domyślnego `RELIABLE` spowoduje, że topiki PX4 nie będą widoczne.

### Emulacja ograniczeń M5Stamp Fly (planowane)

Symulacja będzie emulować ograniczenia docelowego hardware'u:

| Sensor / parametr | Stamp Fly (realny) | W symulacji |
|-------------------|--------------------|-------------|
| Kamera | mono, 2 FPS, QVGA | `<update_rate>2</update_rate>` w SDF |
| ToF (VL53L3) | single-point, 0-5m, ~30 Hz | 1-2 promienie ray sensor, range 5m |
| IMU (BMI270) | 6-axis, ~200 Hz | standardowy IMU plugin, 200 Hz |
| Komunikacja | WiFi, 1-5ms latency, packet drops | symulowany node z jitter + drops |
| Compute on-board | ESP32-S3 (brak SLAM) | SLAM na PC, nie na dronie |

---

## Quick Reference Card

```
# === URUCHOMIENIE (4 terminale) ===
T1: MicroXRCEAgent udp4 -p 8888
T2: source ~/px4-venv/bin/activate && cd ~/PX4-Autopilot && make px4_sitl gz_x500
T3: ~/Downloads/QGroundControl-x86_64.AppImage
T4: source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

# === LATANIE ===
pxh> commander takeoff          # start z konsoli PX4
pxh> commander land             # lądowanie z konsoli PX4
python3 offboard_hover.py       # start z ROS2 node'a (T4)

# === DIAGNOSTYKA ===
ros2 topic list | grep fmu      # lista topiców PX4
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 topic hz /fmu/out/vehicle_local_position_v1
pxh> commander check            # preflight checks
pxh> listener vehicle_local_position   # odczyt z konsoli PX4

# === CZYSZCZENIE ===
pxh> shutdown                   # czyste zamknięcie
pkill -9 -f "gz sim"; pkill -9 -f px4   # po crashu
```
