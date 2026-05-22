# Autonomous Drone

## 1. Overview

Autonomous Drone to modułowy system drona autonomicznego. Projekt organizuje kluczowe podsystemy w niezależne paczki w katalogu `packages/`, które wspólnie obsługują:

- percepcję środowiska i ekstrakcję cech z sensorów,
- estymację stanu i pozycjonowanie drona,
- budowę reprezentacji mapy oraz procedury SLAM,
- planowanie trajektorii i ruchu,
- mechanizmy kontroli wykonawczej oraz komunikację.

Framework skupia się na architekturze warstwowej, gdzie każdy moduł powinien być łatwy do wymiany i konfiguracji. W obecnej wersji główny punkt wejścia `main.py` inicjuje logowanie i ma przygotowaną ramę do połączenia poszczególnych komponentów.

Główne założenia/ograniczenia:
- projekt jest zaprojektowany jako Pythonowy pakiet aplikacyjny,
- konfiguracja i logowanie są centralnie zarządzane przez `configs/`,
- logika algorytmiczna powinna być implementowana głównie w `packages/`, a nie w `main.py`,
- obecna wersja zawiera szkielet struktury; wiele podkatalogów jest przygotowanych do dalszej implementacji.


## 2. Requirements

Python / Node / inne — wersje
Zależności systemowe 


## 3. Installation

pip install / npm install / clone + setup
Konfiguracja środowiska (.env, config files)


## 4. Project Structure

### 4.1 Structure

AutonomousDrone/
├── `main.py`             # punkt wejścia aplikacji i orchestracja modułów
├── `README.md`           # dokumentacja projektu
├── `requirements.txt`    # lista zależności Pythona
├── `configs/`            # konfiguracja aplikacji i modułu logowania
│   └── `logger.py`       # wspólna konfiguracja loggera dla aplikacji
├── `data/`               # katalog na zbiory wejściowe i dane testowe
├── `logs/`               # katalog do zapisu plików logów wykonania
├── `notebooks/`          # eksperymentalne notatniki i prototypy
├── `packages/`           # modułowa implementacja systemu drona
│   ├── `common/`         # wspólne narzędzia i klasy pomocnicze
│   ├── `communication/`  # interfejsy i protokoły komunikacji
│   ├── `control/`        # logika sterowania dronem
│   ├── `mappers/`        # moduły mapowania i SLAM
│   │   ├── `feature_matchers/`  # detektory cech i narzędzia dopasowania
│   │   ├── `map_representation/` # reprezentacja map i struktur przestrzennych
│   │   └── `slam/`              # algorytmy SLAM i zarządzanie mapą
│   ├── `perception/`     # przetwarzanie danych sensorowych
│   ├── `planning/`       # planowanie trajektorii i strategii ruchu
│   └── `state_estimation/` # estymacja położenia i stanu drona
└── `tests/`              # miejsce na testy jednostkowe i integracyjne


## 5. CLI Usage

Lista dostępnych komend
Dla każdej: składnia, opcje, przykład

bashpython main.py calculate --input data.csv --mode fast

## 6. API / Programmatic Usage

Jak importować
Główne klasy/funkcje z przykładami kodu


## 7. Configuration

Opis plików konfiguracyjnych
Dostępne parametry i ich wartości domyślne


## 8. Examples

2-3 kompletne use case'y end-to-end


## 9. Testing


## 10. Troubleshooting
