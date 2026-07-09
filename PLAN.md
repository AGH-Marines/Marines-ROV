Specyfikacja Projektu: Autonomiczny System Nawigacji i Mapowania 3D dla Robota Podwodnego (ROV)
1. Cel ProjektuGłównym celem projektu jest stworzenie modularnego, niezawodnego i skalowalnego systemu sterowania oraz percepcji dla robota podwodnego (ROV) w środowisku ROS2 (Humble). Robot ma za zadanie wykonywać autonomiczne misje polegające na bezpiecznym przemieszczaniu się z punktu A do punktu B po najkrótszej (optymalnej) ścieżce, skutecznie omijając statyczne i dynamiczne przeszkody w przestrzeni trójwymiarowej.
Kluczowe założenia funkcjonalne:Stabilizacja bazy (Station Keeping): Robot aktywnie kompensuje siły zewnętrzne (wypornościowe, prądy wodne) i utrzymuje zadaną głębokość ($Z$) oraz orientację ($Yaw$) za pomocą kaskadowych regulatorów PID.Percepcja i Mapowanie 3D: System przetwarza surowe dane z sonaru wielowiązkowego (Multibeam) lub kamer stereowizyjnych i buduje ciągłą mapę zajętości przestrzeni w postaci drzewa ósemkowego (OctoMap).Autonomiczne Planowanie Ścieżki: System wyznacza bezpieczną trajektorię w przestrzeni 3D (6 DoF) wolną od kolizji.Abstrakcja Sprzętowa (Portability): Architektura oddziela warstwę symulacji od logiki biznesowej robota. Przeniesienie systemu na fizycznego robota wymaga jedynie podmiany pakietu wejściowego (sterowników czujników) oraz wyjściowego (miksera silników).Architektura oparta na TF2: Cała lokalizacja i sterowanie bazują na zapytaniach do dynamicznego drzewa transformacji (tf2), eliminując sztywne powiązania z tematami odometrii.2. Tryby Pracy (Launchery nadrzędne)System sterowania udostępnia dwa główne tryby operacyjne uruchamiane z poziomu pakietu integracyjnego:Tryb 1: Sterowanie Padem (Manualno-Półautonomiczny)Launcher: teleop_joy.launch.pyOpis działania: Operator kontroluje ruch postępowy robota (osie $X, Y$) za pomocą gałek kontrolera (np. od Xbox/PlayStation). Osie pionowa ($Z$) oraz obrotowa ($Yaw$) są opcjonalnie kontrolowane przez automatyczny węzeł stabilizacji. Gdy operator puści gałki, robot automatycznie "blokuje" swoją pozycję w wodzie.Zastosowanie: Misje inspekcyjne, mapowanie wstępne zbiornika, procedury awaryjne.Tryb 2: Pełna Autonomia (Przejazd punkt A -> B)Launcher: autonomy.launch.pyOpis działania: Operator (lub zewnętrzna maszyna stanów) zadaje współrzędne celu ($X, Y, Z, Yaw$). Pakiet nawigacji analizuje aktualną strukturę OctoMapy, generuje optymalną trasę i przekazuje ją do regulatora trajektorii. Robot samodzielnie wykonuje pełen przejazd, na bieżąco korygując trasę w przypadku wykrycia nowych obiektów.3. Struktura Folderów Przestrzeni Roboczej (Workspace)Plaintextros2_ws/
├── Dockerfile
├── README.md
└── src/
    ├── rov_bringup/
    │   ├── launch/
    │   │   ├── autonomy.launch.py
    │   │   └── teleop_joy.launch.py
    │   └── rviz/
    │       └── default_view.rviz
    ├── rov_simulation/
    │   ├── data/              # Modele 3D (.obj, .dae) robota i przeszkód
    │   ├── launch/
    │   │   └── sim_launch.py
    │   └── scenarios/
    │       └── underwater_world.scn
    ├── rov_localization/
    │   ├── config/
    │   │   └── ekf.yaml
    │   └── launch/
    │       └── localization_launch.py
    ├── rov_mapping/
    │   └── launch/
    │       └── mapping_launch.py
    ├── rov_navigation/
    │   ├── launch/
    │   │   └── navigation_launch.py
    │   └── rov_navigation/
    │       ├── __init__.py
    │       └── planner_node.py
    ├── rov_control/
    │   ├── launch/
    │   │   └── control_launch.py
    │   └── rov_control/
    │       ├── __init__.py
    │       └── pid_controller_node.py
    ├── rov_actuation/
    │   ├── launch/
    │   │   └── actuation_launch.py
    │   └── rov_actuation/
    │       ├── __init__.py
    │       └── thruster_allocator_node.py
    └── rov_mission/
        ├── launch/
        │   └── mission_launch.py
        └── rov_mission/
            ├── __init__.py
            └── state_machine_node.py
4. Szczegółowy Opis Pakietów1. rov_simulation (Warstwa Środowiskowa)Typ: ament_cmake (Zarządzanie zasobami i plikami sceny dla Stonefish)Zadanie: Odpowiada za fizykę świata, hydrodynamikę robota, odwzorowanie grawitacji i wyporu. Generuje sztuczny szum dla sensorów.Wejścia: /thrusters/STONEFISH (std_msgs/Float64MultiArray - wektor ciągu silników)Wyjścia: Tematy surowe czujników: /marines_rov/odometry, /marines_rov/imu, /marines_rov/dvl, /marines_rov/point_cloud.2. rov_localization (Warstwa Estymacji Stanu)Typ: Integracja zewnętrznych narzędzi ROS2 (robot_localization)Zadanie: Wykonuje fuzję danych z IMU (orientacja, przyspieszenia), DVL (prędkość liniowa względem dna) oraz odometrii kinematycznej za pomocą Rozszerzonego Filtru Kalmana (EKF).Kluczowa funkcja: Odfiltrowuje szum i publikuje dynamiczną relację w drzewie transformacji: odom -> base_link.Wyjścia: /odometry/filtered (nav_msgs/Odometry), transformacja TF2.3. rov_mapping (Warstwa Percepcji 3D)Typ: Integracja zewnętrznych narzędzi ROS2 (octomap_server)Zadanie: Buduje trójwymiarową mapę środowiska. Pobiera chmurę punktów z wirtualnego sonaru, pobiera aktualną pozycję robota poprzez tf2 (relacja odom -> base_link) i rzutuje punkty do przestrzennej bazy danych wokseli.Wyjścia: /octomap_binary, /octomap_full (octomap_msgs), /occupied_cells_vis_array (Wizualizacja struktur 3D w RViz2).4. rov_navigation (Warstwa Planowania Trasy)Typ: ament_pythonZadanie: Implementuje algorytm poszukiwania najkrótszej bezpiecznej ścieżki (np. RRT* lub 3D A*). Reaguje na komendy przejścia do punktu docelowego. Śledzi zajętość przestrzeni na podstawie mapy OctoMapy.Wejścia: Mapy zajętości, /marines_rov/goal_pose (geometry_msgs/PoseStamped)Wyjścia: /marines_rov/global_path (nav_msgs/Path - lista punktów trasy).5. rov_control (Warstwa Sterowania i Śledzenia Trajektorii)Typ: ament_pythonZadanie: Odczytuje zadaną ścieżkę lub pozycję. Wykorzystuje tf2_ros.TransformListener, aby w czasie rzeczywistym sprawdzać pozycję base_link względem układu mapy/odom. Liczy uchyb i generuje wektor pożądanych sił za pomocą PID.Wejścia: Dynamiczne drzewo TF2, zadana ścieżka.Wyjścia: /marines_rov/cmd_wrench (geometry_msgs/Wrench - oczekiwane siły $F_x, F_y, F_z$ i moment $M_z$).6. rov_actuation (Warstwa Sprzętowa / Mixer)Typ: ament_pythonZadanie: Implementuje Macierz Alokacji Ciągu (TAM). Pobiera ogólne żądanie siły i rozbija je na poszczególne pędniki robota.Przenaszalność (Portability): W symulacji wyjście jest publikowane do Stonefisha. Na prawdziwym robocie ten pakiet wysyła pakiety bezpośrednio do sterowników silników (np. poprzez protokół I2C, PWM lub magistralę CAN).Wejścia: /marines_rov/cmd_wrenchWyjścia: /thrusters/STONEFISH (w symulacji) lub bezpośredni interfejs sprzętowy.7. rov_mission (Zarządzanie Misją / Logika Nadrzędna)Typ: ament_pythonZadanie: Mózg operacyjny robota (Maszyna Stanów / Drzewo Zachowań). Odpowiada za sekwencję zdarzeń: uzbrojenie silników -> zanurzenie -> start mapowania -> wysłanie celu -> monitorowanie stanu -> powrót.8. rov_bringup (Integracja i Orkiestracja)Typ: Meta-pakiet startowy.Zadanie: Nie zawiera kodu źródłowego, grupuje pliki launch i konfiguracje RViz2. Pozwala jedną komendą uruchomić cały ekosystem.5. Wymagane Pakiety i Modyfikacja Środowiska (Dockerfile)Projekt opiera się na zaawansowanych bibliotekach nawigacyjnych i percepcyjnych systemu ROS2. Aby konteneryzowane środowisko deweloperskie mogło poprawnie zbudować przestrzeń roboczą, niezbędna jest modyfikacja pliku Dockerfile w celu zainstalowania kluczowych zależności systemowych.Wymagane pakiety zewnętrzne ROS2:ros-humble-robot-localization (Rozszerzony Filtr Kalmana EKF 3D)ros-humble-octomap-server (Serwer mapowania i integracji drzew ósemkowych)ros-humble-octomap-msgs (Definicje wiadomości ROS2 dla struktur OctoMap)ros-humble-joy & ros-humble-teleop-twist-joy (Obsługa interfejsu i mapowania padów)Wymagana sekcja w pliku Dockerfile:Przed etapem kopiowania kodu źródłowego i wywołaniem colcon build, w pliku Dockerfile należy umieścić instrukcję instalacji pakietów APT:Dockerfile# Aktualizacja repozytoriów i instalacja zależności dla systemów percepcji, lokalizacji i kontroli
RUN apt-get update && apt-get install -y \
    ros-humble-robot-localization \
    ros-humble-octomap-server \
    ros-humble-octomap-msgs \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*
6. Strategia Przeniesienia na Platformę Sprzętową (Hardware Porting)Struktura została zaprojektowana tak, aby przejście z symulatora na fizycznego robota pracującego np. pod kontrolą komputera jednopłytkowego (Raspberry Pi 5 / NVIDIA Jetson) zamontowanego na pokładzie ROV, wymagało minimalnej ingerencji w kod:Plaintext    [ TRYB SYMULACJI ]                               [ TRYB FIZYCZNEGO ROBOTA ]
  ┌───────────────────────┐                        ┌───────────────────────┐
  │  pakiet:              │                        │  pakiet:              │
  │  rov_simulation       │                        │  sterowniki_hardware  │
  └──────────┬────────────┘                        └──────────┬────────────┘
             │ (Surowe czujniki)                              │ (Prawdziwe czujniki)
             v                                                v
┌──────────────────────────────────────────────────────────────────────────────────┐
│                             Wspólny Rdzeń Systemu                                │
│  [rov_localization (EKF)] -> [rov_mapping (OctoMap)] -> [rov_navigation (RRT*)]  │
│                                       │                                          │
│                                       v                                          │
│                            [rov_control (PID z TF2)]                             │
└───────────────────────────────────────┬──────────────────────────────────────────┘
                                        │ (Żądany Wrench)
                                        v
                          ┌───────────────────────────┐
                          │ pakiet: rov_actuation     │
                          └─────────────┬─────────────┘
                                        │
                 ┌──────────────────────┴──────────────────────┐
                 ▼ (Wybór środowiska uruchomieniowego)         ▼
      [ Symulator Stonefish ]                         [ Fizyczne pędniki ]
      (Float64MultiArray do .scn)                     (Interfejs PWM / I2C / CAN)
Odcięcie symulacji: W nadrzędnym pliku launch pakietu rov_bringup zmieniamy flagę use_sim na false. Powoduje to nieuruchamianie pakietu rov_simulation.Podpięcie fizycznych czujników: Wstawiamy w to miejsce węzły producenta fizycznego sprzętu (np. pakiet micro-ROS dla IMU, sterowniki DVL po porcie szeregowym, oraz sterownik zed-ros2-wrapper dla fizycznej kamery). Ważne jest jedynie zremapowanie tematów wyjściowych tak, aby trafiały do rov_localization i rov_mapping.Adaptacja alokatora pędników: W pakiecie rov_actuation podmieniamy klasę publikującą. Zamiast wysyłać wektor ciągu na temat /thrusters/STONEFISH, importujemy bibliotekę obsługi szyny sprzętowej (np. smbus/PCA9685 dla Pythona) i wysyłamy sygnały bezpośrednio na piny sterowników pędników. Cała warstwa lokalizacji, mapowania 3D oraz planowania ścieżki pozostaje nienaruszona.