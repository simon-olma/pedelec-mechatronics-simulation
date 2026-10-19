%% Pedelec – Modellparameter
% Lehrbeispiel zur modellbasierten Entwicklung eines mechatronischen Systems.
%
% Dieses Skript definiert die gemeinsamen Fahrzeug-, Motor- und
% Batterieparameter für die stationäre Antriebsauslegung (Analyse 1)
% und die dynamische Systemsimulation (Analyse 2).
%
% Alle Motor- und Batterieparameter sind exemplarische Modellannahmen
% und stammen nicht aus einem konkreten Herstellerdatenblatt. Das Modell 
% ist für Lehrzwecke vereinfacht 

%% 1. Fahrzeug- und Umgebungsparameter

m     = 135;       % [kg]       Gesamtmasse inkl. Fahrer und Gepäck
g     = 9.81;      % [m/s^2]    Erdbeschleunigung
r     = 0.35;      % [m]        Dynamischer Radradius
c_R   = 0.006;     % [-]        Rollwiderstandsbeiwert
rho   = 1.225;     % [kg/m^3]   Luftdichte
c_w_A = 0.60;      % [m^2]      Luftwiderstandsfläche
P_F   = 100;       % [W]        Mechanische Fahrerleistung am Rad

%% 2. Referenzmotor und mechanischer Antriebsstrang

eta_G       = 0.90;  % [-]     Mechanischer Übertragungswirkungsgrad
P_Nenn      = 250;   % [W]     Mechanische Nenndauerleistung an der Motorwelle
P_M_max     = 500;   % [W]     Angenommene kurzzeitige Spitzenleistung
M_M_max     = 5;     % [Nm]    Maximales Motorwellenmoment
omega_M_max = 115;   % [rad/s] Maximale Motorwinkelgeschwindigkeit

% Vereinfachung:
% Die Motorgrenzen werden ohne thermisches Motormodell betrachtet.

%% 3. Motorwirkungsgrad und Batterie

eta_M      = 0.80;  % [-]      Konstanter elektrischer Motorwirkungsgrad
E_Batt_nom = 500;   % [Wh]     Angenommene Batterienennenergie
SoC_0      = 1.00;  % [-]      Anfangsladezustand (1 = 100 %)

% Vereinfachung:
% Der Motorwirkungsgrad wird unabhängig von Drehzahl und Moment
% angenommen. Ein detailliertes Batteriemodell wird nicht verwendet.

%% 4. Gesamtübersetzung und Gangwahl

i_1  = 8;          % [-]       Gesamtübersetzung im ersten Gang
i_2  = 5;          % [-]       Gesamtübersetzung im zweiten Gang
v_gs = 15;         % [km/h]    Geschwindigkeitsschwelle für den Gangwechsel

% Vereinfachung:
% Die Gangwahl erfolgt geschwindigkeitsabhängig mit zwei fest
% vorgegebenen Übersetzungen. Schaltvorgänge werden idealisiert.

%% 5. Geschwindigkeitsabhängige Motorunterstützung

v_fade = 23;       % [km/h] Beginn der Reduzierung der Unterstützung
v_max  = 25;       % [km/h] Ende der Motorunterstützung

% Zwischen v_fade und v_max wird die Unterstützung reduziert.

%% 6. Untere Grenzwerte zur numerischen Absicherung

v_min       = 0.5;  % Einheit und Verwendung im Simulink-Modell prüfen
omega_M_min = 0.1;  % [rad/s] Untere Grenze für Berechnungen mit omega_M

% Diese Grenzwerte vermeiden mögliche Divisionen durch null
% bei sehr kleinen Geschwindigkeiten bzw. Motorwinkelgeschwindigkeiten.
