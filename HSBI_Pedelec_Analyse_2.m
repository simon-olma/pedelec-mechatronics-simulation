%% Pedelec – Analyse 2: Dynamische Systemsimulation
% Lehrbeispiel zur modellbasierten Entwicklung eines mechatronischen Systems.
%
% Ziel:
% Untersuchung des Fahrverhaltens und des elektrischen Energiebedarfs
% eines Pedelecs bei unterschiedlichen Unterstützungsstrategien.
% Verglichen werden die Betriebsmodi Eco und Sport.
%
% Modell:
% Simulink-Gesamtsystem mit Fahrzeuglängsdynamik, Mittelmotor,
% vereinfachter Gangwahl, Unterstützungssteuerung und Batteriemodell.
% Die Fahrbahnsteigung wird aus einem vorbereiteten Streckenprofil
% in Abhängigkeit von der zurückgelegten Strecke bestimmt.
%
% Voraussetzungen:
% - HSBI_Pedelec_Parameter.m
% - HSBI_Pedelec_Model.slx
% - GPX_Simulink_Profil.mat
%
% Alle Dateien müssen im aktuellen MATLAB-Verzeichnis liegen.
%
% Ergebnisse:
% - Fahrzeit und elektrischer Energieverbrauch für Eco und Sport
% - Vergleich von Fahrgeschwindigkeit und Motorbetriebspunkten
% - Übersichtsgrafik und kompakte Ergebnisgrafik
%
% Modellgrenzen:
% - Vereinfachte Motorgrenzen ohne thermisches Modell
% - Konstanter Motorwirkungsgrad
% - Idealisierte Gangwechsel
% - Keine detaillierte Batterie- oder Fahrermodellierung
%
% Hinweis:
% Das Modell dient ausschließlich Lehrzwecken. Die Ergebnisse
% sind Modellvorhersagen und keine experimentellen Nachweise.


%% 1. Initialisierung und Modellparameter

clear; clc; close all;

run('HSBI_Pedelec_Parameter.m');


%% 2. Simulationsparameter und Anfangsbedingungen

t_end  = 18000;    % [s]   Maximale Simulationsdauer
t_step = 0.2;      % [s]   Feste Schrittweite des Simulink-Modells

v_0 = 0;           % [m/s] Anfangsgeschwindigkeit
x_0 = 0;           % [m]   Anfangsposition

% Das Simulink-Modell verwendet diese Variablen.
% Die Simulation endet spätestens bei t_end oder beim Erreichen
% des im Modell definierten Streckenendes.


%% 3. Streckenprofil laden

% Das vorbereitete Profil enthält:
% sGPX:     zurückgelegte Strecke [m]
% alphaGPX: Fahrbahnneigungswinkel [rad]

profil = load('GPX_Simulink_Profil.mat', ...
              'sGPX', 'alphaGPX');

sGPX     = double(profil.sGPX(:));
alphaGPX = double(profil.alphaGPX(:));

streckenlaenge_km = sGPX(end)/1000;

fprintf('\n--- Referenzstrecke ---\n');
fprintf('Streckenlänge: %.2f km\n',streckenlaenge_km);


%% 4. Simulink-Modell laden

mdl = 'HSBI_Pedelec_Model';
load_system(mdl);

% Fahrmodus-Parameter suchen
modusBloecke = find_system(mdl, ...
    'LookUnderMasks','all', ...
    'BlockType','Constant', ...
    'Name','Fahrmodus');
modusBlock = modusBloecke{1};
modusOriginal = get_param(modusBlock,'Value');


%% 5. Simulation der Unterstützungsmodi

kEco   = 1.0;   % Eco:   Motorunterstützung mit Faktor k = 1
kSport = 2.5;   % Sport: Motorunterstützung mit Faktor k = 2.5

fprintf('\n--- Simulink-Simulation ---\n');

set_param(modusBlock,'Value',num2str(kEco));
fprintf('Simuliere Eco ...\n');
Eco = sim(mdl);

set_param(modusBlock,'Value',num2str(kSport));
fprintf('Simuliere Sport ...\n');
Sport = sim(mdl);

% Ursprünglichen Wert des Fahrmodusblocks wiederherstellen
set_param(modusBlock,'Value',modusOriginal);


%% 6. Simulationsdaten einlesen

datenEco   = Eco.get('simData');
datenSport = Sport.get('simData');


%% 7. Simulationsergebnisse aufbereiten

% Zurückgelegte Strecke [km]
sE = datenEco.x.Data(:) / 1000;
sS = datenSport.x.Data(:) / 1000;

% Fahrgeschwindigkeit [km/h]
vE = datenEco.v.Data(:) * 3.6;
vS = datenSport.v.Data(:) * 3.6;

% Mechanisches Motorwellenmoment [Nm]
M_M_E = datenEco.M_M.Data(:);
M_M_S = datenSport.M_M.Data(:);

% Mechanische Motorwellenleistung [W]
P_M_E = datenEco.P_M.Data(:);
P_M_S = datenSport.P_M.Data(:);

% Gesamtübersetzung [-]
iE = datenEco.i.Data(:);
iS = datenSport.i.Data(:);

% Kumulierter elektrischer Energieverbrauch [Wh]
EE = datenEco.E_V.Data(:);
ES = datenSport.E_V.Data(:);

% Batterieladezustand [-] (0 ... 1)
SoC_E = datenEco.SoC.Data(:);
SoC_S = datenSport.SoC.Data(:);

% Simulationszeit [s]
tE = datenEco.x.Time(:);
tS = datenSport.x.Time(:);

% Steigung aus dem vorgegebenen Streckenprofil [%]
sProfil_km = sGPX / 1000;
steigung = 100 * tan(alphaGPX);

% Gemeinsamer Darstellungsbereich
s_max = max([sE; sS]);


%% 8. Ergebnisausgabe

% Prüfung, ob beide Simulationen das Streckenende erreicht haben.
% Toleranz: 50 m aufgrund der diskreten Simulationsschritte.

toleranz_km = 0.05;

zielEco = abs(sE(end)-streckenlaenge_km) <= toleranz_km;
zielSport = abs(sS(end)-streckenlaenge_km) <= toleranz_km;

if ~zielEco || ~zielSport
    warning(['Mindestens eine Simulation hat das Streckenende ', ...
             'nicht innerhalb der Toleranz erreicht. ', ...
             'Fahrzeit und Energiebedarf sind dann nur für ', ...
             'die jeweils simulierte Teilstrecke vergleichbar.']);
end

fprintf('\n--- Simulationsergebnisse ---\n');
fprintf('                       Eco        Sport\n');
fprintf('Fahrstrecke [km]    %8.2f     %8.2f\n',sE(end),sS(end));
fprintf('Fahrzeit [min]      %8.1f     %8.1f\n',tE(end)/60,tS(end)/60);
fprintf('Energie [Wh]        %8.1f     %8.1f\n',EE(end),ES(end));
fprintf('Verbrauch [Wh/km]   %8.2f     %8.2f\n', ...
    EE(end)/sE(end),ES(end)/sS(end));

fprintf('SoC am Ende [%%]    %8.1f     %8.1f\n', ...
    100*SoC_E(end),100*SoC_S(end));

% Rechnerische Energiebilanz der angenommenen Batterie
% unter der vereinfachenden Annahme, dass die gesamte
% Nennenergie genutzt werden kann.

restEco = SoC_0*E_Batt_nom - EE(end);
restSport = SoC_0*E_Batt_nom - ES(end);

fprintf('Restenergie [Wh]    %8.1f     %8.1f\n',restEco,restSport);

% Plausibilitätsprüfung des vereinfachten Batteriemodells
SoC_E_berechnet = SoC_0 - EE(end)/E_Batt_nom;
SoC_S_berechnet = SoC_0 - ES(end)/E_Batt_nom;

fprintf('\n--- Prüfung der Energiebilanz ---\n');
fprintf('Eco:   SoC Simulation = %.3f | Berechnung = %.3f\n', ...
    SoC_E(end),SoC_E_berechnet);

fprintf('Sport: SoC Simulation = %.3f | Berechnung = %.3f\n', ...
    SoC_S(end),SoC_S_berechnet);

%% 9. Gemeinsame Diagrammeinstellungen

% Farben der Unterstützungsmodi
colorEco   = [40/255 160/255 0/255];  % Grün
colorSport = [192/255 0/255 0/255];   % Rot

navy = [0 26/255 84/255];

fontsize = 13;

exportAbbildungen = false;

% Skalierung abhängig von den Simulationsergebnissen
v_oben = max(80,ceil(max([vE; vS])/10)*10);
E_oben = max(500,ceil((max([EE; ES])+35)/100)*100);
P_oben = max(550,ceil(max([P_M_E; P_M_S])/50)*50+50);


%% 10. Übersichtsgrafik mit sechs Ergebnisgrößen

figUebersicht = figure('Color','w', ...
    'Name','Dynamische Pedelec-Systemsimulation', ...
    'Position',[100 100 1250 780]);

tl = tiledlayout(figUebersicht,3,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

ax = gobjects(6,1);

for k = 1:6

    ax(k) = nexttile(tl);

    hold(ax(k),'on');
    grid(ax(k),'on');
    box(ax(k),'off');

    ax(k).FontSize = 11;
    ax(k).LineWidth = 0.9;
    ax(k).TickDir = 'out';
    ax(k).GridAlpha = 0.15;
    ax(k).XColor = navy;
    ax(k).YColor = navy;

    xlim(ax(k),[0 s_max]);
end


% 1 – Streckensteigung

plot(ax(1),sProfil_km,steigung, ...
    'Color',navy, ...
    'LineWidth',1.8);

yline(ax(1),0,':', ...
    'Color',navy, ...
    'LineWidth',0.8);

ylabel(ax(1),'Steigung [\%]', ...
    'Interpreter','latex');

ylim(ax(1),[-15 15]);
yticks(ax(1),-15:5:15);


% 2 – Fahrgeschwindigkeit

hEco = plot(ax(2),sE,vE, ...
    'Color',colorEco, ...
    'LineWidth',2.2);

hSport = plot(ax(2),sS,vS, ...
    'Color',colorSport, ...
    'LineWidth',2.2);

yline(ax(2),25,'--','25 km/h', ...
    'Color',navy, ...
    'LineWidth',1.3, ...
    'LabelHorizontalAlignment','right');

ylabel(ax(2),'$v$ [km/h]', ...
    'Interpreter','latex');

ylim(ax(2),[0 v_oben]);


% 3 – Kumulierte elektrische Energie

plot(ax(3),sE,EE, ...
    'Color',colorEco, ...
    'LineWidth',2.2);

plot(ax(3),sS,ES, ...
    'Color',colorSport, ...
    'LineWidth',2.2);

ylabel(ax(3),'$E_V$ [Wh]', ...
    'Interpreter','latex');

ylim(ax(3),[0 E_oben]);


% 4 – Gesamtübersetzung

plot(ax(4),sE,iE, ...
    'Color',colorEco, ...
    'LineWidth',1.8);

plot(ax(4),sS,iS, ...
    'Color',colorSport, ...
    'LineWidth',1.8);

ylabel(ax(4),'$i$ [-]', ...
    'Interpreter','latex');

ylim(ax(4),[4.7 8.3]);
yticks(ax(4),5:8);


% 5 – Motorwellenmoment

plot(ax(5),sE,M_M_E, ...
    'Color',colorEco, ...
    'LineWidth',2.0);

plot(ax(5),sS,M_M_S, ...
    'Color',colorSport, ...
    'LineWidth',2.0);

yline(ax(5),M_M_max,'--', ...
    'Color',navy, ...
    'LineWidth',1.2);

ylabel(ax(5),'$M_M$ [Nm]', ...
    'Interpreter','latex');

ylim(ax(5),[0 max(6,1.1*max([M_M_E; M_M_S]))]);


% 6 – Mechanische Motorwellenleistung

plot(ax(6),sE,P_M_E, ...
    'Color',colorEco, ...
    'LineWidth',2.0);

plot(ax(6),sS,P_M_S, ...
    'Color',colorSport, ...
    'LineWidth',2.0);

% Nenndauerleistung und angenommene Spitzenleistungsgrenze
yline(ax(6),P_Nenn,'--', ...
    'Color',navy, ...
    'LineWidth',1.2);

yline(ax(6),P_M_max,':', ...
    'Color',navy, ...
    'LineWidth',1.2);

ylabel(ax(6),'$P_M$ [W]', ...
    'Interpreter','latex');

ylim(ax(6),[0 P_oben]);


% Gemeinsame x-Achsenbeschriftung

xlabel(ax(5),'Strecke [km]');
xlabel(ax(6),'Strecke [km]');


% Legende unterhalb der Diagramme

lgd = legend(ax(2),[hEco hSport], ...
    {'Eco','Sport'}, ...
    'Orientation','horizontal', ...
    'Box','off', ...
    'FontSize',fontsize);

lgd.Layout.Tile = 'south';


%% 11. Kompakte Ergebnisgrafik

figErgebnis = figure('Color','w', ...
    'Name','Systemsimulation: Ergebnisse und Bewertung', ...
    'Position',[100 100 1100 350]);

tl2 = tiledlayout(figErgebnis,1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');


% 1 – Fahrgeschwindigkeit

ax1 = nexttile(tl2);
hold(ax1,'on');
grid(ax1,'on');
box(ax1,'off');

h1 = plot(ax1,sE,vE, ...
    'Color',colorEco, ...
    'LineWidth',1.5);

h2 = plot(ax1,sS,vS, ...
    'Color',colorSport, ...
    'LineWidth',1.5);

xlabel(ax1,'$x$ [km]', 'Interpreter','latex');
ylabel(ax1,'$v$ [km/h]', 'Interpreter','latex');

xlim(ax1,[0 s_max]);
ylim(ax1,[0 v_oben]);

ax1.FontSize = fontsize;
ax1.LineWidth = 0.9;
ax1.TickDir = 'out';
ax1.GridAlpha = 0.15;
ax1.XColor = navy;
ax1.YColor = navy;


% 2 – Kumulierte elektrische Energie

ax2 = nexttile(tl2);
hold(ax2,'on');
grid(ax2,'on');
box(ax2,'off');

plot(ax2,sE,EE, ...
    'Color',colorEco, ...
    'LineWidth',1.5);

plot(ax2,sS,ES, ...
    'Color',colorSport, ...
    'LineWidth',1.5);

% Energieverbrauch am Ende der simulierten Fahrt
text(ax2,sE(end)-1.0,EE(end)+20, ...
    sprintf('%.0f Wh',EE(end)), ...
    'Color',colorEco, ...
    'FontSize',fontsize, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','right');

text(ax2,sS(end)-1.0,ES(end)+20, ...
    sprintf('%.0f Wh',ES(end)), ...
    'Color',colorSport, ...
    'FontSize',fontsize, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','right');

xlabel(ax2,'$x$ [km]', 'Interpreter','latex');
ylabel(ax2,'$E_V$ [Wh]', 'Interpreter','latex');

xlim(ax2,[0 s_max]);
ylim(ax2,[0 E_oben]);

ax2.FontSize = fontsize;
ax2.LineWidth = 0.9;
ax2.TickDir = 'out';
ax2.GridAlpha = 0.15;
ax2.XColor = navy;
ax2.YColor = navy;


% Gemeinsame Legende

lgd2 = legend(ax1,[h1 h2], ...
    {'Eco','Sport'}, ...
    'Orientation','horizontal', ...
    'Box','off', ...
    'FontSize',fontsize);

lgd2.Layout.Tile = 'south';


%% 12. Optionaler Export 

if exportAbbildungen
    exportgraphics(figUebersicht, ...
        'pedelec_systemsimulation_uebersicht.emf', ...
        'BackgroundColor','white');

    exportgraphics(figErgebnis, ...
        'pedelec_systemsimulation_eco_sport.emf', ...
        'BackgroundColor','white');
end
