%% Pedelec – Analyse 1: Stationäre Antriebsauslegung
% Lehrbeispiel zur modellbasierten Entwicklung eines mechatronischen Systems.
%
% Ziel:
% Untersuchung des Leistungs- und Drehmomentbedarfs eines Pedelecs
% für zwei stationäre Betriebsfälle:
%   1. Steigungsfahrt: 9 % Steigung bei 8 km/h
%   2. Ebene Fahrt:    0 % Steigung bei 24 km/h
%
% Mit einem vereinfachten Referenzmotormodell werden die erforderliche
% Motorwellenleistung und die zulässigen Gesamtübersetzungen bestimmt.
% Anschließend wird geprüft, ob eine feste bzw. schaltbare Übersetzung
% beide Betriebsfälle abdecken kann.
%
% Voraussetzungen:
% - HSBI_Pedelec_Parameter.m
%
% Ergebnisse:
% - Tabelle mit Leistungs-, Momenten- und Übersetzungsanforderungen
% - Prüfung der Motorgrenzen und gewählten Gangübersetzungen
% - Diagramme für die grafische Auswertung
%
% Modellannahmen:
% - Stationäre Fahrt mit konstanter Geschwindigkeit
% - Konstante Fahrerleistung am Rad
% - Kein Gegen- oder Rückenwind
% - Konstante Fahrwiderstandsbeiwerte und Übertragungsverluste
% - Vereinfachte Motorgrenzen ohne thermisches Modell
%
% Hinweis:
% Das Modell dient Lehrzwecken. Die Motorparameter sind exemplarisch
% gewählt.


%% 1. Initialisierung und Modellparameter

clear; clc; close all;

run('HSBI_Pedelec_Parameter.m');


%% 2. Definition der stationären Betriebsfälle

% Steigung als Verhältnis (z. B. 0.09 entspricht 9 %)
s = [0.09, 0.00];

v_kmh = [8, 24];               % [km/h] Fahrgeschwindigkeiten
v = v_kmh / 3.6;               % [m/s]  Umrechnung in SI-Einheiten

namen = {'Steigungsfahrt', 'Ebene Fahrt'};

%% 3. Berechnung des stationären Antriebsbedarfs

% Umrechnung der Steigung in den Fahrbahnneigungswinkel
alpha = atan(s);

% Fahrwiderstände
FS = m*g.*sin(alpha);          % [N] Steigungswiderstand
FR = c_R*m*g.*cos(alpha);      % [N] Rollwiderstand
FW = 0.5*rho*c_w_A.*v.^2;      % [N] Luftwiderstand

Fges = FS + FR + FW;           % [N] Gesamter Fahrwiderstand

% Motorischer Anteil am erforderlichen Vortrieb
% F_F = P_F / v; negative Motorzugkraft wird zu null begrenzt.
FM = max(0, Fges - P_F./v);     % [N] Motorische Zugkraft

MRad = FM*r;                   % [Nm] Motorisches Drehmoment am Rad
PMotor = FM.*v/eta_G;          % [W] Mechanische Motorwellenleistung


%% 4. Zulässige Gesamtübersetzungen

% Definition: i = omega_M / omega_Rad
%
% Momentengrenze:
% M_M = MRad / (i*eta_G) <= M_M_max
% Daraus folgt i >= imin.
%
% Drehzahlgrenze:
% omega_M = i*v/r <= omega_M_max
% Daraus folgt i <= imax.

imin = MRad./(eta_G*M_M_max);
imax = omega_M_max*r./v;


%% 5. Ergebnisausgabe und Prüfung der Motorgrenzen

T = table(namen(:),s(:)*100,v_kmh(:),FM(:),MRad(:), ...
          PMotor(:),imin(:),imax(:), ...
    'VariableNames', ...
    {'Betriebsfall','Steigung_Prozent','v_kmh','F_M_N', ...
     'M_Rad_Nm','P_M_W','i_min','i_max'});

fprintf('\n--- Stationäre Antriebsauslegung ---\n');
disp(T);

% Prüfung der angenommenen Nenndauerleistung
leistungOK = all(PMotor <= P_Nenn);

if leistungOK
    fprintf('Leistungsprüfung: Beide Betriebsfälle liegen unter %.0f W.\n', ...
        P_Nenn);
else
    fprintf('Leistungsprüfung: Nenndauerleistungsgrenze überschritten.\n');
end

% Prüfung, ob sich die zulässigen Übersetzungsbereiche überschneiden
i_unten = max(imin);
i_oben  = min(imax);

festeUebersetzungMoeglich = ...
    leistungOK && (i_unten <= i_oben);

fprintf('\n--- Prüfung einer festen Gesamtübersetzung ---\n');
fprintf('Erforderliches Gesamtintervall: %.2f <= i <= %.2f\n', ...
    i_unten, i_oben);

if festeUebersetzungMoeglich
    fprintf('Eine feste Übersetzung ist rechnerisch möglich.\n');
else
    fprintf('Keine feste Übersetzung erfüllt alle Prüfbedingungen.\n');
end


%% 6. Prüfung der gewählten Gangübersetzungen

% Schaltbarer Antrieb mit zwei vorgegebenen Gesamtübersetzungen
% aus der gemeinsamen Parameterdatei.
iWahl = [i_1, i_2];

% Tatsächliche Motorbetriebspunkte
MWelle = MRad./(eta_G*iWahl);   % [Nm] Motorwellenmoment
omegaWelle = iWahl.*v/r;       % [rad/s] Motorwinkelgeschwindigkeit
PWelleCheck = MWelle.*omegaWelle;

fprintf('\n--- Prüfung des schaltbaren Antriebs ---\n');

for k = 1:2

    momentOK = MWelle(k) <= M_M_max;
    drehzahlOK = omegaWelle(k) <= omega_M_max;
    leistungOK = PMotor(k) <= P_Nenn;

    istZulaessig = momentOK && drehzahlOK && leistungOK;

    fprintf(['%s: i = %.1f | M_M = %.2f Nm | ', ...
             'omega_M = %.2f rad/s | P_M = %.1f W | ', ...
             'zulässig = %d\n'], ...
             namen{k},iWahl(k),MWelle(k),omegaWelle(k), ...
             PMotor(k),istZulaessig);
end

% Konsistenzprüfung der mechanischen Leistungsberechnung
assert(all(abs(PWelleCheck-PMotor) < 1e-9), ...
    'Motorwellenleistung und Antriebsmodell sind nicht konsistent.');


%% 7. Gemeinsame Diagrammeinstellungen

% Farben der Betriebsfälle
color_1 = [220/255 70/255 243/255;   % Magenta: Steigungsfahrt
           127/255 127/255 127/255]; % Grau: Ebene Fahrt

navy = [0 26/255 84/255];

fontsize = 13;

exportAbbildungen = false;


%% 8. Diagramm 1: Motorleistungsbedarf

fig = figure('Color','w', ...
    'Name','Stationäre Antriebsauslegung', ...
    'Position',[100 100 750 260]);

tl = tiledlayout(fig,1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

% Geschwindigkeitsbereiche für die beiden Fahrzustände
geschw = {linspace(6,12,120), linspace(16,28,120)};

for k = 1:2

    ax = nexttile(tl);
    hold(ax,'on');
    grid(ax,'on');
    box(ax,'off');

    vk = geschw{k};            % [km/h]
    vm = vk/3.6;               % [m/s]
    alpha_k = atan(s(k));

    % Erforderliche mechanische Motorwellenleistung
    Fbedarf = m*g*sin(alpha_k) ...
            + c_R*m*g*cos(alpha_k) ...
            + 0.5*rho*c_w_A.*vm.^2;

    Pbedarf = max(0,Fbedarf.*vm-P_F)/eta_G;

    % Verlauf des Motorleistungsbedarfs
    hBedarf = plot(ax,vk,Pbedarf, ...
        'Color',color_1(k,:), ...
        'LineWidth',2.8);

    % Angenommene Nenndauerleistungsgrenze
    yline(ax,P_Nenn,'--', ...
        'Color',navy, ...
        'LineWidth',1.6);

    % Untersuchter Betriebspunkt
    plot(ax,v_kmh(k),PMotor(k),'o', ...
        'MarkerFaceColor',color_1(k,:), ...
        'MarkerEdgeColor','w', ...
        'LineWidth',1.2, ...
        'MarkerSize',9);

    % Leistungswert am Betriebspunkt
    if k == 1
        text(ax,v_kmh(k)+0.20,PMotor(k)-28, ...
            sprintf('%.0f W',PMotor(k)), ...
            'Color',color_1(k,:), ...
            'FontSize',fontsize, ...
            'FontWeight','bold');
    else
        text(ax,v_kmh(k)-1.0,PMotor(k)+50, ...
            sprintf('%.0f W',PMotor(k)), ...
            'Color',color_1(k,:), ...
            'FontSize',fontsize, ...
            'FontWeight','bold');
    end

    xlabel(ax,'$v$ [km/h]','Interpreter','latex');

    if k == 1
        ylabel(ax,'$P_M$ [W]','Interpreter','latex');
    end

    % Einheitliche Achsenskalierung
    xlim(ax,[vk(1) vk(end)]);
    ylim(ax,[0 400]);
    yticks(ax,0:100:400);

    ax.FontSize = 11;
    ax.LineWidth = 0.9;
    ax.TickDir = 'out';
    ax.GridAlpha = 0.15;
    ax.XColor = navy;
    ax.YColor = navy;

    % Legendenobjekte in derselben Achse erzeugen
    if k == 1
        hSteigung = hBedarf;

        hEbene = plot(ax,nan,nan, ...
            'Color',color_1(2,:), ...
            'LineWidth',2.8);
    end
end

% Gemeinsame Legende
lgd = legend([hSteigung hEbene], ...
    {'Steigungsfahrt: 9 %; 8 km/h', ...
     'Ebene Fahrt: 0 %; 24 km/h'}, ...
    'Orientation','horizontal', ...
    'Box','off', ...
    'FontSize',12);

lgd.Layout.Tile = 'south';


%% 9. Diagramm 2: Zulässige Gesamtübersetzungen

fig = figure('Color','w', ...
    'Name','Übersetzungsprüfung', ...
    'Position',[100 100 750 140]);

ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
box(ax,'off');

% Zulässige Übersetzungsbereiche:
% Steigungsfahrt: i >= imin(1)
% Ebene Fahrt:    i <= imax(2)
%
% Die dargestellten Balken werden an den Achsengrenzen abgeschnitten.
% Die tatsächlichen mathematischen Bereiche sind nicht dort begrenzt.

plot(ax,[imin(1) 10],[2 2], ...
    'Color',color_1(1,:), ...
    'LineWidth',7);

plot(ax,[3.5 imax(2)],[1 1], ...
    'Color',color_1(2,:), ...
    'LineWidth',7);

% Grenzpunkte
plot(ax,imin(1),2,'o', ...
    'MarkerSize',9, ...
    'MarkerFaceColor',color_1(1,:), ...
    'MarkerEdgeColor','w');

plot(ax,imax(2),1,'o', ...
    'MarkerSize',9, ...
    'MarkerFaceColor',color_1(2,:), ...
    'MarkerEdgeColor','w');

% Beschriftung der Grenzwerte
text(ax,imin(1)+0.12,2.23, ...
    sprintf('$i \\geq %.2f$',imin(1)), ...
    'Interpreter','latex', ...
    'Color',color_1(1,:), ...
    'FontSize',fontsize, ...
    'FontWeight','bold');

text(ax,imax(2)-0.12,1.23, ...
    sprintf('$i \\leq %.2f$',imax(2)), ...
    'Interpreter','latex', ...
    'HorizontalAlignment','right', ...
    'Color',color_1(2,:), ...
    'FontSize',fontsize, ...
    'FontWeight','bold');

% Achseneinstellungen
xlim(ax,[3.5 10.2]);
ylim(ax,[0.65 2.5]);

xticks(ax,4:10);
yticks(ax,[1 2]);
yticklabels(ax,{'Ebene Fahrt','Steigungsfahrt'});

% Einheitliche Darstellung
ax.FontSize = 11;
ax.LineWidth = 0.9;
ax.TickDir = 'out';
ax.GridAlpha = 0.15;
ax.XGrid = 'on';
ax.YGrid = 'off';
ax.XColor = navy;
ax.YColor = navy;

% Ausreichender linker Rand für die Betriebsfallbezeichnungen
ax.Position = [0.25 0.25 0.72 0.65];


%% 10. Optionaler Export für die Präsentation
if exportAbbildungen
    exportgraphics(findobj('Name','Stationäre Antriebsauslegung'), ...
        'motorleistungsbedarf.emf','BackgroundColor','white');
    
    exportgraphics(findobj('Name','Übersetzungsprüfung'), ...
        'uebersetzungsbereiche.emf','BackgroundColor','white');
end

