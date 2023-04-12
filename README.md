# **Digitaler Zwilling Mikrowunderland** <br> [*Dokumentation nach arc42*]

## [1] Einführung und Ziele
### [1.1] Aufgabenstellung
Der Digitale Zwilling soll dafür genutzt werden können komplexere Szenarien für Experimente im Bereich der Miniaturautonomie durchführen zu können, sowie einen Möglichst hohen grad an Automatisierung bei der Überwachung und Durchführung dieser zu gewährleisten.
Dies soll durch Folgende übergeordnete Funktionen des Digitalen Zwillings erreicht werden:
* Möglichst vollständige Zustandserfassung des Mikrowunderlands
* Kontrolle aller elektronisch steuerbaren Systeme
* Simulation von vorzudefiniertender Szenarien
* Prognose zukünftiger Zustände mittels Extrapolation

| ID | Name | Beschreibung|
|---|---|---|
| R_1 | HAL | Alle im Dokument ***Zustandserfassung*** genannten Systeme werden durch je eine Softwareabstraktion vollständig repräsentiert, welche alle Funktionen des Systems in einer Schnittstelle bündelt
| R_2 | Positionsbestimmung von Fahrzeugen | Fahrzeugpositionen können live erfasst werden
| R_3 | Straßennetz | Das Straßennetz des Mikrowunderlands ist Digital repräsentiert. Ermittelte Positionen aus ***R_2*** können auf das Digitale Straßennetz Projiziert werden
| R_4 | Aufgaben für Smart-Carts | Smart-Carts kann eine Aufgabe in form eines an zu fahrenden Ziels, oder eines kontinuierlich zu folgenden Zyklus Strecke gegeben werden
| R_5 | Überwachung/Streckenverfolgung ML-Cart | Es kann überwacht werden ob ein ML-Cart einer vordefinierten Strecke folgt oder davon abweicht
| R_6 | Virtuelle Verkehrsteilnehmer | Es können Virtuelle Verkehrsteilnehmer hinzugefügt werden. Informationen über ihre Position, Geschwindigkeit und Route können abgerufen werden
| R_7 | Interface für manuelle Kontrolle | Es existiert ein Interface mit dem sich alle Systeme direkt ansteuern lassen können
| R_8 | Visualisierung | Der Tischaufbau wird als 3D Modell visualisiert 
| R_9 | Szenarien | Es können Pläne erstellt werden mittels welcher sich spezielle Szenarien wiederholbar, ausführen lassen. Ein Szenario kann dabei aus Zeitpunkten oder Intervallen bestehen zu denen Events, z.B. das an oder aus schalten von Lichtern, Ampeln, etc. durchgeführt wird, sowie das Starten eines anderen Services mit einer Liste vorgegebener Parameter.
### [1.2] Qualitätsziele
| ID | Name | Beschreibung |
|---|---|---|
| Q_1 | Abstraktion | Es gibt eine Abstraktion für jedes Hardware System, welche ohne Kenntnis der zugrunde liegenden Hardwarebausteine genutzt werden kann
| Q_2 | Erweiterbarkeit | Neu Hardware System wie auch reine Software Systeme sollen sich ohne das Änderungen an bestehenden Softwareartefakten durchgeführt werden müssen einfügen lassen
| Q_3 | Latenz | Steuerbefehle entfalten ihre Wirkung rechtzeitig also so das die Reaktion dem ordnungsgemäßen Ablauf gewährleistet
| Q_4 | Graphiken | Die 3D Modelle der Visualisierung müssen optisch nicht den realen Objekt gleichen, sollen jedoch den realen Pendant zuordenbar sein
### [1.3] Stakeholder
| Name | Rolle | E-mail |
|---|---|---|
Tobias Haugg     | Bachelorrand, Entwicker | tobias.haugg@haw-hamburg.de
Stephan Pareigis | Professor               | stephan.pareigis@haw-hamburg.de
Markus Karsten   | Studentische Hilfe      | markus.kasten@haw-hamburg.de
Daniel Riege   | Studentische Hilfe      | daniel.riege@haw-hamburg.de

## [2] Randbedingungen
### [2.1] Technische Randbedingungen
| ID | Name | Beschreibung |
|---|---|---|
| C_1 | Performance | Alle Softwareartefakte müssen mit den auf einem Raspberry Pi 4 zur Verfügungsteeenden Ressourcen uneingeschränkt lauffähig sein
| C_2 | Hartes Echtzeitsystem | Es gibt Deadlines bei deren nicht Einhaltung kein ordnungsgemäßer Betrieb möglich ist
| C_4 | Platform | Alle Softwareartefakte, ausgenommen Artefakte die für Mikroprozessoren (Arduino, ESP32, etc) geschrieben werden müssen, haben auf Linux lauffähig zu sein
## [3] Kontextabgrenzung
### [3.1] Technischer Kontext
![Alt-Text](/diagramme/Systemkontext_DigitalerZwilling.jpg)

## [4] Lösungskonzept
- Das System ist als Service Orientierte Architektur aufgebaut. Zweck der Service Orientierten Architektur ist es das ersetzen, erweitern sowie einbinden neuer Dienste möglichst einfach zu gestalten ( siehe Qualitätsziele: Q1, Q2).
- Als Middleware für das System wird einen DDS-Architektur (Data Distributed Service) verwendet. Diese bietet ähnlich wie andere Publish Subscribe Systeme die Möglichkeit Daten über Topics verschiedenen Prozessen asynchron zugänglich zu machen. Vorteil der DDS-Architektur ist es, dass Nachrichten direkt vom Publisher zu den Subscribern gesendet werden, also kein Broker notwendig ist. Hierdurch wird sowohl die Resilienz des Systems gefördert, da kein SPOF existiert, als auch die Skalierbarkeit (Q_2). Ferner lässt sich das Nachrichten aufkommen hierdurch ebenfalls optimieren, da Nachrichten nicht erst zum Broker und von dort zu den Subscribern gesendet werden müssen (Q_3). Darüber hinaus bietet DDS eine umfangreiche Auswahl an Quality of Service Policies mit welchen sich die Einhaltung von Echtzeitanforderungen überwachen lässt (siehe Technische Randbedingungen: C_2)
