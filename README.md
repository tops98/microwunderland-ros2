# Fahrtplanung  für Smartcars im Mikrowunderland
## Packages:
+ **Servo Controller:**\
*Hardwareabstraktion für Weichensteuerung*
+ **Object Detector:**\
*Object Detector basierend auf ssd_mobilenet_fp_320 zur Erkennung von Fahrzeugen in über Bilddaten*
+ **Object Tracker:**\
*Assoziiert neue erkannten Fahrzeugpositionen mit bekannten um die Identität zu beobachtender Objekte/ Fahrzeuge über mehrere Frames aufrechtzuerhalten*
+ **Traffic Planner:**\
*Steuert Weichen und fahrzeuge nach einen im yaml format festlegbaren plan*

+ **Tracker Visualizer:**\
*Zeichnet die erkannten Objekte/ Fahrzeuge auf dem empfangen Bilddaten ein*

# How to install
**TODO**

# Offene ToDos:
+ Service für ändern von Namen von getrackten Fahrzuegen
+ Fish eye Effekt von Bildern entfernen
+ Weichenpositionen ermitteln
+ Object detector auf realen Daten testen ggf. nachtrainieren
+ Traffic planer mit neuen Tracker testen
+ map mit für simulation mit Pixelkoordinaten Aktualisieren