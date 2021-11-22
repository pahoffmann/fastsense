# Loop Closure in TSDF based SLAM #

# Inhalt #

1. Definition des Problems
2. Plan zur Lösung des Problems
3. Evaluation der Lösung des Problems




# Kapitel 1 - Problemdefinition #

Loop Closure ( auch Schleifenschluss ) ist ein wichtiges und besonders in der Large Scale Reconstruction unabdingbares und nützliches Feature zur Erkennung bereits bekannter Posen im dreidimensionalen Raum und nachfolgender Korrektur der vorherigen Posen auf Basis des erkannten Poseunterschieds.
Durch diese Posekorrekturen entsteht eine konsistentere Karte und ein Drift, der möglicherweise im Posegraphen vorherrscht, kann ausgeglichen werden.

Damit dies funktioniert, müssen (alle / die) vorigen Positionen gespeichert werden (z.B. in einem Posegraph), damit sie im Falle eines Schleifenschlusses optimiert werden können.
Zur Detektion eines Schleifenschlusses, muss eine vergleichbare Darstellung (Deskriptor) für den zur Pose p gesehenen TSDF-Kartenausschnitt, bzw. die zum Pose tp gesehene Punktwolke existieren. Es existieren diverse Methoden, einen Deskriptor für Punktwolken zu definieren. Hier gilt es zu evaluieren, ob diese Deskriptoren auch auf implizite Oberflächendarstellungen angepasst werden können und genügend Informationen für einen Vergleich enthalten (ggf. TSDF-Werte nahe Null nehmen??).
Nach Berechnung eines Deskriptors gilt es, diesen mit den Deskriptoren vergangener Posen zu vergleichen ( wann sinnvoll ). Wird ein Schleifenschluss detektiert, muss neben dem Posegraphen auch die TSDF-Karte optimiert werden (ggf. asynchron). Dafür werden Informationen über vergangene Cell-States benötigt ( vor allem: aus welcher Position wurde der implizite Oberflächenwert welcher Zelle aktualisiert?). Wie speichere ich diese Information? Direkt an der Zelle? Braucht es hier eine Historie?

# Kapitel 2 - Problemlösung #

Dieses Problem ist groß und muss in unterschiedliche Teilprozesse gegliedert werden.

1. Basics
   1. Vertraut machen mit dem Problem
   2. Ursprungspaper für Loop Closure finden
   3. wie wurde darauf aufgesetzt
   4. wie funktioniert grundsätzlich das Auffinden der Matches, welche Ansätze gibt es


2. Basisimplementation des Matching Algorithms
   1. Wie funktioniert das Updaten der Positionen? Ggf. basic icp mit loop closure implementieren?




# Kapitel 3 - Evaluation #

* Welche Evaluationsmöglichkeiten gibt es?
* Visuell veranschaulichen (Posegraph previous and after loop closure)
* Vergleich mit ground truth (Positionen), Messung der Abweichung
