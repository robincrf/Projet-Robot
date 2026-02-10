# Robot Suiveur de Ligne (STM32) — Suivi de Parcours avec LEDs

Projet en C pour un robot roulant basé sur STM32 capable de **suivre une ligne noire** et d’indiquer son état et sa trajectoire grâce à des **LEDs**.  
Le projet met l’accent sur le **temps réel**, le contrôle moteur et la fiabilité des capteurs en environnement embarqué.

---

##  Fonctionnalités
- **Suivi automatique d’une ligne noire**
- **Système de LEDs**
  - Indication direction (gauche / droite / tout droit)
  - États du robot (calibration, ligne perdue, erreur)
- Exécution **temps réel** via timers et interruptions STM32
- Architecture modulaire : capteurs, contrôle, moteurs, LEDs

---

##  Principe de fonctionnement
1. Les **capteurs infrarouges** détectent le contraste ligne / sol.
2. Un **algorithme de contrôle** calcule la correction de trajectoire.
3. Les **PWM moteurs** sont ajustés pour recentrer le robot.
4. Les **LEDs affichent l’état du robot** en temps réel.

---

##  Matériel utilisé
- Carte **STM32** (F1 / F4 ou équivalent)
- Moteurs DC + **pont en H**
- Capteurs de ligne (IR reflectance)
- LEDs (sorties GPIO)
- Batterie / alimentation embarquée

---

##  Stack firmware
- Langage : **C**
- Drivers : **STM32 HAL**
- Outils : **STM32CubeIDE** (ou PlatformIO)

---

