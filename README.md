# ČASŤ 1. ODOMETRIA

V prvej časti bolo potrebné naprogramovať odometriu robota a základný pohyb k nastavenému cieľu. 

Pôvodne sme v kóde využívali odometriu z koliesok robota, ale po problémoch s inými časťami zadania sme sa rozhodli pre využitie gyroskopu. 

Kód využíva dáta z enkodérov a gyroskopu robota na odhad jeho aktuálnej pozície a orientácie. 

Tieto dáta sú poskytované užívateľovi priamo v ovládacom paneli robota (viď Obrázok 1). Cieľ, ku ktorému sa robot presúva, sa nastaví tlačidlom (Pri mapovaní je to tlačidlo „Mapovanie“ a pri prechádzaní mapy je to tlačidlo „Ďalší bod“). 

![image](https://github.com/user-attachments/assets/1cf938ca-d955-4c71-9484-fa72ba9fd143)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*Obr. 1. Tlačidlá na ovládanie a zobrazenie dát o polohe robota* 
## Metódy

### `void robot::moveToGoal(double goal_x, double goal_y)`

* Nastaví cieľové súradnice `targetX` a `targetY` pre robot. 
* Nastaví `movingToGoal` na `true`, čím aktivuje logiku pohybu k cieľu v metóde `processThisRobot`. 
* Resetuje `distance_scan` na `false`, aby sa pri prvom behu s novým cieľom prepočítala celková vzdialenosť. 

### `int robot::processThisRobot(TKobukiData robotdata)`

Toto je callback funkcia, ktorá sa volá vždy, keď prídu nové dáta z robota (enkodéry, gyroskop atď.). [cite: 8] Implementuje hlavnú logiku odometrie a navigácie k cieľu.

#### Inicializácia:

Pri prvom spustení (`firstRun == true`) uloží aktuálne hodnoty enkodérov (`EncoderRight`, `EncoderLeft`) a gyroskopu (`GyroAngle`) do predošlých hodnôt a nastaví `firstRun` na `false`. 

#### Konštanty:

Definuje konštanty súvisiace s geometriou robota a enkodérmi: 

* `WHEEL_RADIUS`: Polomer kolesa. [cite: 10]
* `WHEEL_BASE`: Rozchod kolies. [cite: 10]
* `TICKS_PER_REV`: Počet impulzov enkodéra na jednu otáčku kolesa. [cite: 11]
* `TWO_PI`: 2π. [cite: 11]
* `MAX_ENCODER_VALUE`: Maximálna hodnota enkodéra (pre detekciu pretečenia). [cite: 11]

#### Výpočet zmien enkodérov a gyroskopu:

Vypočíta zmenu hodnôt pravého a ľavého enkodéra (`deltaRight`, `deltaLeft`) a zmenu uhla z gyroskopu (`gyroFi`) od posledného merania. [cite: 12] Uhol z gyroskopu sa konvertuje na radiány (`gyroFi_RAD`). [cite: 13]

#### Ošetrenie pretečenia enkodérov:

Kontroluje a koriguje zmeny enkodérov, ak došlo k pretečeniu (wrap-around). [cite: 13, 14] Predpokladá sa 16-bitový enkodér. [cite: 14]

#### Aktualizácia predošlých hodnôt:

Uloží aktuálne hodnoty enkodérov a gyroskopu pre ďalší výpočet. [cite: 14]

#### Konverzia impulzov na uhlovú rotáciu kolies:

Premení zmenu impulzov enkodérov na uhlovú rotáciu kolies v radiánoch (`omegaRight`, `omegaLeft`). [cite: 15]

#### Výpočet lineárnej a uhlovej rýchlosti robota:

* Lineárna rýchlosť `v` sa vypočíta ako priemer rýchlostí oboch kolies vynásobený polomerom kolesa. [cite: 16]
* Uhlová rýchlosť `omega` sa získava priamo z gyroskopu (`gyroFi_RAD`). Pôvodné riešenie bolo cez kolieska. [cite: 17, 18]

#### Výpočet novej orientácie:

Aktualizuje aktuálnu orientáciu robota (`fi`) pridaním zistenej uhlovej zmeny (`omega`). [cite: 18]

#### Výpočet novej pozície:

Aktualizuje súradnice robota (`x`, `y`) na základe vypočítanej lineárnej rýchlosti a zmeny orientácie pomocou diferenciálnych rovníc pre pohyb robota: [cite: 19]

* Ak robot rotuje (`fabs(omega) > 1e-6`), použijú sa rovnice pre otáčanie na mieste. [cite: 19, 20]
* Ak sa robot pohybuje priamo (`fabs(omega) <= 1e-6`), použijú sa jednoduché rovnice pre priamy pohyb. [cite: 20]

#### Normalizácia orientácie:

Uhol `fi` sa normalizuje do rozsahu `[-π, π]` pomocou funkcie `atan2(sin(newFi), cos(newFi))`, aby sa predišlo problémom s viacnásobnými reprezentáciami rovnakého uhla. [cite: 21, 22] Ak bol pôvodne v stupňoch, konvertuje sa späť na stupne. [cite: 22]

#### Logika pohybu k cieľu (`if (movingToGoal)`):

Ak je nastavený `movingToGoal`, vykonáva sa nasledujúca logika pre navigáciu k cieľovým súradniciam (`targetX`, `targetY`): [cite: 23]

* Vypočíta sa požadovaný uhol k cieľu (`targetAngle`) pomocou funkcie `atan2`. [cite: 23]
* Vypočíta sa rozdiel medzi požadovaným uhlom a aktuálnou orientáciou robota (`errorAngle`). [cite: 24]
* Normalizuje sa `errorAngle` do rozsahu `[-180, 180]` pre zabezpečenie najkratšej otočky. [cite: 25]
* Vypočíta sa vzdialenosť k cieľu (`distance`). [cite: 25, 26, 27]
* Pri prvom volaní s novým cieľom (`distance_scan == false`) sa vypočíta celková vzdialenosť (`distance_all`) a nastaví sa `distance_scan` na `true`. [cite: 26, 27] Využije sa pri dynamickom menení tolerancie uhlu. [cite: 27]
* Vypočítajú sa požadované uhlové (`angular_speed`) a lineárne (`linear_speed`) rýchlosti pomocou proporcionálnych regulátorov s konštantami `Kp_angle` a `Kp_position`. [cite: 27, 28]
* Definuje sa tolerancia pre uhol (`tolerance_angle`) a pozíciu (`tolerance_pos`) pre zastavenie rotácie a dosiahnutie cieľa. [cite: 28, 29]
* Tolerancia uhla sa dynamicky mení v závislosti od vzdialenosti, akú robot prešiel. [cite: 29, 30, 31]
    * Na začiatku je tolerancia 1°. [cite: 30]
    * Následne sa zvýši na 5° (alebo inú aktuálne nastavenú hodnotu). [cite: 30] (Tieto hodnoty sme menili počas súťaže). [cite: 30]
    * Na konci dráhy sa robotu nastaví opäť nízka hodnota. [cite: 31]
* **Pôvodná reverzná logika:** Pôvodne bola implementovaná logika pre cúvanie robotu, ktorá prikázala robotu cúvať, ak bolo potrebné otočiť sa o viac ako 90 stupňov. [cite: 32, 33]
* **Limitovanie rýchlostí:** Vypočítané rýchlosti sa obmedzia na definované maximálne (`max_ot`, `max_s`) a minimálne (`min_ot`, `min_s`) hodnoty. [cite: 33, 34]
* **Akcelerácia a decelerácia:** Implementuje sa plynulá zmena aktuálnych lineárnych (`current_linear_speed`) a uhlových (`current_angular_speed`) rýchlostí pomocou definovaných hodnôt akcelerácie a decelerácie. [cite: 34, 35, 36, 37]
* **Riadenie pohybu:**
    * Ak je uhlová odchýlka väčšia ako `tolerance_angle`, robot rotuje na mieste, kým sa nezarovná s cieľom (`setSpeed(0, current_angular_speed)`). [cite: 35, 36, 37] Lineárna rýchlosť sa nastaví na nulu. [cite: 36]
    * Ak je robot dostatočne natočený a vzdialenosť k cieľu je väčšia ako `tolerance_pos`, robot sa pohybuje lineárne k cieľu (`setSpeed(current_linear_speed, 0)`). [cite: 36, 37]
    * Ak je vzdialenosť k cieľu menšia alebo rovná `tolerance_pos`, robot zastaví (`setSpeed(0, 0)`), nastaví sa `movingToGoal` na `false` a resetuje sa `distance_scan`. [cite: 37]
* **Publikovanie pozície:** Každých 5 iterácií (`datacounter % 5 == 0`) sa emituje signál `publishPosition` s aktuálnou polohou a orientáciou robota. [cite: 38, 39] Toto slúži na aktualizáciu používateľského rozhrania. [cite: 39]

## Záver

Tento kód implementuje základnú odometriu pre mobilného robota s diferenciálnym pohonom, využívajúc dáta z enkodérov a gyroskopu. [cite: 39, 40] Zároveň obsahuje implementáciu jednoduchého kontroléra pre pohyb robota k zadanému cieľu s plynulou akceleráciou a deceleráciou. [cite: 40]
