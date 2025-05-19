#   ČASŤ 1. ODOMETRIA

V prvej časti bolo potrebné naprogramovať odometriu robota a základný pohyb k nastavenému cieľu.

Pôvodne sme v kóde využívali odometriu z koliesok robota, ale po problémoch s inými časťami zadania sme sa rozhodli pre využitie gyroskopu.

Kód využíva dáta z enkodérov a gyroskopu robota na odhad jeho aktuálnej pozície a orientácie.

Tieto dáta sú poskytované užívateľovi priamo v ovládacom paneli robota (viď Obrázok 1). Cieľ, ku ktorému sa robot presúva, sa nastaví tlačidlom (Pri mapovaní je to tlačidlo „Mapovanie“ a pri prechádzaní mapy je to tlačidlo „Ďalší bod“).

![image](https://github.com/user-attachments/assets/1cf938ca-d955-4c71-9484-fa72ba9fd143)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*Obr. 1. Tlačidlá na ovládanie a zobrazenie dát o polohe robota* 
## Metódy

###   `void robot::moveToGoal(double goal_x, double goal_y)`

* Nastaví cieľové súradnice `targetX` a `targetY` pre robot.
* Nastaví `movingToGoal` na `true`, čím aktivuje logiku pohybu k cieľu v metóde `processThisRobot`.
* Resetuje `distance_scan` na `false`, aby sa pri prvom behu s novým cieľom prepočítala celková vzdialenosť.

###   `int robot::processThisRobot(TKobukiData robotdata)`

Toto je callback funkcia, ktorá sa volá vždy, keď prídu nové dáta z robota (enkodéry, gyroskop atď.). Implementuje hlavnú logiku odometrie a navigácie k cieľu.

####   Inicializácia:

Pri prvom spustení (`firstRun == true`) uloží aktuálne hodnoty enkodérov (`EncoderRight`, `EncoderLeft`) a gyroskopu (`GyroAngle`) do predošlých hodnôt a nastaví `firstRun` na `false`.

####   Konštanty:

Definuje konštanty súvisiace s geometriou robota a enkodérmi:

* `WHEEL_RADIUS`: Polomer kolesa.
* `WHEEL_BASE`: Rozchod kolies.
* `TICKS_PER_REV`: Počet impulzov enkodéra na jednu otáčku kolesa.
* `TWO_PI`: 2π.
* `MAX_ENCODER_VALUE`: Maximálna hodnota enkodéra (pre detekciu pretečenia).

####   Výpočet zmien enkodérov a gyroskopu:

Vypočíta zmenu hodnôt pravého a ľavého enkodéra (`deltaRight`, `deltaLeft`) a zmenu uhla z gyroskopu (`gyroFi`) od posledného merania. Uhol z gyroskopu sa konvertuje na radiány (`gyroFi_RAD`).

####   Ošetrenie pretečenia enkodérov:

Kontroluje a koriguje zmeny enkodérov, ak došlo k pretečeniu (wrap-around). Predpokladá sa 16-bitový enkodér.

####   Aktualizácia predošlých hodnôt:

Uloží aktuálne hodnoty enkodérov a gyroskopu pre ďalší výpočet.

####   Konverzia impulzov na uhlovú rotáciu kolies:

Premení zmenu impulzov enkodérov na uhlovú rotáciu kolies v radiánoch (`omegaRight`, `omegaLeft`).

####   Výpočet lineárnej a uhlovej rýchlosti robota:

* Lineárna rýchlosť `v` sa vypočíta ako priemer rýchlostí oboch kolies vynásobený polomerom kolesa.
* Uhlová rýchlosť `omega` sa získava priamo z gyroskopu (`gyroFi_RAD`). Pôvodné riešenie bolo cez kolieska.

####   Výpočet novej orientácie:

Aktualizuje aktuálnu orientáciu robota (`fi`) pridaním zistenej uhlovej zmeny (`omega`).

####   Výpočet novej pozície:

Aktualizuje súradnice robota (`x`, `y`) na základe vypočítanej lineárnej rýchlosti a zmeny orientácie pomocou diferenciálnych rovníc pre pohyb robota:

* Ak robot rotuje (`fabs(omega) > 1e-6`), použijú sa rovnice pre otáčanie na mieste.
* Ak sa robot pohybuje priamo (`fabs(omega) <= 1e-6`), použijú sa jednoduché rovnice pre priamy pohyb.

####   Normalizácia orientácie:

Uhol `fi` sa normalizuje do rozsahu `[-π, π]` pomocou funkcie `atan2(sin(newFi), cos(newFi))`, aby sa predišlo problémom s viacnásobnými reprezentáciami rovnakého uhla. Ak bol pôvodne v stupňoch, konvertuje sa späť na stupne.

####   Logika pohybu k cieľu (`if (movingToGoal)`):

Ak je nastavený `movingToGoal`, vykonáva sa nasledujúca logika pre navigáciu k cieľovým súradniciam (`targetX`, `targetY`):

* Vypočíta sa požadovaný uhol k cieľu (`targetAngle`) pomocou funkcie `atan2`.
* Vypočíta sa rozdiel medzi požadovaným uhlom a aktuálnou orientáciou robota (`errorAngle`).
* Normalizuje sa `errorAngle` do rozsahu `[-180, 180]` pre zabezpečenie najkratšej otočky.
* Vypočíta sa vzdialenosť k cieľu (`distance`).
* Pri prvom volaní s novým cieľom (`distance_scan == false`) sa vypočíta celková vzdialenosť (`distance_all`) a nastaví sa `distance_scan` na `true`. Využije sa pri dynamickom menení tolerancie uhlu.
* Vypočítajú sa požadované uhlové (`angular_speed`) a lineárne (`linear_speed`) rýchlosti pomocou proporcionálnych regulátorov s konštantami `Kp_angle` a `Kp_position`.
* Definuje sa tolerancia pre uhol (`tolerance_angle`) a pozíciu (`tolerance_pos`) pre zastavenie rotácie a dosiahnutie cieľa.
* Tolerancia uhla sa dynamicky mení v závislosti od vzdialenosti, akú robot prešiel.
    * Na začiatku je tolerancia 1°.
    * Následne sa zvýši na 5° (alebo inú aktuálne nastavenú hodnotu). (Tieto hodnoty sme menili počas súťaže).
    * Na konci dráhy sa robotu nastaví opäť nízka hodnota.
* **Pôvodná reverzná logika:** Pôvodne bola implementovaná logika pre cúvanie robotu, ktorá prikázala robotu cúvať, ak bolo potrebné otočiť sa o viac ako 90 stupňov.
* **Limitovanie rýchlostí:** Vypočítané rýchlosti sa obmedzia na definované maximálne (`max_ot`, `max_s`) a minimálne (`min_ot`, `min_s`) hodnoty.
* **Akcelerácia a decelerácia:** Implementuje sa plynulá zmena aktuálnych lineárnych (`current_linear_speed`) a uhlových (`current_angular_speed`) rýchlostí pomocou definovaných hodnôt akcelerácie a decelerácie.
* **Riadenie pohybu:**
    * Ak je uhlová odchýlka väčšia ako `tolerance_angle`, robot rotuje na mieste, kým sa nezarovná s cieľom (`setSpeed(0, current_angular_speed)`). Lineárna rýchlosť sa nastaví na nulu.
    * Ak je robot dostatočne natočený a vzdialenosť k cieľu je väčšia ako `tolerance_pos`, robot sa pohybuje lineárne k cieľu (`setSpeed(current_linear_speed, 0)`).
    * Ak je vzdialenosť k cieľu menšia alebo rovná `tolerance_pos`, robot zastaví (`setSpeed(0, 0)`), nastaví sa `movingToGoal` na `false` a resetuje sa `distance_scan`.
* **Publikovanie pozície:** Každých 5 iterácií (`datacounter % 5 == 0`) sa emituje signál `publishPosition` s aktuálnou polohou a orientáciou robota. Toto slúži na aktualizáciu používateľského rozhrania.

##   Záver

Tento kód implementuje základnú odometriu pre mobilného robota s diferenciálnym pohonom, využívajúc dáta z enkodérov a gyroskopu. Zároveň obsahuje implementáciu jednoduchého kontroléra pre pohyb robota k zadanému cieľu s plynulou akceleráciou a deceleráciou.
