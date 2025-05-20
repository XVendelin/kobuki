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

/*************************************************************************************************************************************************************************************************************************************************************************/
Nutné doplnenie
/*************************************************************************************************************************************************************************************************************************************************************************/
# ČASŤ 3. MAPOVANIE A INTERPOLÁCIA

Táto časť kódu sa zameriava na spracovanie dát z lidaru, ich interpoláciu a následné mapovanie prostredia do mapy. Mapa je realizovaná štvorcovou mriežkou (dvojrozmerné pole, to je neskôr transformované do vektora vektorov). Rozmer štvorca (bunky) je 10x10 cm. Mapa má nasledne nastavený rozmer 100x120 buniek (je to viac ako je potrebné s problematikou sa vysporiadavame vyplnením voľného miesta okolo mapy stenamy, teda 1-tkami). 

Ovládanie je nasledovné. V mainwindow.cpp máme nastavené body, ktorými má robot prejsť. Stláčaním tlačidla "Mapovanie" zadávame robotu novú pozíciu, kam sa má vydať a popri tom si tvorí mapu (zatiaľ sám pre seba). Po dosiahnutí bodu je nutné opätovne stlačiť tlačidlo "Mapovanie" na zadanie ďalšieho bodu. Priebežne si môžeme stĺačať tlačidlo "Vykreslenie Mapy", ktorým si už načítanú časť nielen vypíšeme do konzoly, ale si ju rovno aj uložíme do súboru.

Ak bol priestor už zmapovaný a nachádza sa vo forme mapy v súbore, tak môžeme stlačiť tlačidlo "Záplavový Algoritmus", ktoré nám načíta mapu transformuje ju do binárnej podoby. A tak bude pripravená na použitie práve už spomínaného záplavového algoritmu.

## Metódy

### `int robot::processThisLidar(LaserMeasurement laserData)`

Táto metóda je callback funkcia, ktorá sa volá vždy, keď sú prijaté nové dáta z lidaru. Jej hlavnou úlohou je spracovať tieto dáta a zaznamenať zistené prekážky do mapy.

* **Kopírovanie dát:** Dáta z `laserData` sa skopírujú do `copyOfLaserData`, aby sa predišlo prepísaniu počas spracovania.
* **Iterácia cez skeny lidaru:** Pre každý jednotlivý sken z lidaru (`copyOfLaserData.numberOfScans`):
    * **Výpočet uhla a vzdialenosti:** Načíta sa uhol (`uhol_lid`) a vzdialenosť (`vzdialenost_lid`) jednotlivého bodu z lidaru. Vzdialenosť sa konvertuje z centimetrov na metre.
    * **Normalizácia uhla lidaru:** Uhol lidaru sa normalizuje do rozsahu `[-180, 180]` stupňov.
    * **Filtrovanie dát:** Sú spracované len body, ktorých vzdialenosť spadá do určených rozsahov (`(vzdialenost_lid > 2 && vzdialenost_lid < 5) || (vzdialenost_lid > 7 && vzdialenost_lid < 30)`). Teda od 20 cm po 50 cm a od 70 cm po 300 cm. Je to spravené z dôvodu, že lidar na reálnom robote by nám vnášal do mapy chybné dáta. 
    * **Interpolácia pozície robota:** Na základe časovej pečiatky (`timestamp_lidar`) sa interpoluje presná pozícia (`interpolated_x`, `interpolated_y`) a orientácia (`interpolated_angle`) robota v čase merania lidaru. To je kľúčové pre presné mapovanie, pretože lidar meria asynchrónne s pohybom robota.
    * **Normalizácia interpolovaného uhla:** Interpolovaný uhol robota sa normalizuje do rozsahu `[-180, 180]` stupňov.
    * **Výpočet globálneho uhla bodu:** Globálny uhol sa vypočíta súčtom interpolovaného uhla robotu a uhla lidaru. Následne sa uhol normalizuje a konvertuje na radiány. Chýba tu síce ošetrenie kedy sa uhol interpoluje cez hranicu 180° a -180. To môže zavádzať šum do mapy. 
    * **Prepočet na súradnice mapy:** Súradnice prekážky (`mapX`, `mapY`) sa vypočítajú z interpolovanej pozície robota, vzdialenosti a globálneho uhla. Tieto súradnice sa následne pretypujú z float na integer a vykoná sa odsadenie počiatku v mriežke (`mapX_int`, `mapY_int`). odsadenie je `+10` (stĺpec) a `+50` (riadky).
    * **Kontrola rozmerov mapy:** Samozrejme je nutné hodnoty skontrolovať a zistiť či sa nachádzajú v mape (či sú v rámci hraníc mapy).
    * **Zápis do mapy:** Ak robot nerotuje (`rotacia == 0`), hodnota príslušnej bunky v 2D poli `Mapa` sa inkrementuje (`Mapa[mapY_int][mapX_int] += 1`). Ide o realizáciu váhovania prekážok. Ak lidar robota zaznamená prekážku pripíše danej bunke hodnotu. Čím viac krát je daná prekážka detegovaná tým je väčšia hodnota následne bude v dokumentácií spomenutá funkcia ktorá bude slúžiť ako filter podľa váh.
    * **Zaznamenanie počiatku:** Ak sú interpolované súradnice robota (0,0), bunka sa nastaví na 2, ide o naznačenie štartovacieho bodu robota ktoré slúžilo len na debugovanie a bude ofiltrovaná.

### `std::array<double, 3> robot::interpol(uint32_t Lid_TS)`

Táto funkcia vykonáva lineárnu interpoláciu pozície a orientácie robota na základe timestamp lidaru. Cieľom je určiť presnú polohu robota v momente, keď lidar zaznamenal konkrétne meranie.

* **Kontrola dostatočnosti dát:** Funkcia najprv skontroluje, či sú k dispozícii aspoň dve merania odometrie (v `Robot_TS`, `x_TS`, `y_TS`, `fi_TS`) na vykonanie interpolácie. Ak nie, vráti poslednú známu interpolovanú pozíciu (`interpol_X_backup`, `interpol_Y_backup`, `interpol_Fi_backup`).
* **Prehľadávanie queue:** Iteruje cez queue timestampu (`Robot_TS`) a zodpovedajúce pozície a orientáciu (`x_TS`, `y_TS`, `fi_TS`). Hľadá dvojicu timestampov (`Robot_1_TS`, `Robot_2_TS`), medzi ktorými leží timestamp lidaru (`Lid_TS`).
* **Výpočet interpolovaných hodnôt:** Ak sa nájde vhodná dvojica, vykoná sa lineárna interpolácia pre `x`, `y` a `fi`. Vzorec používa pomer časového rozdielu (`T_timestamp / T`) pre daný interval. Hodnoty `x` a `y` sa vynásobia 10 aby sa zabezpečila konverzia z metrov na decimetre.
* **Aktualizácia záloh x,y,fi:** Výsledné interpolované hodnoty sa uložia do záložných premenných (`interpol_X_backup`, `interpol_Y_backup`, `interpol_Fi_backup`) pre prípad, že by nebol dostatok dát na interpoláciu.
* **Vyhadzovanie starých a načítanie nových hodnôt z queue:** Po úspešnej interpolácii sa z frontov odoberú staršie hodnoty, aby sa zachovala ich veľkosť a relevancia.

### `void robot::vypisMapy()`

Ide o pomocnú funkciu, ktorá slúži na vizuálny výpis aktuálneho stavu mapy do konzoly.

* Funkcia prechádza cez 2D `Mapa`.
* Bunky s hodnotou menšou ako 10 toto je filter váh ktorý sme spomýnali vyššie. To sa zobrazí ako dve medzery ("  ").
* Bunky s hodnotou 2 počiatočná sa zobrazí ako `+`.
* Ostatné bunky (prekážky) sa zobrazia ako `.`.

### `void robot::saveMapToFile(const std::string &filename)`

Ukladá surovú (nefiltrovanú) mapu `Mapa` do textového súboru.

* Otvorí súbor pre zápis. Ak sa súbor nepodarí otvoriť, vypíše chybu.
* Prechádza všetky bunky mapy a zapíše ich hodnoty oddelené medzerou.
* Po každom riadku vloží nový riadok.
* Po zapísaní zatvorí súbor.

### `void robot::saveFilledMap(const std::string &filename)`

Ukladá upravenú mapu (v tomto prípade vektor vektorov) `map_na_fill` (je výsledkom spracovania z `openSavedMap`) do textového súboru. Funguje v princípe rovnako ako `saveMapToFile`.

### `void robot::openSavedMap(const std::string &filename)`

Načíta mapu z textového súboru a vykoná na nej transformáciu, aby ju pripravila pre algoritmy hľadania cesty (záplavový algoritmus).

* **Načítanie a binarizácia:**
    * Otvorí súbor a prečíta ho riadok po riadku.
    * Každá hodnota v riadku sa prečíta a **binarizuje**: Ak je hodnota `>= 10`, zapíše sa ako `1` (čiže prekážka), inak `0` (voľný priestor).
    * Binarizovaná mapa sa uloží do `temp_map` (vektor vektorov).
* **Spracovanie zľava a sprava (vyplňovanie nepotreného priestoru 1-tkami):**
    * Ide o vyplnenie prázdneho nepotrebného priestoru za "stenou", čím sa vytvorí súvislejší obraz stien.
    * Mapa je prechádzaná:
        * **Sprava:** Hľadá sa prvá `1` (prekážka) od pravého okraja. Všetky `0` pred touto `1` (smerom doprava) sa zmenia na `1`.
        * **Zľava:** Hľadá sa prvá `1` (prekážka) od ľavého okraja. Všetky `0` pred touto `1` (smerom doľava) sa zmenia na `1`.
* **Rozšírenie jednotiek do okolia (dilatácia):**
    * Cieľom je rozšíriť detegované prekážky (hodnoty `1`) do ich okolia. Toto je dôležité pre vytvorenie "bezpečnostnej zóny" okolo prekážok, aby sa robot do nich nenarážal.
    * Pre každý bod `(y, x)` v `temp_map`, ak je `1` (prekážka):
        * Pre všetky okolité bunky v rozsahu 2 sa nastavia na `1` v `expanded_map`. To znamená, že ak je prekážka v `(x,y)`, bunky v okruhu 2 buniek okolo nej sa tiež označia ako prekážky. Rozsah 2 z dôvodu že robot má polomer približne 17cm a jedna naša bunka má veľkosť 10x10 cm.
* Konečná upravená mapa (`expanded_map`) sa uloží do `map_na_fill`. Takto sa spravil v podstate konverzia z 2D pola na vektor vektorov. Táto zmena sa udiala alebo v Časti 4. pracujeme práve s vektorom vektorov. Mapa sa následne vypíše do konzoly už v binárnej forme.

/_._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._._/
este nekontrolovane

# ČASť 4. PLÁNOVANIE CESTY

v 4. časti sa implementuje algoritmus pre plánovanie cesty robota v mape. Využívame záplavový algoritmus a následne po zaplavení mapy vytvárame cesty potažmo body do ktorých sa robot musí dostať. Následné prejdenie zo štartovacieho bodu `0;0` do bodu určenia. Body sa vypočítajú stlačením tlačidla "Výpočet bodov prechodu" a mapa sa prechádza stláčaním tlačidla "Ďalší bod". Aktuálny bod do ktorého sa robot presúva sa vypíše do konzoly.  

## Metódy

### `std::vector<std::vector<int>> robot::gradualFill(std::vector<std::vector<int>> map, Point start, Point goal)`

Táto funkcia implementuje záplavový algoritmus, ktorý vypĺňa mapu hodnotami predstavujúcimi vzdialenosť od cieľového bodu. Tým sa vytvorí gradient, po ktorom môže algoritmus hľadania cesty nasledovať. Poznámka ako štartovací bod pre gradualFill je v podstate cieľový bod, pretože mapu začíname zaplavovať od cieľa.

* **Inicializácia:**
    * `q`: queue pre záplavový algoritmus, inicializovaná štartovým bodom.
    * `currentValue`: Hodnota, ktorá sa bude postupne priraďovať bunkám mapy, začína na 3.
    * `WIDTH`, `HEIGHT`: Rozmery mapy.
    * `dx`, `dy`: Polia pre smery pohybu (hore, doprava, dole, doľava).
* **Slučka pre záplavový algoritmus:**
    * Kým queue `q` nie je prázdna:
        * Pre každú bunku na aktuálnej úrovni:
            * Vyberie sa aktuálna bunka z fronty.
            * **Kontrola susedov:** Pre každého zo 4 susedov (pomocou `dx`, `dy`):
                * Ak je sused mimo mapy, preskočí sa.
                * Ak je susedom cieľový bod (`map[nx][ny] == -1`), cieľ je pridaný do `checkpoints` - (cesta nájdená).
                * Ak je susedom voľné miesto (`map[nx][ny] == 0`), priradí sa mu `currentValue`, pridá sa do fronty a pokračuje sa vo vyhľadávaní.
* **Checkpoints:** `checkpoints.push_back(q.back());` zaznamenáva posledný bod každej úrovne záplavového algoritmu ako checkpoint, čo sa využíva pre plánovanie cesty.
* Hodnota `currentValue` sa inkrementuje po každej spracovanej úrovni.

### `std::vector<Point> robot::findpath(const std::vector<std::vector<int>>& matrix, Point start, Point end)`

Táto funkcia rekonštruuje cestu z vyplnenej mapy (`matrix`) z cieľového bodu (`end`) späť k štartovému bodu (`start`). V `matrix` sa nachádza vyplnená mapa so vzdialenosťami od cieľa.

* **Inicializácia:**
    * `positions`: Vektor pre ukladanie bodov nájdenej cesty.
    * `current`: Aktuálny bod na ceste, inicializovaný štartovým bodom.
    * `directions`: Možné smery pohybu (ľavá, pravá, hore, dole).
    * `bestMove`: Najlepší ďalší krok na ceste.
    * `minValue`: Minimálna hodnota, ktorá sa má hľadať v susedných bunkách (inicializovaná na maximálnu možnú hodnotu).
* **Prehľadávanie cesty:**
    * Slučka beží, kým sa aktuálny bod nerovná cieľovému bodu.
    * Pre každý smer (`dir`):
        * Vypočítajú sa súradnice susednej bunky (`newX`, `newY`).
        * Ak je sused v rámci mapy:
            * Načíta sa hodnota suseda (`neighborValue`).
            * Ak je sused prekážkou (`1` alebo `-1`) alebo nulou (`0`), priradí sa mu veľmi vysoká hodnota (`minValue + 1`), aby sa zabránilo výberu.
            * Ak je hodnota suseda menšia ako `minValue` (čo znamená, že je bližšie k cieľu v gradiente vyplnenej mapy), táto bunka sa stane `bestMove` a jej hodnota sa stane `minValue`.
    * Aktuálny bod (`current`) sa aktualizuje na `bestMove` a `bestMove` sa pridá do `positions`.

### `std::vector<Point> robot::r_checkpoint(const std::vector<Point>& points)`

Táto funkcia zoberie sekvenciu bodov a zredukuje ju na "body prechodu" (checkpoints), ktoré reprezentujú zmeny smeru v trajektórii.

* **Logika:** Iteruje sa cez body cesty. Ak sa zmení smer medzi aktuálnym bodom a predchádzajúcim bodom, aktuálny bod sa pridá do `checkpoints`.
* Posledný bod pôvodnej cesty sa vždy pridá do `checkpoints`.

### `std::vector<Point> robot::volaj_findpath(Point start, Point goal)`

Táto funkcia slúži na externé volanie volanie všetkých častí algoritmu hľadania cesty. Vracia body ,ktorými má robot prejsť. V mainwindow.cpp sa body štart a cieľ konvertujú
z reálnych rozmerov v metroch na súradnice mapy. Po zísakní bodov prechodu, musíme vykonať opačný proces, čiže zo súradníc mapy do súradníc reálneho sveta. Následne stláčaním tlačidla "Ďalši bod" sa dokáže robot presunúť do cieľa po získaných bodoch.

![image](https://github.com/user-attachments/assets/967f6e5e-079f-4c09-9506-26488d579a9b)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*Obr. XX. konverzia štartu a cieľa na súranice mapy* 

![image](https://github.com/user-attachments/assets/664b1485-2537-402f-b620-193e99f4ebba)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;*Obr. XX. konverzia bod po bode prechodu na reálne jednotky* 

* **Nastavenie štartu a cieľa:**
    * Štartový bod (`start`) sa v mape `map_na_fill` označí ako `-1`.
    * Cieľový bod (`goal`) sa v mape `map_na_fill` označí ako `2`.
* **Vyplnenie mapy:** Volá sa `gradualFill` na vyplnenie mapy
* **Uloženie vyplnenej mapy:** Vyplnená mapa sa uloží do súboru "mapa_fill.txt".
* **Nájdete cestu:** Volá sa `findpath` na nájdenie cesty.
* **Redukcia na body prechodu:** Na nájdenú cestu sa aplikuje `r_checkpoint`, ktorý ju zjednoduší na body prechodu.
* Výsledné kontrolné body sa vracajú.
/*************************************************************************************************************************************************************************************************************************************************************************/
