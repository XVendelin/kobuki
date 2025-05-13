#include "robot.h"
#include <cmath>
#include <iostream>
#include <queue>


std::queue<uint32_t> Robot_TS;
std::queue<uint32_t> Robot_TS_1;
std::queue<uint32_t> Robot_TS_2;
std::queue<uint32_t> Robot_TS_3;
std::queue<double> x_TS;
std::queue<double> y_TS;
std::queue<double> fi_TS;
double interpol_X_backup=0;
double interpol_Y_backup=0;
double interpol_Fi_backup=0;

uint32_t timestamp_POM=0;
uint32_t timestamp_POM_1=0;
uint32_t timestamp_POM_2=0;
uint32_t timestamp_POM_3=0;
double x_TS_POM=0;
double y_TS_POM=0;
double fi_TS_POM=0;
double rotacia = 0;

std::vector<std::vector<int>> map_na_fill; // vektor pre uloženie upravenej mapy

//struct Point {int x, y;};


robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
    #endif
    #ifndef DISABLE_SKELETON
    qRegisterMetaType<skeleton>("skeleton");
    #endif
}

void robot::initAndStartRobot(std::string ipaddress)
{

    forwardspeed=0;
    rotationspeed=0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&robot::processThisLidar,this,std::placeholders::_1));
    robotCom.setRobotParameters(ipaddress,53000,5300,std::bind(&robot::processThisRobot,this,std::placeholders::_1));
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&robot::processThisCamera,this,std::placeholders::_1));
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters("127.0.0.1",23432,23432,std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}


void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane

bool movingToGoal;
float targetY, targetX;
bool distance_scan = false; // ak false, tak sa zistí celkova vzdialenosť, ktoru ma robot prejst
float distance_all;//celkova vzdialenost


void robot::moveToGoal(double goal_x, double goal_y) {
    targetX = goal_x;
    targetY = goal_y;
    movingToGoal = true;  // Flag to activate movement
}

int robot::processThisRobot(TKobukiData robotdata)
{


    ///tu mozete robit s datami z robota
    #define DEG_TO_RAD (M_PI / 180.0)  // Convert degrees to radians
    #define RAD_TO_DEG (180.0 / M_PI)  // Convert radians to degrees
    double max_ot=1, min_ot=0.3, max_s=500, min_s=30;


    // If this is the first run, store encoder values and exit
    if (firstRun) {
        prevEncoderRight = robotdata.EncoderRight;
        prevEncoderLeft = robotdata.EncoderLeft;
        prevEncoderGyro = robotdata.GyroAngle/100;//data z gyra
        firstRun = false;
        return 0;
    }

    // Constants
    const double WHEEL_RADIUS = 0.035;  // 35 mm
    const double WHEEL_BASE = 0.23;     // 230 mm
    const double TICKS_PER_REV = 52.0*49.5833;  // Encoder ticks per revolution
    const double TWO_PI = 2 * M_PI;     // Constant for full revolution
    const int MAX_ENCODER_VALUE = 65536; // Adjust for your encoder type

    // Compute wheel displacements (difference in encoder ticks)
    int deltaRight = robotdata.EncoderRight - prevEncoderRight;
    int deltaLeft = robotdata.EncoderLeft - prevEncoderLeft;
    double gyroFi = robotdata.GyroAngle/100 - prevEncoderGyro;
    //std::cout << "gyro uhol: "<<gyroFi <<endl;
    double gyroFi_RAD = gyroFi * DEG_TO_RAD;
    //std::cout << "gyro uhol RAD: "<< gyroFi_RAD << endl;

    // Handle encoder wrap-around (assumes 16-bit encoder, adjust if needed)
    if (deltaRight > MAX_ENCODER_VALUE / 2) deltaRight -= MAX_ENCODER_VALUE;
    if (deltaRight < -MAX_ENCODER_VALUE / 2) deltaRight += MAX_ENCODER_VALUE;
    if (deltaLeft > MAX_ENCODER_VALUE / 2) deltaLeft -= MAX_ENCODER_VALUE;
    if (deltaLeft < -MAX_ENCODER_VALUE / 2) deltaLeft += MAX_ENCODER_VALUE;

    // Store previous encoder values
    prevEncoderRight = robotdata.EncoderRight;
    prevEncoderLeft = robotdata.EncoderLeft;
    prevEncoderGyro = robotdata.GyroAngle/100;

    // Convert encoder ticks to wheel rotation (radians)
    double omegaRight = (deltaRight / TICKS_PER_REV) * TWO_PI;  // Now in radians
    double omegaLeft = (deltaLeft / TICKS_PER_REV) * TWO_PI;    // Now in radians

    // Compute linear and angular velocity
    double v = ((omegaLeft + omegaRight) * WHEEL_RADIUS) / 2.0;
    //double omega = ((omegaRight - omegaLeft) * WHEEL_RADIUS) / WHEEL_BASE;
    double omega = gyroFi_RAD;
// Ensure correct sign

    // Convert `fi` to radians if it was stored in degrees
    bool fiInDegrees = true;  // Set to true if `fi` is originally in degrees
    if (fiInDegrees) {
        fi *= DEG_TO_RAD;  // Convert degrees to radians
    }
    // Compute new orientation
    double newFi = fi + omega;

    // Compute new position using differential drive equations
    if (fabs(omega) > 1e-6) { // Robot is turning
        double R = v / omega; // Turning radius
        x += R * (sin(newFi) - sin(fi));
        y -= R * (cos(newFi) - cos(fi));
    } else { // Robot is moving straight
        x += v * cos(fi);
        y += v * sin(fi);
    }

    // Update orientation and normalize to [-π, π]
    fi = atan2(sin(newFi), cos(newFi));  // Keeps angle within [-π, π]
    //std::cout <<"NFi " << newFi <<", FI "<< fi <<endl;
    // Convert `fi` back to degrees if needed
    if (fiInDegrees) {
        fi *= RAD_TO_DEG;
    }



    if (movingToGoal) {
        double targetAngle = atan2(targetY - y, targetX - x) * RAD_TO_DEG;
        double errorAngle = targetAngle - fi;

        // Normalize errorAngle to [-180, 180] to always take the shortest turn
        errorAngle = fmod(errorAngle + 180, 360);
        if (errorAngle < 0) errorAngle += 360;
        errorAngle -= 180;


        double distance = std::hypot(targetX - x, targetY - y);

        if (distance_scan == false){
            distance_all = std::hypot(targetX - x, targetY - y);
            std::cout << "Hello, World!" << std::endl;
            distance_scan = true;
        }
        //double distance = std::hypot(targetX - x, targetY - y);

        double Kp_angle = 0.1;  // Adjust as needed
        double Kp_position = 300;
        double tolerance_angle;  // Small threshold for angle alignment
        double tolerance_pos = 0.05;   // Stop threshold (meters)

        double angular_speed = Kp_angle * errorAngle;
        double linear_speed = Kp_position * distance;

        //tolerancia uhlov

       // std::cout << "Dis: " <<distance<< std::endl;

        if ((distance > (0.99)*distance_all)) tolerance_angle = 1;
        else if ((distance <= (0.99)*distance_all)&&(distance > (0.05)*distance_all)) tolerance_angle = 10;
        else tolerance_angle = 1;
    //std::cout << "tolerance: " <<tolerance_angle<< std::endl;

        //if (distance>0.5 && distance<2) tolerance_angle = 45;
        //else tolerance_angle = 1;
        //tolerancia uhlov



        // **Reverse Logic: Move backward if turning more than 90° is needed**
        /*if (fabs(errorAngle) > 90) {
            if (errorAngle>90) errorAngle-=180;
            else errorAngle+=180;
            angular_speed = Kp_angle * errorAngle;
            linear_speed = -linear_speed;  // Reverse movement

        }*///odstavil som to uvidime co sa stane

        // Limit speeds
        if (angular_speed > max_ot) angular_speed = max_ot;
        else if (angular_speed < min_ot && angular_speed > 0) angular_speed = min_ot;
        if (angular_speed < -max_ot) angular_speed = -max_ot;
        else if (angular_speed < -min_ot && angular_speed < 0) angular_speed = -min_ot;

        if (linear_speed > max_s) linear_speed = max_s;
        else if (linear_speed < min_s && linear_speed > 0) linear_speed = min_s;
        if (linear_speed < -max_s) linear_speed = -max_s;
        else if (linear_speed > -min_s && linear_speed < 0) linear_speed = -min_s;

        // Define acceleration and deceleration rates
        double acceleration_linear = 10;   // How fast it speeds up
        double deceleration_linear = 10;   // How fast it slows down

        double acceleration_angular = 0.1;  // Angular acceleration
        double deceleration_angular = 0.1;  // Angular deceleration

        static double current_linear_speed = 0;
        static double current_angular_speed = 0;

        if (fabs(errorAngle) > tolerance_angle) {
            linear_speed=0;
            rotacia = 1;
            if (fabs(current_linear_speed)>0+min_s){
                if (current_linear_speed < linear_speed) {
                    current_linear_speed += acceleration_linear;
                    if (current_linear_speed > linear_speed) current_linear_speed = linear_speed;
                } else {
                    current_linear_speed -= deceleration_linear;
                    if (current_linear_speed < linear_speed) current_linear_speed = linear_speed;
                }

                setSpeed(current_linear_speed, 0);  // Move forward or backward
            }
            else{
                if (current_angular_speed < angular_speed) {
                    current_angular_speed += acceleration_angular;
                    if (current_angular_speed > angular_speed) current_angular_speed = angular_speed;
                } else {
                    current_angular_speed -= deceleration_angular;
                    if (current_angular_speed < angular_speed) current_angular_speed = angular_speed;
                }
                setSpeed(0, current_angular_speed);  // Rotate towards goal
                rotacia = 1;
            }


        }
        else if (distance > tolerance_pos) {
            rotacia = 0;
            // Linear speed ramp-up & ramp-down
            if (current_linear_speed < linear_speed) {
                current_linear_speed += acceleration_linear;
                if (current_linear_speed > linear_speed) current_linear_speed = linear_speed;
            } else {
                current_linear_speed -= deceleration_linear;
                if (current_linear_speed < linear_speed) current_linear_speed = linear_speed;
            }

            setSpeed(current_linear_speed, 0);  // Move forward or backward
        }
        else {
            rotacia = 0;
            setSpeed(0, 0);  // Stop completely
            movingToGoal = false;  // Goal reached
            distance_scan = false; // aby mohol zase skenovat pre zadanau vzdialenost
        }
    }

    x_TS.push(x); //push pridanie na koniec, pop vybratie a vyhodenie, front precitanie hodnoty
    y_TS.push(y);
    fi_TS.push(fi);
    Robot_TS.push(robotdata.synctimestamp);
    Robot_TS_1.push(robotdata.synctimestamp);
    Robot_TS_2.push(robotdata.synctimestamp);
    Robot_TS_3.push(robotdata.synctimestamp);

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(x,y,fi);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru


// Rozmery mapy (6000 x 5000 grid, 0.1m na bunku)
const int MAP_WIDTH = 120;     //6m
const int MAP_HEIGHT = 100;    //5m
const double CELL_SIZE = 1;   // 10 cm
static int Mapa[MAP_WIDTH][MAP_HEIGHT] = {0};



int robot::processThisLidar(LaserMeasurement laserData){
    memcpy(&copyOfLaserData, &laserData, sizeof(LaserMeasurement));
    //copyof lebo laserdata sa prepisuju
    // Spracovanie dát z lidaru a zápis do štvorcovej (mriežka) mapy
    //if (rotacia == 0){
    for (int i = 0; i < copyOfLaserData.numberOfScans; i++) {
        double uhol_lid = -copyOfLaserData.Data[i].scanAngle;       // uhol lidaru
        double vzdialenost_lid = copyOfLaserData.Data[i].scanDistance / 100.0; // vzdialenosť bodu v metroch (cm -> m)

        // Normalizovať uhol_lid na [-180, 180]
        uhol_lid = fmod(uhol_lid + 180, 360);
        if (uhol_lid < 0)
            uhol_lid += 360;
        uhol_lid -= 180;

        //std::cout << "uhol_lid = " << uhol_lid << endl;
        uint32_t timestamp_lidar = copyOfLaserData.Data[i].timestamp; // Časová značka bodu lidaru

        //std::cout <<vzdialenost_lid << "\n";
        ////TODO: sparviť od 15cm do 50 a od 70 do metra
        if ((vzdialenost_lid > 2 && vzdialenost_lid < 5) || (vzdialenost_lid > 7 && vzdialenost_lid < 30)) {
            std::array<double, 3> interpol_pozicia = interpol(timestamp_lidar);  // [interpol_X, interpol_Y, interpol_Fi]
            //double interpolated_x = interpol_X(timestamp_lidar);
            //double interpolated_y = interpol_Y(timestamp_lidar);
            //double interpolated_angle = interpol_Fi(timestamp_lidar);

            double interpolated_x = interpol_pozicia[0];
            double interpolated_y = interpol_pozicia[1];
            double interpolated_angle = interpol_pozicia[2];

            interpolated_angle = fmod(interpolated_angle + 180, 360);
            if (interpolated_angle < 0)
                interpolated_angle += 360;
            interpolated_angle -= 180;


            //double global_angle = Fi + uhol_lid * M_PI / 180 ;//
            //double interpolated_angle_RAD = interpolated_angle* M_PI/180;
            //std::cout << "interpolated v radianoch = " << interpolated_angle_RAD << endl;
            //uhol_lid = uhol_lid * M_PI/180;
            double global_angle = interpolated_angle + uhol_lid ;//žeby trebalo ošetriť toto???

            global_angle = fmod(global_angle + 180, 360);
            if (global_angle < 0)
                global_angle += 360;
            global_angle -= 180;

            //std::cout << "global_angle = " << global_angle << endl;
            global_angle = global_angle*M_PI/180;
            // Prepočet súradníc na mriežku
            //if (rotacia==0){
           // double mapX = (interpolated_x + vzdialenost_lid * cos(global_angle)) / CELL_SIZE;//normalizácia uhla
           // double mapY = (interpolated_y + vzdialenost_lid * sin(global_angle)) / CELL_SIZE;
            double mapX = (interpolated_x + vzdialenost_lid * cos(global_angle)) / CELL_SIZE;//normalizácia uhla
            double mapY = (interpolated_y + vzdialenost_lid * sin(global_angle)) / CELL_SIZE;
            int mapX_int = static_cast<int>(std::round(mapX) + 10);
            int mapY_int = static_cast<int>(std::round(-mapY) + 50);
            //std::cout << mapX_int << " " << mapY_int;
            if (interpolated_x == 0 && interpolated_y == 0){
                Mapa[static_cast<int>(std::round(-interpolated_y) + 50)][static_cast<int>(std::round(interpolated_x) + 10)] = 2;
            }
            //std::cout <<"  int_X:"<< interpolated_x << "  vzdialnost*cos(glob):" << vzdialenost_lid * cos(global_angle) << "  celkove:" << interpolated_x + vzdialenost_lid * cos(global_angle)  << "  X do mapy:" << mapX_int  <<  " \n";
            // Kontrola, či sú súradnice v rozsahu mapy
            if (mapX_int >= 0 && mapX_int < MAP_WIDTH && mapY_int >= 0 && mapY_int < MAP_HEIGHT) {
                //if (Mapa[mapY_int][mapX_int] == 0 && rotacia == 0) {
                if (rotacia == 0){
                    Mapa[mapY_int][mapX_int] += 1;
                   // std::cout << mapX_int << " " << mapY_int;
                }
            }
            //else
                //std::cout << mapX_int << " " << mapY_int << " robi blbosti?";
        }
    }//}

    //vypisMapy();
    // Tu môžete robiť s dátami z lidaru ďalšie spracovanie, napríklad nájsť prekážky,
    // zapísať do mapy alebo naplánovať, ako sa prekážke vyhnúť.
    emit publishLidar(copyOfLaserData); // Publikovanie dát pre vizualizáciu
    return 0;
}

std::array<double, 3> robot::interpol(uint32_t Lid_TS) {
    // Skontroluj, že máme aspoň 2 hodnoty na interpoláciu
    if (Robot_TS.size() < 2 || x_TS.size() < 2 || y_TS.size() < 2 || fi_TS.size() < 2) {
        //std::cout << "nedostatok hodnot";
       //return {0.0, 0.0, 0.0};  // Alebo throw, alebo iný spôsob ošetrenia
        return {interpol_X_backup, interpol_Y_backup, interpol_Fi_backup};  
    }

    //std::queue<uint32_t> ts_queue = Robot_TS;
    //std::queue<double> x_queue = x_TS;
    //std::queue<double> y_queue = y_TS;
    //std::queue<double> fi_queue = fi_TS;

    uint32_t Robot_1_TS = timestamp_POM;
    double x_1 = x_TS_POM;
    double y_1 = y_TS_POM;
    double fi_1 = fi_TS_POM;

    fi_1 = fi_1;


    //std::cout << x_1 << "    " << y_1 << "    " << fi_1 << " r " << x << "    " << y << "    " << fi << "\n";

    // Hľadáme dvojicu TS medzi ktorou je lidarový timestamp
    while (!Robot_TS.empty()) {
        uint32_t Robot_2_TS = Robot_TS.front();
        double x_2 = x_TS.front();
        double y_2 = y_TS.front();
        double fi_2 = fi_TS.front();
        //x_2 = x_2;
        //y_2 = y_2;
        fi_2 = fi_2;


        if (Robot_1_TS <= Lid_TS && Lid_TS < Robot_2_TS) {
            // Interpolácia
            uint32_t T = Robot_2_TS - Robot_1_TS;
            //if (T == 0) return {x_1, y_1, fi_1}; // fallback pre rovnaké timestamps

            double XX = (x_2-x_1)*10; //T_timestamp/T a priratat x_1
            double YY = (y_2-y_1)*10;
            double fifi = (fi_2-fi_1);

            uint32_t T_timestamp = Lid_TS - Robot_1_TS;

            double T_Tt = static_cast<double>(T_timestamp)/static_cast<double>(T);
            double interpol_X = T_Tt * XX;
            double interpol_Y = T_Tt * YY;
            double interpol_Fi = T_Tt * fifi;
            interpol_X = x_1*10 + interpol_X;
            //std::cout << fixed << setprecision(6); // nastaví 6 desatinných miest
            //std::cout << "x_1 = " << x_1 << endl; // Vypíše: T_Tt = 0.655583
            //std::cout << "interpol_x = " << interpol_X << endl;

            interpol_Y = y_1*10 + interpol_Y;
            //std::cout << fixed << setprecision(6);
           // std::cout << "y_1 = " << x_1 << endl;
            //std::cout << "interpol_y = " << interpol_X << endl;

            interpol_Fi = fi_1 + interpol_Fi;//poriešiť treba todo.................................
            //interpol_Fi = interpol_Fi*M_PI/180;
            //std::cout << fixed << setprecision(6); // nastaví 6 desatinných miest
            //std::cout << "fi_1 = " << fi_1 << endl; // Vypíše: T_Tt = 0.655583
           // std::cout << "interpol_fi = " << interpol_Fi << endl;

            //std::cout << "T = " << T << ", T_timestamp = " << T_timestamp << "," ;
            //std::cout << fixed << setprecision(6); // nastaví 6 desatinných miest
            //std::cout << "T_Tt = " << T_Tt << endl; // Vypíše: T_Tt = 0.655583
            interpol_X_backup = interpol_X;
            interpol_Y_backup = interpol_Y;
            interpol_Fi_backup = interpol_Fi;

            //return{x_1, y_1, fi_1};

            return {interpol_X, interpol_Y, interpol_Fi};
        }

        // Posuň sa o jednu dopredu
        Robot_1_TS = Robot_2_TS;
        timestamp_POM = Robot_2_TS;
        Robot_TS.pop();
        x_1 = x_2;
        x_TS_POM = x_2;
        x_TS.pop();
        y_1 = y_2;
        y_TS_POM = y_2;
        y_TS.pop();
        fi_1 = fi_2;
        fi_TS_POM = fi_2;
        fi_TS.pop();

        //std::cout <<" x1:"<< x_1 << " y1:" << y_1 << " fi1" << fi_1 << " z "<<" x:" << x << " y:" << y << " fi: " << fi << "\n";
    }
    //return{0,0,0};

    // Ak sa nenašla vhodná dvojica na interpoláciu
    return {interpol_X_backup, interpol_Y_backup, interpol_Fi_backup}; // posledná známa pozícia
}

void robot::vypisMapy(){
    for (int i = 0; i < 120; ++i) {
        for (int j = 0; j < 100; ++j) {
            if (Mapa[i][j] < 10)
                std::cout << " " << " ";
            else if (Mapa[i][j] == 2)
                std::cout << "+" << " ";
            else
                //std::cout << Mapa[i][j];
                std::cout << "."<< " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl<<std::endl;
}

void robot::saveMapToFile(const std::string &filename) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }
    for (const auto &row : Mapa) {
        for (int cell : row) {
            file << cell <<" ";
        }
        file << '\n';
    }
    std::cout << "vypisane" << endl;
    file.close();
}


void robot::saveFilledMap(const std::string &filename) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }
    for (const auto &row : map_na_fill) {
        for (int cell : row) {
            file << cell <<" ";
        }
        file << '\n';
    }
    std::cout << "vypisane" << endl;
    file.close();
}


/*void robot::openSavedMap(const std::string &filename) {
    std::ifstream file(filename); // otvorí súbor na čítanie
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::cout << line << std::endl; // vypíše celý riadok
    }
    file.close();
}*/

/*void robot::openSavedMap(const std::string &filename) {
    std::ifstream file(filename); // otvorí súbor na čítanie
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    std::string line;
    std::vector<std::vector<int>> map; // vektor pre uloženie mapy

    while (std::getline(file, line)) {
        std::vector<int> row; // dočasný vektor pre jeden riadok
        std::stringstream ss(line);
        int value;

        // Načítanie hodnôt zo všetkých častí riadku
        while (ss >> value) {
            row.push_back(value);
        }

        map.push_back(row); // pridanie riadku do mapy
    }

    file.close();

    // Ak chcete vypísať načítanú mapu pre kontrolu:
    for (const auto &row : map) {
        for (int val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}*/

/*void robot::openSavedMap(const std::string &filename) {
    std::ifstream file(filename); // otvorí súbor na čítanie
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    map_na_fill.clear(); // vymažeme starú mapu pred načítaním novej

    std::string line;
    while (std::getline(file, line)) {
        std::vector<int> row;
        std::stringstream ss(line);
        int value;

        // Načítanie riadku do vektora
        while (ss >> value) {
            row.push_back(value);
        }

        // Spracovanie sprava: všetky 0 → 1 až po prvú 1
        bool foundRightOne = false;
        for (int i = static_cast<int>(row.size()) - 1; i >= 0; --i) {
            if (!foundRightOne && row[i] == 1) {
                foundRightOne = true;
            }
            if (!foundRightOne && row[i] == 0) {
                row[i] = 1;
            }
        }

        // Spracovanie zľava: všetky 0 → 1 až po prvú 1
        bool foundLeftOne = false;
        for (int i = 0; i < row.size(); ++i) {
            if (!foundLeftOne && row[i] == 1) {
                foundLeftOne = true;
            }
            if (!foundLeftOne && row[i] == 0) {
                row[i] = 1;
            }
        }

        map_na_fill.push_back(row); // uloženie spracovaného riadku
    }

    file.close();

    // Výpis mapy pre kontrolu (voliteľné)
    for (const auto &row : map_na_fill) {
        for (int val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}*/
void robot::openSavedMap(const std::string &filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    map_na_fill.clear();

    std::string line;
    std::vector<std::vector<int>> temp_map;

    // 1. Načítanie a binarizácia
    while (std::getline(file, line)) {
        std::vector<int> row;
        std::stringstream ss(line);
        int value;

        while (ss >> value) {
            row.push_back(value >= 10 ? 1 : 0);  // >=10 → 1, inak 0 //ternárny operator
        }

        temp_map.push_back(row);
    }

    file.close();

    // 2. Spracovanie zľava a sprava
    for (auto& row : temp_map) {
        // Sprava
        bool foundRightOne = false;
        for (int i = static_cast<int>(row.size()) - 1; i >= 0; --i) {
            if (!foundRightOne && row[i] == 1) {
                foundRightOne = true;
            }
            if (!foundRightOne && row[i] == 0) {
                row[i] = 1;
            }
        }

        // Zľava
        bool foundLeftOne = false;
        for (int i = 0; i < row.size(); ++i) {
            if (!foundLeftOne && row[i] == 1) {
                foundLeftOne = true;
            }
            if (!foundLeftOne && row[i] == 0) {
                row[i] = 1;
            }
        }
    }

    // 3. Rozšírenie jednotiek do okolia (vzdialenosť 2 = 5x5 blok)
    int rows = temp_map.size();
    int cols = rows > 0 ? temp_map[0].size() : 0;
    std::vector<std::vector<int>> expanded_map = temp_map;

    const int spread = 2; // vzdialenosť rozšírenia

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (temp_map[y][x] == 1) {
                for (int dy = -spread; dy <= spread; ++dy) {
                    for (int dx = -spread; dx <= spread; ++dx) {
                        int ny = y + dy;
                        int nx = x + dx;
                        if (ny >= 0 && ny < rows && nx >= 0 && nx < cols) {
                            expanded_map[ny][nx] = 1;
                        }
                    }
                }
            }
        }
    }

    map_na_fill = expanded_map;

    // Výpis (voliteľné)
    for (const auto &row : map_na_fill) {
        for (int val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}



////zadanie 4
std::vector<std::vector<int>> robot::gradualFill(std::vector<std::vector<int>> map, Point start, Point goal) {
    std::vector<Point> checkpoints;
    std::queue<Point> q;
    q.push(start);
    int currentValue = 3;
    int WIDTH = map[0].size();
    int HEIGHT = map.size();

    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};

    while (!q.empty()) {
        int size = q.size();
        for (int i = 0; i < size; ++i) {
            Point current = q.front();
            q.pop();

            for (int j = 0; j < 4; ++j) {
                int nx = current.x + dx[j];
                int ny = current.y + dy[j];

                if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT) {
                    if (map[nx][ny] == -1) {
                        checkpoints.push_back(goal);
                        return map; // Stop when reaching goal
                    }
                    if (map[nx][ny] == 0) { // Ensure obstacles remain unchanged
                        map[nx][ny] = currentValue;
                        q.push({nx, ny});
                    }
                }
            }
        }
        checkpoints.push_back(q.back()); // Store a checkpoint
        currentValue++;
    }
    return map;
}

std::vector<Point> robot::findpath(const std::vector<std::vector<int>>& matrix, Point start, Point end) {
    std::vector<Point> positions;
    // Extracting start and end coordinates
    Point current=start;
    int startRow = start.x, startCol = start.y;
    int endRow = end.x, endCol = end.y;

    // Ensure valid boundaries
    int rows = matrix[0].size();
    int cols = matrix.size();

    std::vector<Point> directions = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}}; // Left, Right, Up, Down
    Point bestMove = start;
    int minValue = rows*cols;

    if (startRow < 0 || startCol < 0 || endRow >= rows || endCol >= cols) {
        std::cerr << "Invalid range" << std::endl;
        return positions;
    }

    while (current.x!=end.x || current.y!=end.y){
        for (const auto& dir : directions) {
            int newX = current.x + dir.x;
            //std::cout << newX << endl;////vypis
            int newY = current.y + dir.y;
            if (newX >= 0 && newX < rows && newY >= 0 && newY < cols) {
                int neighborValue = matrix[newX][newY];
                if (neighborValue==-1 || neighborValue==1 || neighborValue==0){
                    neighborValue=minValue+1;
                }
                else if (neighborValue < minValue ) {
                    minValue = neighborValue;
                    bestMove = {newX, newY};
                }
            }

        }
        current=bestMove;
        positions.push_back(current);
    }

    return positions;
}

std::vector<Point> robot::r_checkpoint(const std::vector<Point>& points) {
    std::vector<Point> checkpoints;
    std::cout << "aj tu zijem" << endl;
    if (points.size() < 3) return points;

    for (size_t i = 1; i < points.size() - 1; ++i) {
        int dx1 = points[i].x - points[i - 1].x;
        int dy1 = points[i].y - points[i - 1].y;
        int dx2 = points[i + 1].x - points[i].x;
        int dy2 = points[i + 1].y - points[i].y;

        // If the direction changes, add the point as a checkpoint
        if (dx1 != dx2 || dy1 != dy2) {
            checkpoints.push_back(points[i]);
        }
    }

    checkpoints.push_back(points.back()); // Always include the last point
    return checkpoints;
}
////zadanie 4
////ZADANIE 4 externé volanie
std::vector<Point> robot::volaj_findpath(Point start, Point goal){
    map_na_fill[start.x][start.y] = -1;
    map_na_fill[goal.x][goal.y] = 2;
    map_na_fill=gradualFill(map_na_fill, goal, start);
    saveFilledMap("C:\\Users\\petri\\OneDrive\\Desktop\\RMR\\ROBOT_3\\kobuki\\mapa_fill.txt");
    //saveMapToFile(map, "map.txt");
    //std::cout << "Gradual fill map saved to map.txt" << std::endl;
    std::cout << "zijem aj tu" << endl;
    std::vector<Point> positions = findpath(map_na_fill, start, goal);
    //std::cout << "Path found\nCheckpoints:" << std::endl;

    std::cout << "Gruss Gott" << endl;
    std::vector<Point> result = r_checkpoint(positions);
    //for(const auto& pt : result) std::cout << pt.x << "," << pt.y << endl;
    return result;
}
////ZADANIE 4 externé volanie
  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
