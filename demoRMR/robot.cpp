#include "robot.h"
#include <cmath>



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

void robot::moveToGoalLogic(double max_ot, double min_ot, double max_s, double min_s){
    #define DEG_TO_RAD (M_PI / 180.0)  // Convert degrees to radians
    #define RAD_TO_DEG (180.0 / M_PI)  // Convert radians to degrees
    double targetAngle = atan2(targetY - y, targetX - x) * RAD_TO_DEG;
    double errorAngle = targetAngle - fi;

    // Normalize errorAngle to [-180, 180] to always take the shortest turn
    errorAngle = fmod(errorAngle + 180, 360);
    if (errorAngle < 0) errorAngle += 360;
    errorAngle -= 180;


    double distance = std::hypot(targetX - x, targetY - y);

    if (distance_scan == false){
        distance_all = std::hypot(targetX - x, targetY - y);
        distance_scan = true;
    }
    //double distance = std::hypot(targetX - x, targetY - y);

    double Kp_angle = 0.2;  // Adjust as needed
    double Kp_position = 300;
    double tolerance_angle;  // Small threshold for angle alignment
    double tolerance_pos = 0.05;   // Stop threshold (meters)

    double angular_speed = Kp_angle * errorAngle;
    double linear_speed = Kp_position * distance;

    //tolerancia uhlov

    if ((distance > (0.95)*distance_all)) tolerance_angle = 1;
    else if ((distance <= (0.95)*distance_all)&&(distance > (0.10)*distance_all)) tolerance_angle = 5;
    else tolerance_angle = 1;
    //if (distance>0.5 && distance<2) tolerance_angle = 45;
    //else tolerance_angle = 1;
    //tolerancia uhlov



    // **Reverse Logic: Move backward if turning more than 90° is needed**
    if (fabs(errorAngle) > 160) {
        if (errorAngle>90) errorAngle-=180;
        else errorAngle+=180;
        angular_speed = Kp_angle * errorAngle;
        linear_speed = -linear_speed;  // Reverse movement
    }

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
        }


    }
    else if (distance > tolerance_pos) {
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
        setSpeed(0, 0);  // Stop completely
        movingToGoal = false;  // Goal reached
        distance_scan = false; // aby mohol zase skenovat pre zadanau vzdialenost
    }
}

std::vector<double> occupancy_copy(36, 0.0);

void robot::obstacleAvoidance(double max_ot, double min_ot, double max_s, double min_s) {
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

    double targetAngle = atan2(targetY - y, targetX - x) * RAD_TO_DEG;
    double errorAngle = targetAngle - fi;

    // Normalize errorAngle to [-180, 180]
    errorAngle = fmod(errorAngle + 180, 360);
    if (errorAngle < 0) errorAngle += 360;
    errorAngle -= 180;

    double distance = std::hypot(targetX - x, targetY - y);

    if (distance_scan == false) {
        distance_all = std::hypot(targetX - x, targetY - y);
        distance_scan = true;
    }

    double Kp_angle = 0.05;
    double Kp_position = 300;
    double tolerance_angle;
    double tolerance_pos = 0.05;

    double angular_speed = Kp_angle * errorAngle;
    double linear_speed = Kp_position * distance;

    if ((distance > (0.95) * distance_all)) tolerance_angle = 1;
    else if ((distance <= (0.95) * distance_all) && (distance > (0.10) * distance_all)) tolerance_angle = 5;
    else tolerance_angle = 1;

    // === Polar Histogram Building ONLY ===
    const int num_sectors = 36; // 10 degrees per sector
    std::vector<double> sector_occupancy(num_sectors, 0.0);
    double obstacle_distance_threshold = 2.0; // meters

    // Build Polar Histogram of obstacles
    for (int i = 0; i < copyOfLaserData.numberOfScans; ++i) {
        double angle = copyOfLaserData.Data[i].scanAngle;
        double dist_meters = copyOfLaserData.Data[i].scanDistance / 1000.0;

        if (dist_meters > 0.05 && dist_meters < obstacle_distance_threshold) {
            if (angle < 0) angle += 360.0;
            else if (angle >= 360.0) angle -= 360.0;

            int sector_idx = static_cast<int>(angle / (360.0 / num_sectors)) % num_sectors;
            sector_occupancy[sector_idx] += (obstacle_distance_threshold - dist_meters);
        }
    }

    // Occupancy threshold and sector categorization
    double occupancy_threshold = 12;
    for (int i = 0; i < num_sectors; ++i) {
        if (sector_occupancy[i] < occupancy_threshold - 7) {
            sector_occupancy[i] = 0;  // Free sector
        }
        else if (sector_occupancy[i] > occupancy_threshold) {
            sector_occupancy[i] = 1;  // Occupied sector
        }
        else {
            // Transitioning: just copy from the previous sector (could use other logic here)
            if (i == 0) sector_occupancy[i] = occupancy_copy[num_sectors-1];
            else sector_occupancy[i] = occupancy_copy[i - 1];
        }
    }
    occupancy_copy=sector_occupancy;
    /* Just printing the polar histogram for debug purposes
    std::cout << "Polar Histogram (Sector Occupancy):" << std::endl;
    for (int i = 0; i < num_sectors; ++i) {
        std::cout << "[" << i << "] \t"
                  << sector_occupancy[i]
                  << std::endl;
    }
    std::cout << "---------------------------------" << std::endl;*/
    // === End of Polar Histogram ===


    // Calculate the goal angle (in degrees) relative to the robot's orientation
    double goal_angle = errorAngle;  // Angle in degrees
    goal_angle = fmod(goal_angle + 360.0, 360.0); // Wrap the angle into [0, 360) range
    int g_a = static_cast<int>(goal_angle / (360/num_sectors));  // num_sectors = 36


    // Print candidates for debugging
    std::cout << "Goal Angle: " << g_a << " angle:" << goal_angle<<std::endl;

    // Detect if obstacle is around the goal sector (not fixed front)
    bool obstacle_ahead = false;

    // Define how many sectors around the goal sector you consider "ahead"
    int sectors_to_check = 9; // example: +/-5 sectors

    for (int offset = -sectors_to_check; offset <= sectors_to_check; ++offset) {
        int check_sector = (g_a + offset + num_sectors) % num_sectors;
        if (sector_occupancy[check_sector] == 1) {
            obstacle_ahead = true;
            break;
        }
    }


    if (obstacle_ahead) {
        std::cout << "Obstacle detected ahead! Searching for free sector..." << std::endl;

        std::vector<float> candidate_sectors;

        for (int offset = 1; offset <= num_sectors/2; ++offset) {
            // Search to the right
            int right_sector = (g_a + offset) % num_sectors;
            if (sector_occupancy[right_sector] == 0) {
                candidate_sectors.push_back(right_sector+1.5);
                break;
            }
            // Search to the left
            int left_sector = (g_a - offset + num_sectors) % num_sectors;
            if (sector_occupancy[left_sector] == 0) {
                candidate_sectors.push_back(left_sector-1.5);
                break;
            }
        }

        if (candidate_sectors.size()) {
            float best_sector = candidate_sectors[0]; // Pick the 3rd found free sector

            double sector_size_deg = 360.0 / num_sectors;
            double new_goal_angle = best_sector * sector_size_deg;

            double angle_difference = new_goal_angle;

            // Normalize angle_difference to [-180, 180]
            angle_difference = fmod(angle_difference + 180, 360);
            if (angle_difference < 0) angle_difference += 360;
            angle_difference -= 180;

            errorAngle = angle_difference;
            angular_speed = Kp_angle * errorAngle;
            std::cout << "Redirecting to 3rd closest sector " << best_sector << " (new errorAngle: " << errorAngle << ")" << std::endl;
        }
        else if (!candidate_sectors.empty()) {
            int best_sector = candidate_sectors.back(); // Pick the last available one if less than 3

            double sector_size_deg = 360.0 / num_sectors;
            double new_goal_angle = best_sector * sector_size_deg;

            double angle_difference = new_goal_angle;

            // Normalize angle_difference to [-180, 180]
            angle_difference = fmod(angle_difference + 180, 360);
            if (angle_difference < 0) angle_difference += 360;
            angle_difference -= 180;

            errorAngle = angle_difference;
            angular_speed = Kp_angle * errorAngle;
            std::cout << "Redirecting to available sector " << best_sector << " (new errorAngle: " << errorAngle << ")" << std::endl;
        }
        else {
            std::cout << "No free sector found, staying on course!" << std::endl;
        }
    }




    // --- Movement Logic ---
    // The rest of the movement logic can continue as before, but now we can use the candidate directions
    // for choosing the next direction. For example, if the robot has a candidate in a sector,
    // you could choose that direction and attempt to move there.

    // Limit speeds
    if (angular_speed > max_ot) angular_speed = max_ot;
    else if (angular_speed < min_ot && angular_speed > 0) angular_speed = min_ot;
    if (angular_speed < -max_ot) angular_speed = -max_ot;
    else if (angular_speed < -min_ot && angular_speed < 0) angular_speed = -min_ot;

    if (linear_speed > max_s) linear_speed = max_s;
    else if (linear_speed < min_s && linear_speed > 0) linear_speed = min_s;
    if (linear_speed < -max_s) linear_speed = -max_s;
    else if (linear_speed > -min_s && linear_speed < 0) linear_speed = -min_s;

    // Acceleration & Deceleration
    double acceleration_linear = 10;
    double deceleration_linear = 10;
    double acceleration_angular = 0.1;
    double deceleration_angular = 0.1;

    static double current_linear_speed = 0;
    static double current_angular_speed = 0;

    if (fabs(errorAngle) > tolerance_angle) {
        linear_speed = 0;
        if (fabs(current_linear_speed) > 0 + min_s) {
            if (current_linear_speed < linear_speed) {
                current_linear_speed += acceleration_linear;
                if (current_linear_speed > linear_speed) current_linear_speed = linear_speed;
            } else {
                current_linear_speed -= deceleration_linear;
                if (current_linear_speed < linear_speed) current_linear_speed = linear_speed;
            }
            setSpeed(current_linear_speed, 0);
        }
        else {
            if (current_angular_speed < angular_speed) {
                current_angular_speed += acceleration_angular;
                if (current_angular_speed > angular_speed) current_angular_speed = angular_speed;
            } else {
                current_angular_speed -= deceleration_angular;
                if (current_angular_speed < angular_speed) current_angular_speed = angular_speed;
            }
            setSpeed(0, current_angular_speed);
        }
    }
    else if (distance > tolerance_pos) {
        if (current_linear_speed < linear_speed) {
            current_linear_speed += acceleration_linear;
            if (current_linear_speed > linear_speed) current_linear_speed = linear_speed;
        } else {
            current_linear_speed -= deceleration_linear;
            if (current_linear_speed < linear_speed) current_linear_speed = linear_speed;
        }
        setSpeed(current_linear_speed, 0);
    }
    else {
        setSpeed(0, 0);
        movingToGoal = false;
        distance_scan = false;
    }

}






int robot::processThisRobot(TKobukiData robotdata)
{


    ///tu mozete robit s datami z robota
    #define DEG_TO_RAD (M_PI / 180.0)  // Convert degrees to radians
    #define RAD_TO_DEG (180.0 / M_PI)  // Convert radians to degrees
    double max_ot=0.5, min_ot=0.3, max_s=300, min_s=30;


    // If this is the first run, store encoder values and exit
    if (firstRun) {
        prevEncoderRight = robotdata.EncoderRight;
        prevEncoderLeft = robotdata.EncoderLeft;
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

    // Handle encoder wrap-around (assumes 16-bit encoder, adjust if needed)
    if (deltaRight > MAX_ENCODER_VALUE / 2) deltaRight -= MAX_ENCODER_VALUE;
    if (deltaRight < -MAX_ENCODER_VALUE / 2) deltaRight += MAX_ENCODER_VALUE;
    if (deltaLeft > MAX_ENCODER_VALUE / 2) deltaLeft -= MAX_ENCODER_VALUE;
    if (deltaLeft < -MAX_ENCODER_VALUE / 2) deltaLeft += MAX_ENCODER_VALUE;

    // Store previous encoder values
    prevEncoderRight = robotdata.EncoderRight;
    prevEncoderLeft = robotdata.EncoderLeft;

    // Convert encoder ticks to wheel rotation (radians)
    double omegaRight = (deltaRight / TICKS_PER_REV) * TWO_PI;  // Now in radians
    double omegaLeft = (deltaLeft / TICKS_PER_REV) * TWO_PI;    // Now in radians

    // Compute linear and angular velocity
    double v = ((omegaLeft + omegaRight) * WHEEL_RADIUS) / 2.0;
    double omega = ((omegaRight - omegaLeft) * WHEEL_RADIUS) / WHEEL_BASE; // Ensure correct sign

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
    // Convert `fi` back to degrees if needed
    if (fiInDegrees) {
        fi *= RAD_TO_DEG;
    }

    if (movingToGoal) {
        obstacleAvoidance(max_ot, min_ot, max_s, min_s);
    }







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
int robot::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

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
