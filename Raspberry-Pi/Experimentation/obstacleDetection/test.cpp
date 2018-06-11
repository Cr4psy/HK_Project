
#include <iostream>
#include <math.h>

#define PI 3.14159265
using namespace std;

#define RED 1
#define YEL 2
#define GRE 3


/*Treshold that define the distance for the danger zone*/
#define RED_TRESHOLD 200
#define YEL_TRESHOLD 400
#define TRESHOLD_SAME_OBJECT 10


#define DRONE_RADIUS 30

#define RADAR_MAX_RANGE 1000

#define SONAR_CONE 67
#define SONAR_MAX_RANGE 600

#define LONG_IR_MAX_RANGE 400

#define LIMIT_ZONE_IR 230

/*Class for each zone defined*/
class zone {
    public:
        int area;//Class of danger
        bool detection;//Obstacle detected
        bool prob;//Whether you are sure or not


        void areaComputation(int distance){
            if(detection)
            {
                if(distance <= RED_TRESHOLD){
                    area = RED;
                    return;
                }
                else if(distance<= YEL_TRESHOLD){
                    area = YEL;
                    return;
                }
                else{
                    area = GRE;
                    return;
                }
            }
            else{
                area = GRE;
                return;
            }
        }
};


class sensor {
    public:
        int measuredDistance;
        int xSensor;
        int ySensor;
        int angle;
        int maxRange;
        int coneAngle;

        void setParam(int inAngle, int inMaxRange, int inConeAngle){
            angle = inAngle;
            maxRange = inMaxRange;
            coneAngle = inConeAngle;
            computeSensorCoord();
            return;
        }

        void computeSensorCoord(){
            xSensor = cos(angle)*DRONE_RADIUS;
            ySensor = -sin(angle)*DRONE_RADIUS;
        }
};


class IRSensor : public sensor {
    public:
        int xObject;
        int yObject;

    void computeObjectCoord(int inMeasuredDistance) {
        xObject = xSensor + cos(angle)*inMeasuredDistance;
        yObject = ySensor - sin(angle)*inMeasuredDistance;
        measuredDistance = inMeasuredDistance;
        return;
    }
};


bool sameObjectIR(IRSensor ir, sensor sonar){
        int distance = sqrt(pow((ir.xObject-sonar.xSensor),2)+pow((ir.yObject-sonar.ySensor),2));
        return (fabs(distance - sonar.measuredDistance)<=TRESHOLD_SAME_OBJECT);
}

bool sameObjectSonar(sensor sonar1, sensor sonar2){
    return (fabs(sonar1.measuredDistance-sonar2.measuredDistance)<=TRESHOLD_SAME_OBJECT);
}

int main()
{
    /*Initialisation of the sensors*/
    sensor radar;
    radar.setParam(0,RADAR_MAX_RANGE,50);
    IRSensor longIRfront;
    longIRfront.setParam(0,LONG_IR_MAX_RANGE,0);//Initialise sensors param
    IRSensor longIRright;
    longIRright.setParam(30,LONG_IR_MAX_RANGE,0);
    IRSensor longIRleft;
    longIRleft.setParam(-30,LONG_IR_MAX_RANGE,0);
    sensor sonarFrontRight;
    sonarFrontRight.setParam(45,SONAR_MAX_RANGE,SONAR_CONE);
    sensor sonarFrontLeft;
    sonarFrontLeft.setParam(-45,SONAR_MAX_RANGE,SONAR_CONE);
    sensor sonarRight;
    sonarRight.setParam(90,SONAR_MAX_RANGE,SONAR_CONE);
    sensor sonarLeft;
    sonarLeft.setParam(-90,SONAR_MAX_RANGE,SONAR_CONE);
    sensor sonarBack;
    sonarBack.setParam(180,SONAR_MAX_RANGE,SONAR_CONE);

    /*Initialisation of the zone*/
    //Refer to the drawing
    zone zone1;
    zone zone2;
    zone zone3;
    zone zone4;
    zone zone5;
    zone zone6;
    zone zone7;
    zone zone8;
    zone zone9;
    zone zone10;
    zone zone11;
    zone zone12;
    zone zone13;
    zone zone14;
    zone zone15;


    /* Update the sensors measures */
    //TODO

    /* Object detection*/
    zone1.detection = ((radar.measuredDistance <= radar.maxRange) && (radar.measuredDistance > longIRfront.maxRange));
    zone2.detection = (longIRfront.measuredDistance <= longIRfront.maxRange);
    zone3.prob = ((radar.measuredDistance <= longIRfront.maxRange) && (!zone2.detection));
    zone4.prob = ((radar.measuredDistance <= longIRfront.maxRange) && (!zone2.detection));
    zone5.detection = ((longIRright.measuredDistance > LIMIT_ZONE_IR) && (longIRright.measuredDistance <= longIRright.maxRange));
    zone6.detection = ((longIRleft.measuredDistance > LIMIT_ZONE_IR) && (longIRleft.measuredDistance <= longIRleft.maxRange));
    zone7.detection = (longIRright.measuredDistance <= LIMIT_ZONE_IR);
    zone8.detection = (longIRleft.measuredDistance <= LIMIT_ZONE_IR);
    zone11.detection = sameObjectSonar(sonarFrontRight,sonarRight);
    zone9.detection = ((sonarFrontRight.measuredDistance <= sonarFrontRight.maxRange) && !(sameObjectIR(longIRright,sonarFrontRight)) && !zone11.detection);
    zone12.detection = sameObjectSonar(sonarFrontLeft,sonarLeft);
    zone10.detection = ((sonarFrontLeft.measuredDistance <= sonarFrontLeft.maxRange) && !(sameObjectIR(longIRleft,sonarFrontLeft)) && !zone12.detection);
    zone13.detection = ((sonarRight.measuredDistance <= sonarRight.maxRange) && !zone11.detection);
    zone14.detection = ((sonarLeft.measuredDistance <= sonarLeft.maxRange) && !zone12.detection);
    zone15.detection = (sonarBack.measuredDistance <= sonarBack.maxRange);


    /*Define area*/
    zone1.areaComputation(radar.measuredDistance);
    zone2.areaComputation(longIRfront.measuredDistance);
    zone3.areaComputation(radar.measuredDistance);
    zone4.areaComputation(radar.measuredDistance);
    zone5.areaComputation(longIRright.measuredDistance);
    zone6.areaComputation(longIRleft.measuredDistance);
    zone7.areaComputation(longIRright.measuredDistance);
    zone8.areaComputation(longIRleft.measuredDistance);
    zone9.areaComputation(sonarFrontRight.measuredDistance);
    zone10.areaComputation(sonarFrontLeft.measuredDistance);
    zone11.areaComputation(sonarFrontRight.measuredDistance);
    zone12.areaComputation(sonarFrontLeft.measuredDistance);
    zone13.areaComputation(sonarRight.measuredDistance);
    zone14.areaComputation(sonarLeft.measuredDistance);
    zone15.areaComputation(sonarBack.measuredDistance);



    int measure = 3;

    bool a = true;
    longIRfront.computeObjectCoord(measure);//Call to compute object pos

    cout << "Sensor:  " << !(a) << endl;
    return 0;
}
