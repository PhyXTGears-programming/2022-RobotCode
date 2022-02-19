#pragma once

namespace interfaces
{
    //for future note, each subsystem has their own block of 10 CAN IDs (presuming it needs them)
    //block 0 (00-09) is reserved for the drivetrain
    //block 1 (10-19) is not reserved
    //block 2 (20-29) is not reserved
    //block 3 (30-39) is not reserved
    //block 4 (40-49) is reserved for the climber

    //done to prevent direct access to CAN pins and instead you have to use the pretty print name
    namespace
    {
        //block 0
        const int kCAN00 = 0;
        const int kCAN01 = 1;
        const int kCAN02 = 2;
        const int kCAN03 = 3;
        const int kCAN04 = 4;
        const int kCAN05 = 5;
        const int kCAN06 = 6;
        const int kCAN07 = 7;
        const int kCAN08 = 8;
        const int kCAN09 = 9;

        //block 1
        const int kCAN10 = 10;
        const int kCAN11 = 11;
        const int kCAN12 = 12;
        const int kCAN13 = 13;
        const int kCAN14 = 14;
        const int kCAN15 = 15;
        const int kCAN16 = 16;
        const int kCAN17 = 17;
        const int kCAN18 = 18;
        const int kCAN19 = 19;

        //block 2
        const int kCAN20 = 20;
        const int kCAN21 = 21;
        const int kCAN22 = 22;
        const int kCAN23 = 23;
        const int kCAN24 = 24;
        const int kCAN25 = 25;
        const int kCAN26 = 26;
        const int kCAN27 = 27;
        const int kCAN28 = 28;
        const int kCAN29 = 29;

        //block 4
        const int kCAN40 = 40;
        const int kCAN41 = 41;
        const int kCAN42 = 42;
        const int kCAN43 = 43;
        const int kCAN44 = 44;
        const int kCAN45 = 45;
        const int kCAN46 = 46;
        const int kCAN47 = 47;
        const int kCAN48 = 48;
        const int kCAN49 = 49;

        //PWM Servos
        const int kPWM0 = 0;
        const int kPWM1 = 1;
        const int kPWM2 = 2;
        const int kPWM3 = 3;
        const int kPWM4 = 4;
        const int kPWM5 = 5;
        const int kPWM6 = 6;
        const int kPWM7 = 7;
        const int kPWM8 = 8;
        const int kPWM9 = 9;
        

        //DIO
        const int kDIO0 = 0;
        const int kDIO1 = 1;
        const int kDIO2 = 2;
        const int kDIO3 = 3;
        const int kDIO4 = 4;
        const int kDIO5 = 5;
        const int kDIO6 = 6;
        const int kDIO7 = 7;
        const int kDIO8 = 8;
        const int kDIO9 = 9;
    }

    /*
    |1       2|
    |         |
    |         |
    |4       3|
    */
    //drivetrain CAN assignments (block 0)
    
    const int kDrive1 = kCAN00;
    const int kDrive2 = kCAN01;
    const int kDrive3 = kCAN02;
    const int kDrive4 = kCAN03;
    const int kSteer1 = kCAN04;
    const int kSteer2 = kCAN05;
    const int kSteer3 = kCAN06;
    const int kSteer4 = kCAN07;

    // climber CAN (block 4), PWM, and DIO assignments

    const int kInnerArm1 = kCAN40;
    const int kInnerArm2 = kCAN41;
    const int kOuterArm1 = kCAN42;
    const int kOuterArm2 = kCAN43;
    
    const int kStopServo1 = kPWM0;
    const int kStopServo2 = kPWM1;
    const int kInnerServo1 = kPWM2;
    const int kInnerServo2 = kPWM3;
    const int kOuterServo1 = kPWM4;
    const int kOuterServo2 = kPWM5;

    const int kBackDriveRelay = kDIO0;
}
