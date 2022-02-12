#pragma once

namespace interfaces
{
    //for future note, each subsystem has their own block of 10 CAN IDs (presuming it needs them)
    //block 0 (00-09) is reserved for the drivetrain
    //block 1 (10-19) is reserved for the shooter
    //block 2 (20-29) is not reserved
    //block 3 (30-39) is not reserved
    //block 4 (40-49) is not reserved

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

    const int kShooterMotor = kCAN10;
}