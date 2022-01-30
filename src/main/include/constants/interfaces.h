#pragma once

namespace interfaces
{
    //for future note, each subsystem has their own block of 10 CAN IDs (presuming it needs them)
    //block 0 (00-09) is reserved for the drivetrain
    //block 1 (10-19) is not reserved
    //block 2 (20-29) is not reserved
    //block 3 (30-39) is not reserved
    //block 4 (40-49) is not reserved

    //done to prevent direct access to CAN pins and instead you have to use the pretty print name
    namespace
    {
        //block 0
        const int k_CAN00 = 0;
        const int k_CAN01 = 1;
        const int k_CAN02 = 2;
        const int k_CAN03 = 3;
        const int k_CAN04 = 4;
        const int k_CAN05 = 5;
        const int k_CAN06 = 6;
        const int k_CAN07 = 7;
        const int k_CAN08 = 8;
        const int k_CAN09 = 9;

        //block 1
        const int k_CAN10 = 10;
        const int k_CAN11 = 11;
        const int k_CAN12 = 12;
        const int k_CAN13 = 13;
        const int k_CAN14 = 14;
        const int k_CAN15 = 15;
        const int k_CAN16 = 16;
        const int k_CAN17 = 17;
        const int k_CAN18 = 18;
        const int k_CAN19 = 19;
    }

    /*
    |1       2|
    |         |
    |         |
    |4       3|
    */
    //drivetrain CAN assignments (block 0)
    
    const int k_Drive1 = k_CAN00;
    const int k_Drive2 = k_CAN01;
    const int k_Drive3 = k_CAN02;
    const int k_Drive4 = k_CAN03;
    const int k_Steer1 = k_CAN04;
    const int k_Steer2 = k_CAN05;
    const int k_Steer3 = k_CAN06;
    const int k_Steer4 = k_CAN07;
}