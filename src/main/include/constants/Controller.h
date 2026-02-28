#pragma once

namespace constants {
    namespace controller {
        constexpr int kDriverControllerPort {0};
        constexpr int kCoDriverControllerPort {1};
        constexpr int kTestControllerPort {2};
        constexpr int kButtonIDSquare {1};
        constexpr int kButtonIDX {2};
        constexpr int kButtonIDCircle {3};
        constexpr int kButtonIDTriangle {4};
        constexpr int kButtonIDLeftBumper {5};
        constexpr int kButtonIDRightBumper {6};
        constexpr int kButtonIDLeftTrigger {7};
        constexpr int kButtonIDRightTrigger {8};
        constexpr int kButtonIDCreate {9};
        constexpr int kButtonIDMenu {10};
        constexpr int kButtonIDLeftStick {11};
        constexpr int kButtonIDRightStick {12};
        constexpr int kButtonIDPlaystation {13};
        constexpr int kButtonIDTouchpad {14};
        constexpr int kButtonIDMicrophone {15};

        constexpr int kAxisLeftStickX {0};
        constexpr int kAxisLeftStickY {1};
        constexpr int kAxisRightStickX {2};
        constexpr int kAxisLeftTrigger {3};
        constexpr int kAxisRightTrigger {4};
        constexpr int kAxisRightStickY {5};

        // ! Button board 1
        constexpr int kButtonBoardPort {3}; 
        constexpr int kButtonBoard2Port {4}; 
        // constexpr int KButtonBoardPort {3};

        constexpr int KSetupElevatorL1 {1};
        constexpr int KSetupElevatorL2 {2};
        constexpr int KSetupElevatorL3 {3};
        constexpr int KSetupElevatorL4 {4};

        constexpr int KManualControlToggle {5};

        constexpr int kAlgaeToggle {6};

        constexpr int KMoveCoralUpButton{7};
        constexpr int KMoveCoralDownButton{8};

        constexpr int kNetScoreSetUpButton{9};

        constexpr int KResetElevator {10};

        // ! END OF Board 1

        constexpr int kSetUpProcesser{17};


        // constexpr int kAlgaeGrabHigh {6};
        // constexpr int kAlgaeGrabLow {5}; 
        constexpr int kScoreShot {9};

        //! ButtonBoard2 Buttons

        constexpr int kBranch1{1};
        constexpr int kBranch2{2};
        constexpr int kBranch3{3};
        constexpr int kBranch4{4};
        constexpr int kBranch5{5};
        constexpr int kBranch6{6}; //kBlueBranch9
        constexpr int kBranch7{7}; //kBlueBranch10
        constexpr int kBranch8{8};
        constexpr int kBranch9{9};
        constexpr int kBranch10{10};
        constexpr int kBranch11{11};
        constexpr int kBranch12{12};


        // constexpr int kL2ScoreButton {11};
        // constexpr int kL3ScoreButton {12};
        // constexpr int kL4ScoreButton {7};
        // constexpr int kL1ScoreButton {};

    };
};
