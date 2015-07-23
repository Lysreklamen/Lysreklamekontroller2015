------------------------------------------------------------
-- VHDL Kontrollerkort_top
-- 2015 7 23 20 4 41
-- Created By "DXP VHDL Generator"
-- "Copyright (c) 2002-2014 Altium Limited"
-- Product Version: 15.0.7.36915
------------------------------------------------------------

------------------------------------------------------------
-- VHDL Kontrollerkort_top
------------------------------------------------------------

Library IEEE;
Use     IEEE.std_logic_1164.all;

Entity Kontrollerkort_top Is
  attribute MacroCell : boolean;

End Kontrollerkort_top;
------------------------------------------------------------

------------------------------------------------------------
Architecture Structure Of Kontrollerkort_top Is
   Component X_8_2pF_0603                                    -- ObjectKind=Part|PrimaryId=C2|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=C2-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=C2-2
      );
   End Component;

   Component X_22uF_1206                                     -- ObjectKind=Part|PrimaryId=C4|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=C4-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=C4-2
      );
   End Component;

   Component X_100nF_0603                                    -- ObjectKind=Part|PrimaryId=C1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=C1-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=C1-2
      );
   End Component;

   Component X_2384                                          -- ObjectKind=Part|PrimaryId=R13|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=R13-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=R13-2
      );
   End Component;

   Component X_2444                                          -- ObjectKind=Part|PrimaryId=R4|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=R4-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=R4-2
      );
   End Component;

   Component X_3050                                          -- ObjectKind=Part|PrimaryId=R1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=R1-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=R1-2
      );
   End Component;

   Component X_3186                                          -- ObjectKind=Part|PrimaryId=R5|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=R5-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=R5-2
      );
   End Component;

   Component AVR_ATXMEGA64A3UMINUSAU                         -- ObjectKind=Part|PrimaryId=U1|SecondaryId=1
      port
      (
        X_1  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-1
        X_2  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-2
        X_3  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-3
        X_4  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-4
        X_5  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-5
        X_6  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-6
        X_7  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-7
        X_8  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-8
        X_9  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-9
        X_10 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-10
        X_11 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-11
        X_12 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-12
        X_13 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-13
        X_16 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-16
        X_17 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-17
        X_18 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-18
        X_19 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-19
        X_20 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-20
        X_21 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-21
        X_22 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-22
        X_23 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-23
        X_26 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-26
        X_27 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-27
        X_28 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-28
        X_29 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-29
        X_30 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-30
        X_31 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-31
        X_32 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-32
        X_33 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-33
        X_36 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-36
        X_37 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-37
        X_38 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-38
        X_39 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-39
        X_40 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-40
        X_41 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-41
        X_42 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-42
        X_43 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-43
        X_46 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-46
        X_47 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-47
        X_48 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-48
        X_49 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-49
        X_50 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-50
        X_51 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-51
        X_54 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-54
        X_55 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-55
        X_56 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-56
        X_57 : in    STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-57
        X_58 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-58
        X_59 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-59
        X_62 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-62
        X_63 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=U1-63
        X_64 : inout STD_LOGIC                               -- ObjectKind=Pin|PrimaryId=U1-64
      );
   End Component;

   Component Header_1x3                                      -- ObjectKind=Part|PrimaryId=J1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J1-1
        X_2 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J1-2
        X_3 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=J1-3
      );
   End Component;

   Component Lysreklameparalellbusskontakt2015               -- ObjectKind=Part|PrimaryId=J2|SecondaryId=1
      port
      (
        X_1  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-1
        X_2  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-2
        X_3  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-3
        X_4  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-4
        X_5  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-5
        X_6  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-6
        X_7  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-7
        X_8  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-8
        X_9  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-9
        X_10 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-10
        X_11 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-11
        X_12 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-12
        X_13 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-13
        X_14 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-14
        X_15 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-15
        X_16 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-16
        X_17 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-17
        X_18 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-18
        X_19 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-19
        X_20 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-20
        X_21 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-21
        X_22 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-22
        X_23 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-23
        X_24 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-24
        X_25 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-25
        X_26 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-26
        X_27 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-27
        X_28 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-28
        X_29 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-29
        X_30 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-30
        X_31 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J2-31
        X_32 : inout STD_LOGIC                               -- ObjectKind=Pin|PrimaryId=J2-32
      );
   End Component;

   Component MAX3483                                         -- ObjectKind=Part|PrimaryId=U2|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U2-1
        X_2 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U2-2
        X_3 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U2-3
        X_4 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U2-4
        X_5 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U2-5
        X_6 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U2-6
        X_7 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U2-7
        X_8 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=U2-8
      );
   End Component;

   Component NX5032GAMINUS16                                 -- ObjectKind=Part|PrimaryId=X1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=X1-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=X1-2
      );
   End Component;

   Component PDIMINUSHEADER                                  -- ObjectKind=Part|PrimaryId=J3|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J3-1
        X_2 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J3-2
        X_3 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J3-3
        X_4 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J3-4
        X_5 : out   STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J3-5
        X_6 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=J3-6
      );
   End Component;

   Component SMD_LED_Red                                     -- ObjectKind=Part|PrimaryId=D1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=D1-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=D1-2
      );
   End Component;

   Component TLV1117                                         -- ObjectKind=Part|PrimaryId=U3|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U3-1
        X_2 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U3-2
        X_3 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=U3-3
        X_4 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=U3-4
      );
   End Component;


    Signal NamedIOSignal_BOKS_ID : STD_LOGIC;
    Signal NamedIOSignal_DEBUG_LED_1 : STD_LOGIC;
    Signal NamedIOSignal_DEBUG_LED_2 : STD_LOGIC;
    Signal NamedIOSignal_DEBUG_LED_3 : STD_LOGIC;
    Signal NamedIOSignal_DMX_DE : STD_LOGIC;
    Signal NamedIOSignal_DMX_DI : STD_LOGIC;
    Signal NamedIOSignal_DMX_RD : STD_LOGIC;
    Signal NamedIOSignal_KANAL_1MINUS1 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_1MINUS2 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_1MINUS3 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_2MINUS1 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_2MINUS2 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_2MINUS3 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_3MINUS1 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_3MINUS2 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_3MINUS3 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_4MINUS1 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_4MINUS2 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_4MINUS3 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_5MINUS1 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_5MINUS2 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_5MINUS3 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_6MINUS1 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_6MINUS2 : STD_LOGIC;
    Signal NamedIOSignal_KANAL_6MINUS3 : STD_LOGIC;
    Signal NamedIOSignal_PB0    : STD_LOGIC;
    Signal NamedIOSignal_PB1    : STD_LOGIC;
    Signal NamedIOSignal_PB2    : STD_LOGIC;
    Signal NamedIOSignal_PB3    : STD_LOGIC;
    Signal NamedIOSignal_PB4    : STD_LOGIC;
    Signal NamedIOSignal_PB5    : STD_LOGIC;
    Signal NamedIOSignal_PB6    : STD_LOGIC;
    Signal NamedIOSignal_PB7    : STD_LOGIC;
    Signal NamedIOSignal_RXD1   : STD_LOGIC;
    Signal NamedIOSignal_TXD1   : STD_LOGIC;
    Signal NamedIOSignal_X_1    : STD_LOGIC;
    Signal NamedIOSignal_X_2    : STD_LOGIC;
    Signal NamedIOSignal_X_29   : STD_LOGIC;
    Signal NamedIOSignal_X_3    : STD_LOGIC;
    Signal NamedIOSignal_X_30   : STD_LOGIC;
    Signal NamedIOSignal_X_4    : STD_LOGIC;
    Signal NamedIOSignal_X_46   : STD_LOGIC;
    Signal NamedIOSignal_X_5    : STD_LOGIC;
    Signal NamedIOSignal_X_50   : STD_LOGIC;
    Signal NamedIOSignal_X_51   : STD_LOGIC;
    Signal NamedIOSignal_X_54   : STD_LOGIC;
    Signal NamedIOSignal_X_55   : STD_LOGIC;
    Signal NamedIOSignal_X_58   : STD_LOGIC;
    Signal NamedIOSignal_X_59   : STD_LOGIC;
    Signal NamedIOSignal_X_62   : STD_LOGIC;
    Signal NamedSignal_DMXMINUS : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX-
    Signal NamedSignal_DMXPLUS  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX+
    Signal PinSignal_D1_1       : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetD1_1
    Signal PinSignal_D2_1       : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetD2_1
    Signal PinSignal_D3_1       : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetD3_1
    Signal PinSignal_J3_5       : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetC1_2
    Signal PowerSignal_GND      : STD_LOGIC; -- ObjectKind=Net|PrimaryId=GND
    Signal PowerSignal_VCC_P3V3 : STD_LOGIC; -- ObjectKind=Net|PrimaryId=VCC_P3V3
    Signal PowerSignal_VCC_P5V0 : STD_LOGIC; -- ObjectKind=Net|PrimaryId=VCC_P5V0

   attribute antall : string;
   attribute antall of X1  : Label is "42";
   attribute antall of U3  : Label is "10";
   attribute antall of U2  : Label is "40";
   attribute antall of U1  : Label is "40";
   attribute antall of R14 : Label is "100";
   attribute antall of R13 : Label is "100";
   attribute antall of R4  : Label is "100";
   attribute antall of D3  : Label is "50";
   attribute antall of D2  : Label is "50";
   attribute antall of D1  : Label is "50";

   attribute Database_Table_Name : string;
   attribute Database_Table_Name of X1  : Label is "altium";
   attribute Database_Table_Name of U3  : Label is "altium";
   attribute Database_Table_Name of U2  : Label is "altium";
   attribute Database_Table_Name of U1  : Label is "altium";
   attribute Database_Table_Name of R14 : Label is "altium_Motstander";
   attribute Database_Table_Name of R13 : Label is "altium_Motstander";
   attribute Database_Table_Name of R12 : Label is "altium_Motstander";
   attribute Database_Table_Name of R11 : Label is "altium_Motstander";
   attribute Database_Table_Name of R10 : Label is "altium_Motstander";
   attribute Database_Table_Name of R9  : Label is "altium_Motstander";
   attribute Database_Table_Name of R8  : Label is "altium_Motstander";
   attribute Database_Table_Name of R7  : Label is "altium_Motstander";
   attribute Database_Table_Name of R6  : Label is "altium_Motstander";
   attribute Database_Table_Name of R5  : Label is "altium_Motstander";
   attribute Database_Table_Name of R4  : Label is "altium_Motstander";
   attribute Database_Table_Name of R3  : Label is "altium_Motstander";
   attribute Database_Table_Name of R2  : Label is "altium_Motstander";
   attribute Database_Table_Name of R1  : Label is "altium_Motstander";
   attribute Database_Table_Name of J3  : Label is "altium";
   attribute Database_Table_Name of J1  : Label is "altium";
   attribute Database_Table_Name of D3  : Label is "altium";
   attribute Database_Table_Name of D2  : Label is "altium";
   attribute Database_Table_Name of D1  : Label is "altium";
   attribute Database_Table_Name of C11 : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C10 : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C9  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C8  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C7  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C6  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C5  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C4  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C3  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C2  : Label is "altium_Kondensatorer";
   attribute Database_Table_Name of C1  : Label is "altium_Kondensatorer";

   attribute dybde : string;
   attribute dybde of X1  : Label is "1";
   attribute dybde of U3  : Label is "0";
   attribute dybde of U2  : Label is "0";
   attribute dybde of U1  : Label is "0";
   attribute dybde of R14 : Label is "96";
   attribute dybde of R13 : Label is "96";
   attribute dybde of R4  : Label is "72";
   attribute dybde of D3  : Label is "2";
   attribute dybde of D2  : Label is "2";
   attribute dybde of D1  : Label is "2";

   attribute hylle : string;
   attribute hylle of X1  : Label is "2";
   attribute hylle of U3  : Label is "9";
   attribute hylle of U2  : Label is "2";
   attribute hylle of U1  : Label is "0";
   attribute hylle of R14 : Label is "6";
   attribute hylle of R13 : Label is "6";
   attribute hylle of R4  : Label is "6";
   attribute hylle of D3  : Label is "13";
   attribute hylle of D2  : Label is "13";
   attribute hylle of D1  : Label is "13";

   attribute id : string;
   attribute id of X1  : Label is "1594";
   attribute id of U3  : Label is "2386";
   attribute id of U2  : Label is "3048";
   attribute id of U1  : Label is "2105";
   attribute id of R14 : Label is "2384";
   attribute id of R13 : Label is "2384";
   attribute id of R12 : Label is "3186";
   attribute id of R11 : Label is "3186";
   attribute id of R10 : Label is "3186";
   attribute id of R9  : Label is "3186";
   attribute id of R8  : Label is "3186";
   attribute id of R7  : Label is "3186";
   attribute id of R6  : Label is "3186";
   attribute id of R5  : Label is "3186";
   attribute id of R4  : Label is "2444";
   attribute id of R3  : Label is "3050";
   attribute id of R2  : Label is "3050";
   attribute id of R1  : Label is "3050";
   attribute id of J3  : Label is "2407";
   attribute id of J1  : Label is "2430";
   attribute id of D3  : Label is "2209";
   attribute id of D2  : Label is "2209";
   attribute id of D1  : Label is "2209";
   attribute id of C11 : Label is "3028";
   attribute id of C10 : Label is "3028";
   attribute id of C9  : Label is "3028";
   attribute id of C8  : Label is "3028";
   attribute id of C7  : Label is "3028";
   attribute id of C6  : Label is "3028";
   attribute id of C5  : Label is "3049";
   attribute id of C4  : Label is "3049";
   attribute id of C3  : Label is "3032";
   attribute id of C2  : Label is "3032";
   attribute id of C1  : Label is "3028";

   attribute kolonne : string;
   attribute kolonne of X1  : Label is "3";
   attribute kolonne of U3  : Label is "0";
   attribute kolonne of U2  : Label is "4";
   attribute kolonne of U1  : Label is "1";
   attribute kolonne of R14 : Label is "0";
   attribute kolonne of R13 : Label is "0";
   attribute kolonne of R4  : Label is "0";
   attribute kolonne of D3  : Label is "0";
   attribute kolonne of D2  : Label is "0";
   attribute kolonne of D1  : Label is "0";

   attribute lager_type : string;
   attribute lager_type of X1  : Label is "Fremlager";
   attribute lager_type of U3  : Label is "Fremlager";
   attribute lager_type of U2  : Label is "Fremlager";
   attribute lager_type of U1  : Label is "Fremlager";
   attribute lager_type of R14 : Label is "Fremlager";
   attribute lager_type of R13 : Label is "Fremlager";
   attribute lager_type of R4  : Label is "Fremlager";
   attribute lager_type of D3  : Label is "Fremlager";
   attribute lager_type of D2  : Label is "Fremlager";
   attribute lager_type of D1  : Label is "Fremlager";

   attribute leverandor : string;
   attribute leverandor of X1 : Label is "DigiKey";
   attribute leverandor of U3 : Label is "Farnell";
   attribute leverandor of U2 : Label is "Farnell";
   attribute leverandor of J3 : Label is "Farnell";
   attribute leverandor of D3 : Label is "Farnell";
   attribute leverandor of D2 : Label is "Farnell";
   attribute leverandor of D1 : Label is "Farnell";
   attribute leverandor of C5 : Label is "Farnell";
   attribute leverandor of C4 : Label is "Farnell";

   attribute leverandor_varenr : string;
   attribute leverandor_varenr of X1 : Label is "644-1135-1-ND";
   attribute leverandor_varenr of U3 : Label is "1494942";
   attribute leverandor_varenr of U2 : Label is "1296640";
   attribute leverandor_varenr of J3 : Label is "Derp";
   attribute leverandor_varenr of D3 : Label is "8554641";
   attribute leverandor_varenr of D2 : Label is "8554641";
   attribute leverandor_varenr of D1 : Label is "8554641";
   attribute leverandor_varenr of C5 : Label is "1457420";
   attribute leverandor_varenr of C4 : Label is "1457420";

   attribute navn : string;
   attribute navn of X1  : Label is "NX5032GA-16";
   attribute navn of U3  : Label is "TLV1117";
   attribute navn of U2  : Label is "MAX3483";
   attribute navn of U1  : Label is "AVR ATXMEGA64A3U-AU";
   attribute navn of R14 : Label is "100k(0603)";
   attribute navn of R13 : Label is "100k(0603)";
   attribute navn of R12 : Label is "0R(0603)";
   attribute navn of R11 : Label is "0R(0603)";
   attribute navn of R10 : Label is "0R(0603)";
   attribute navn of R9  : Label is "0R(0603)";
   attribute navn of R8  : Label is "0R(0603)";
   attribute navn of R7  : Label is "0R(0603)";
   attribute navn of R6  : Label is "0R(0603)";
   attribute navn of R5  : Label is "0R(0603)";
   attribute navn of R4  : Label is "10k(0603)";
   attribute navn of R3  : Label is "120(0603)";
   attribute navn of R2  : Label is "120(0603)";
   attribute navn of R1  : Label is "120(0603)";
   attribute navn of J3  : Label is "PDI-HEADER";
   attribute navn of J1  : Label is "Header 1x3";
   attribute navn of D3  : Label is "SMD LED Red";
   attribute navn of D2  : Label is "SMD LED Red";
   attribute navn of D1  : Label is "SMD LED Red";
   attribute navn of C11 : Label is "100nF(0603)";
   attribute navn of C10 : Label is "100nF(0603)";
   attribute navn of C9  : Label is "100nF(0603)";
   attribute navn of C8  : Label is "100nF(0603)";
   attribute navn of C7  : Label is "100nF(0603)";
   attribute navn of C6  : Label is "100nF(0603)";
   attribute navn of C5  : Label is "22uF(1206)";
   attribute navn of C4  : Label is "22uF(1206)";
   attribute navn of C3  : Label is "8.2pF(0603)";
   attribute navn of C2  : Label is "8.2pF(0603)";
   attribute navn of C1  : Label is "100nF(0603)";

   attribute nokkelord : string;
   attribute nokkelord of X1  : Label is "xtal, oscillator";
   attribute nokkelord of U3  : Label is "ldo";
   attribute nokkelord of U2  : Label is "RS485";
   attribute nokkelord of R14 : Label is "Resistor";
   attribute nokkelord of R13 : Label is "Resistor";
   attribute nokkelord of R12 : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R11 : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R10 : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R9  : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R8  : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R7  : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R6  : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R5  : Label is "Resistor Motstand Jumper Lask";
   attribute nokkelord of R4  : Label is "Resistor Motstand";
   attribute nokkelord of R3  : Label is "Resistor";
   attribute nokkelord of R2  : Label is "Resistor";
   attribute nokkelord of R1  : Label is "Resistor";
   attribute nokkelord of J3  : Label is "PDI Xmega programming";
   attribute nokkelord of D3  : Label is "SMD";
   attribute nokkelord of D2  : Label is "SMD";
   attribute nokkelord of D1  : Label is "SMD";
   attribute nokkelord of C11 : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C10 : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C9  : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C8  : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C7  : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C6  : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C5  : Label is "Capacitor Cap Kondis";
   attribute nokkelord of C4  : Label is "Capacitor Cap Kondis";
   attribute nokkelord of C3  : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C2  : Label is "Kondensator, Capacitor, CAP";
   attribute nokkelord of C1  : Label is "Kondensator, Capacitor, CAP";

   attribute pakke_opprettet : string;
   attribute pakke_opprettet of X1  : Label is "23.08.2014 20:10:40";
   attribute pakke_opprettet of U3  : Label is "11.07.2014 22:00:23";
   attribute pakke_opprettet of U2  : Label is "28.06.2014 18:39:37";
   attribute pakke_opprettet of U1  : Label is "13.07.2014 19:45:53";
   attribute pakke_opprettet of R14 : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R13 : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R12 : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R11 : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R10 : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R9  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R8  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R7  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R6  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R5  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R4  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R3  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R2  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R1  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of J3  : Label is "23.08.2014 13:37:50";
   attribute pakke_opprettet of J1  : Label is "13.11.2014 12:31:03";
   attribute pakke_opprettet of D3  : Label is "06.07.2014 18:55:44";
   attribute pakke_opprettet of D2  : Label is "06.07.2014 18:55:44";
   attribute pakke_opprettet of D1  : Label is "06.07.2014 18:55:44";
   attribute pakke_opprettet of C11 : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C10 : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C9  : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C8  : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C7  : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C6  : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C5  : Label is "11.02.2015 13:21:22";
   attribute pakke_opprettet of C4  : Label is "11.02.2015 13:21:22";
   attribute pakke_opprettet of C3  : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C2  : Label is "18.02.2015 14:18:13";
   attribute pakke_opprettet of C1  : Label is "18.02.2015 14:18:13";

   attribute pakke_opprettet_av : string;
   attribute pakke_opprettet_av of X1  : Label is "774";
   attribute pakke_opprettet_av of U3  : Label is "774";
   attribute pakke_opprettet_av of U2  : Label is "815";
   attribute pakke_opprettet_av of U1  : Label is "815";
   attribute pakke_opprettet_av of R14 : Label is "815";
   attribute pakke_opprettet_av of R13 : Label is "815";
   attribute pakke_opprettet_av of R12 : Label is "815";
   attribute pakke_opprettet_av of R11 : Label is "815";
   attribute pakke_opprettet_av of R10 : Label is "815";
   attribute pakke_opprettet_av of R9  : Label is "815";
   attribute pakke_opprettet_av of R8  : Label is "815";
   attribute pakke_opprettet_av of R7  : Label is "815";
   attribute pakke_opprettet_av of R6  : Label is "815";
   attribute pakke_opprettet_av of R5  : Label is "815";
   attribute pakke_opprettet_av of R4  : Label is "815";
   attribute pakke_opprettet_av of R3  : Label is "815";
   attribute pakke_opprettet_av of R2  : Label is "815";
   attribute pakke_opprettet_av of R1  : Label is "815";
   attribute pakke_opprettet_av of J3  : Label is "774";
   attribute pakke_opprettet_av of J1  : Label is "815";
   attribute pakke_opprettet_av of D3  : Label is "815";
   attribute pakke_opprettet_av of D2  : Label is "815";
   attribute pakke_opprettet_av of D1  : Label is "815";
   attribute pakke_opprettet_av of C11 : Label is "815";
   attribute pakke_opprettet_av of C10 : Label is "815";
   attribute pakke_opprettet_av of C9  : Label is "815";
   attribute pakke_opprettet_av of C8  : Label is "815";
   attribute pakke_opprettet_av of C7  : Label is "815";
   attribute pakke_opprettet_av of C6  : Label is "815";
   attribute pakke_opprettet_av of C5  : Label is "815";
   attribute pakke_opprettet_av of C4  : Label is "815";
   attribute pakke_opprettet_av of C3  : Label is "815";
   attribute pakke_opprettet_av of C2  : Label is "815";
   attribute pakke_opprettet_av of C1  : Label is "815";

   attribute pakketype : string;
   attribute pakketype of X1  : Label is "101";
   attribute pakketype of U3  : Label is "77";
   attribute pakketype of U2  : Label is "53";
   attribute pakketype of U1  : Label is "45";
   attribute pakketype of R14 : Label is "93";
   attribute pakketype of R13 : Label is "93";
   attribute pakketype of R12 : Label is "93";
   attribute pakketype of R11 : Label is "93";
   attribute pakketype of R10 : Label is "93";
   attribute pakketype of R9  : Label is "93";
   attribute pakketype of R8  : Label is "93";
   attribute pakketype of R7  : Label is "93";
   attribute pakketype of R6  : Label is "93";
   attribute pakketype of R5  : Label is "93";
   attribute pakketype of R4  : Label is "93";
   attribute pakketype of R3  : Label is "93";
   attribute pakketype of R2  : Label is "93";
   attribute pakketype of R1  : Label is "93";
   attribute pakketype of J3  : Label is "92";
   attribute pakketype of J1  : Label is "92";
   attribute pakketype of D3  : Label is "93";
   attribute pakketype of D2  : Label is "93";
   attribute pakketype of D1  : Label is "93";
   attribute pakketype of C11 : Label is "93";
   attribute pakketype of C10 : Label is "93";
   attribute pakketype of C9  : Label is "93";
   attribute pakketype of C8  : Label is "93";
   attribute pakketype of C7  : Label is "93";
   attribute pakketype of C6  : Label is "93";
   attribute pakketype of C5  : Label is "95";
   attribute pakketype of C4  : Label is "95";
   attribute pakketype of C3  : Label is "93";
   attribute pakketype of C2  : Label is "93";
   attribute pakketype of C1  : Label is "93";

   attribute pris : string;
   attribute pris of X1  : Label is "5";
   attribute pris of U3  : Label is "5";
   attribute pris of U2  : Label is "22";
   attribute pris of U1  : Label is "0";
   attribute pris of R14 : Label is "0";
   attribute pris of R13 : Label is "0";
   attribute pris of R12 : Label is "0";
   attribute pris of R11 : Label is "0";
   attribute pris of R10 : Label is "0";
   attribute pris of R9  : Label is "0";
   attribute pris of R8  : Label is "0";
   attribute pris of R7  : Label is "0";
   attribute pris of R6  : Label is "0";
   attribute pris of R5  : Label is "0";
   attribute pris of R4  : Label is "0";
   attribute pris of R3  : Label is "0";
   attribute pris of R2  : Label is "0";
   attribute pris of R1  : Label is "0";
   attribute pris of J3  : Label is "3";
   attribute pris of J1  : Label is "2";
   attribute pris of D3  : Label is "1";
   attribute pris of D2  : Label is "1";
   attribute pris of D1  : Label is "1";
   attribute pris of C11 : Label is "1";
   attribute pris of C10 : Label is "1";
   attribute pris of C9  : Label is "1";
   attribute pris of C8  : Label is "1";
   attribute pris of C7  : Label is "1";
   attribute pris of C6  : Label is "1";
   attribute pris of C5  : Label is "3";
   attribute pris of C4  : Label is "3";
   attribute pris of C3  : Label is "1";
   attribute pris of C2  : Label is "1";
   attribute pris of C1  : Label is "1";

   attribute produsent : string;
   attribute produsent of X1  : Label is "NDK";
   attribute produsent of U3  : Label is "Texas Instruments";
   attribute produsent of U2  : Label is "Exar";
   attribute produsent of U1  : Label is "Atmel";
   attribute produsent of R12 : Label is "Multikomp";
   attribute produsent of R11 : Label is "Multikomp";
   attribute produsent of R10 : Label is "Multikomp";
   attribute produsent of R9  : Label is "Multikomp";
   attribute produsent of R8  : Label is "Multikomp";
   attribute produsent of R7  : Label is "Multikomp";
   attribute produsent of R6  : Label is "Multikomp";
   attribute produsent of R5  : Label is "Multikomp";
   attribute produsent of J3  : Label is "OV";
   attribute produsent of D3  : Label is "Avago";
   attribute produsent of D2  : Label is "Avago";
   attribute produsent of D1  : Label is "Avago";
   attribute produsent of C5  : Label is "Kemet";
   attribute produsent of C4  : Label is "Kemet";

   attribute rad : string;
   attribute rad of X1  : Label is "0";
   attribute rad of U3  : Label is "2";
   attribute rad of U2  : Label is "4";
   attribute rad of U1  : Label is "4";
   attribute rad of R14 : Label is "-1";
   attribute rad of R13 : Label is "-1";
   attribute rad of R4  : Label is "-1";
   attribute rad of D3  : Label is "0";
   attribute rad of D2  : Label is "0";
   attribute rad of D1  : Label is "0";

   attribute rom : string;
   attribute rom of X1  : Label is "OV";
   attribute rom of U3  : Label is "OV";
   attribute rom of U2  : Label is "OV";
   attribute rom of U1  : Label is "OV";
   attribute rom of R14 : Label is "OV";
   attribute rom of R13 : Label is "OV";
   attribute rom of R4  : Label is "OV";
   attribute rom of D3  : Label is "OV";
   attribute rom of D2  : Label is "OV";
   attribute rom of D1  : Label is "OV";

   attribute symbol_opprettet : string;
   attribute symbol_opprettet of X1  : Label is "23.08.2014 20:23:20";
   attribute symbol_opprettet of U3  : Label is "11.07.2014 22:01:42";
   attribute symbol_opprettet of U2  : Label is "07.05.2015 21:29:51";
   attribute symbol_opprettet of U1  : Label is "07.05.2015 23:14:48";
   attribute symbol_opprettet of R14 : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R13 : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R12 : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R11 : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R10 : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R9  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R8  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R7  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R6  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R5  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R4  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R3  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R2  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R1  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of J3  : Label is "23.08.2014 13:38:30";
   attribute symbol_opprettet of J1  : Label is "13.11.2014 10:59:05";
   attribute symbol_opprettet of D3  : Label is "06.07.2014 18:59:47";
   attribute symbol_opprettet of D2  : Label is "06.07.2014 18:59:47";
   attribute symbol_opprettet of D1  : Label is "06.07.2014 18:59:47";
   attribute symbol_opprettet of C11 : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C10 : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C9  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C8  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C7  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C6  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C5  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C4  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C3  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C2  : Label is "14.11.2014 20:19:34";
   attribute symbol_opprettet of C1  : Label is "14.11.2014 20:19:34";

   attribute symbol_opprettet_av : string;
   attribute symbol_opprettet_av of X1  : Label is "774";
   attribute symbol_opprettet_av of U3  : Label is "774";
   attribute symbol_opprettet_av of U2  : Label is "815";
   attribute symbol_opprettet_av of U1  : Label is "815";
   attribute symbol_opprettet_av of R14 : Label is "oystesm";
   attribute symbol_opprettet_av of R13 : Label is "oystesm";
   attribute symbol_opprettet_av of R12 : Label is "oystesm";
   attribute symbol_opprettet_av of R11 : Label is "oystesm";
   attribute symbol_opprettet_av of R10 : Label is "oystesm";
   attribute symbol_opprettet_av of R9  : Label is "oystesm";
   attribute symbol_opprettet_av of R8  : Label is "oystesm";
   attribute symbol_opprettet_av of R7  : Label is "oystesm";
   attribute symbol_opprettet_av of R6  : Label is "oystesm";
   attribute symbol_opprettet_av of R5  : Label is "oystesm";
   attribute symbol_opprettet_av of R4  : Label is "oystesm";
   attribute symbol_opprettet_av of R3  : Label is "oystesm";
   attribute symbol_opprettet_av of R2  : Label is "oystesm";
   attribute symbol_opprettet_av of R1  : Label is "oystesm";
   attribute symbol_opprettet_av of J3  : Label is "774";
   attribute symbol_opprettet_av of J1  : Label is "815";
   attribute symbol_opprettet_av of D3  : Label is "815";
   attribute symbol_opprettet_av of D2  : Label is "815";
   attribute symbol_opprettet_av of D1  : Label is "815";
   attribute symbol_opprettet_av of C11 : Label is "815";
   attribute symbol_opprettet_av of C10 : Label is "815";
   attribute symbol_opprettet_av of C9  : Label is "815";
   attribute symbol_opprettet_av of C8  : Label is "815";
   attribute symbol_opprettet_av of C7  : Label is "815";
   attribute symbol_opprettet_av of C6  : Label is "815";
   attribute symbol_opprettet_av of C5  : Label is "815";
   attribute symbol_opprettet_av of C4  : Label is "815";
   attribute symbol_opprettet_av of C3  : Label is "815";
   attribute symbol_opprettet_av of C2  : Label is "815";
   attribute symbol_opprettet_av of C1  : Label is "815";


Begin
    X1 : NX5032GAMINUS16                                     -- ObjectKind=Part|PrimaryId=X1|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_X_59,                           -- ObjectKind=Pin|PrimaryId=X1-1
        X_2 => NamedIOSignal_X_58                            -- ObjectKind=Pin|PrimaryId=X1-2
      );

    U3 : TLV1117                                             -- ObjectKind=Part|PrimaryId=U3|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=U3-1
        X_2 => PowerSignal_VCC_P3V3,                         -- ObjectKind=Pin|PrimaryId=U3-2
        X_3 => PowerSignal_VCC_P5V0,                         -- ObjectKind=Pin|PrimaryId=U3-3
        X_4 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=U3-4
      );

    U2 : MAX3483                                             -- ObjectKind=Part|PrimaryId=U2|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_DMX_RD,                         -- ObjectKind=Pin|PrimaryId=U2-1
        X_2 => NamedIOSignal_DMX_DE,                         -- ObjectKind=Pin|PrimaryId=U2-2
        X_3 => NamedIOSignal_DMX_DE,                         -- ObjectKind=Pin|PrimaryId=U2-3
        X_4 => NamedIOSignal_DMX_DI,                         -- ObjectKind=Pin|PrimaryId=U2-4
        X_5 => PowerSignal_VCC_P3V3,                         -- ObjectKind=Pin|PrimaryId=U2-5
        X_6 => NamedSignal_DMXPLUS,                          -- ObjectKind=Pin|PrimaryId=U2-6
        X_7 => NamedSignal_DMXMINUS,                         -- ObjectKind=Pin|PrimaryId=U2-7
        X_8 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=U2-8
      );

    U1 : AVR_ATXMEGA64A3UMINUSAU                             -- ObjectKind=Part|PrimaryId=U1|SecondaryId=2
      Port Map
      (
        X_14 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=U1-14
        X_15 => PowerSignal_VCC_P3V3,                        -- ObjectKind=Pin|PrimaryId=U1-15
        X_24 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=U1-24
        X_25 => PowerSignal_VCC_P3V3,                        -- ObjectKind=Pin|PrimaryId=U1-25
        X_34 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=U1-34
        X_35 => PowerSignal_VCC_P3V3,                        -- ObjectKind=Pin|PrimaryId=U1-35
        X_44 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=U1-44
        X_45 => PowerSignal_VCC_P3V3,                        -- ObjectKind=Pin|PrimaryId=U1-45
        X_52 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=U1-52
        X_53 => PowerSignal_VCC_P3V3,                        -- ObjectKind=Pin|PrimaryId=U1-53
        X_60 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=U1-60
        X_61 => PowerSignal_VCC_P3V3                         -- ObjectKind=Pin|PrimaryId=U1-61
      );

    U1 : AVR_ATXMEGA64A3UMINUSAU                             -- ObjectKind=Part|PrimaryId=U1|SecondaryId=1
      Port Map
      (
        X_1  => NamedIOSignal_DEBUG_LED_3,                   -- ObjectKind=Pin|PrimaryId=U1-1
        X_2  => NamedIOSignal_X_2,                           -- ObjectKind=Pin|PrimaryId=U1-2
        X_3  => NamedIOSignal_X_3,                           -- ObjectKind=Pin|PrimaryId=U1-3
        X_4  => NamedIOSignal_X_4,                           -- ObjectKind=Pin|PrimaryId=U1-4
        X_5  => NamedIOSignal_X_5,                           -- ObjectKind=Pin|PrimaryId=U1-5
        X_6  => NamedIOSignal_PB0,                           -- ObjectKind=Pin|PrimaryId=U1-6
        X_7  => NamedIOSignal_PB1,                           -- ObjectKind=Pin|PrimaryId=U1-7
        X_8  => NamedIOSignal_PB2,                           -- ObjectKind=Pin|PrimaryId=U1-8
        X_9  => NamedIOSignal_PB3,                           -- ObjectKind=Pin|PrimaryId=U1-9
        X_10 => NamedIOSignal_PB4,                           -- ObjectKind=Pin|PrimaryId=U1-10
        X_11 => NamedIOSignal_PB5,                           -- ObjectKind=Pin|PrimaryId=U1-11
        X_12 => NamedIOSignal_PB6,                           -- ObjectKind=Pin|PrimaryId=U1-12
        X_13 => NamedIOSignal_PB7,                           -- ObjectKind=Pin|PrimaryId=U1-13
        X_16 => NamedIOSignal_KANAL_1MINUS1,                 -- ObjectKind=Pin|PrimaryId=U1-16
        X_17 => NamedIOSignal_KANAL_1MINUS2,                 -- ObjectKind=Pin|PrimaryId=U1-17
        X_18 => NamedIOSignal_KANAL_1MINUS3,                 -- ObjectKind=Pin|PrimaryId=U1-18
        X_19 => NamedIOSignal_KANAL_2MINUS1,                 -- ObjectKind=Pin|PrimaryId=U1-19
        X_20 => NamedIOSignal_KANAL_2MINUS2,                 -- ObjectKind=Pin|PrimaryId=U1-20
        X_21 => NamedIOSignal_KANAL_2MINUS3,                 -- ObjectKind=Pin|PrimaryId=U1-21
        X_22 => NamedIOSignal_BOKS_ID,                       -- ObjectKind=Pin|PrimaryId=U1-22
        X_23 => NamedIOSignal_BOKS_ID,                       -- ObjectKind=Pin|PrimaryId=U1-23
        X_26 => NamedIOSignal_KANAL_3MINUS1,                 -- ObjectKind=Pin|PrimaryId=U1-26
        X_27 => NamedIOSignal_KANAL_3MINUS2,                 -- ObjectKind=Pin|PrimaryId=U1-27
        X_28 => NamedIOSignal_KANAL_3MINUS3,                 -- ObjectKind=Pin|PrimaryId=U1-28
        X_29 => NamedIOSignal_X_29,                          -- ObjectKind=Pin|PrimaryId=U1-29
        X_30 => NamedIOSignal_X_30,                          -- ObjectKind=Pin|PrimaryId=U1-30
        X_31 => NamedIOSignal_DMX_DI,                        -- ObjectKind=Pin|PrimaryId=U1-31
        X_32 => NamedIOSignal_DMX_RD,                        -- ObjectKind=Pin|PrimaryId=U1-32
        X_33 => NamedIOSignal_DMX_DE,                        -- ObjectKind=Pin|PrimaryId=U1-33
        X_36 => NamedIOSignal_KANAL_6MINUS1,                 -- ObjectKind=Pin|PrimaryId=U1-36
        X_37 => NamedIOSignal_KANAL_6MINUS2,                 -- ObjectKind=Pin|PrimaryId=U1-37
        X_38 => NamedIOSignal_KANAL_6MINUS3,                 -- ObjectKind=Pin|PrimaryId=U1-38
        X_39 => NamedIOSignal_KANAL_5MINUS1,                 -- ObjectKind=Pin|PrimaryId=U1-39
        X_40 => NamedIOSignal_KANAL_5MINUS2,                 -- ObjectKind=Pin|PrimaryId=U1-40
        X_41 => NamedIOSignal_KANAL_5MINUS3,                 -- ObjectKind=Pin|PrimaryId=U1-41
        X_42 => NamedIOSignal_RXD1,                          -- ObjectKind=Pin|PrimaryId=U1-42
        X_43 => NamedIOSignal_TXD1,                          -- ObjectKind=Pin|PrimaryId=U1-43
        X_46 => NamedIOSignal_X_46,                          -- ObjectKind=Pin|PrimaryId=U1-46
        X_47 => NamedIOSignal_KANAL_4MINUS1,                 -- ObjectKind=Pin|PrimaryId=U1-47
        X_48 => NamedIOSignal_KANAL_4MINUS2,                 -- ObjectKind=Pin|PrimaryId=U1-48
        X_49 => NamedIOSignal_KANAL_4MINUS3,                 -- ObjectKind=Pin|PrimaryId=U1-49
        X_50 => NamedIOSignal_X_50,                          -- ObjectKind=Pin|PrimaryId=U1-50
        X_51 => NamedIOSignal_X_51,                          -- ObjectKind=Pin|PrimaryId=U1-51
        X_54 => NamedIOSignal_X_54,                          -- ObjectKind=Pin|PrimaryId=U1-54
        X_55 => NamedIOSignal_X_55,                          -- ObjectKind=Pin|PrimaryId=U1-55
        X_56 => NamedIOSignal_X_1,                           -- ObjectKind=Pin|PrimaryId=U1-56
        X_57 => PinSignal_J3_5,                              -- ObjectKind=Pin|PrimaryId=U1-57
        X_58 => NamedIOSignal_X_58,                          -- ObjectKind=Pin|PrimaryId=U1-58
        X_59 => NamedIOSignal_X_59,                          -- ObjectKind=Pin|PrimaryId=U1-59
        X_62 => NamedIOSignal_X_62,                          -- ObjectKind=Pin|PrimaryId=U1-62
        X_63 => NamedIOSignal_DEBUG_LED_1,                   -- ObjectKind=Pin|PrimaryId=U1-63
        X_64 => NamedIOSignal_DEBUG_LED_2                    -- ObjectKind=Pin|PrimaryId=U1-64
      );

    R14 : X_2384                                             -- ObjectKind=Part|PrimaryId=R14|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=R14-1
        X_2 => NamedIOSignal_X_4                             -- ObjectKind=Pin|PrimaryId=R14-2
      );

    R13 : X_2384                                             -- ObjectKind=Part|PrimaryId=R13|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_X_4,                            -- ObjectKind=Pin|PrimaryId=R13-1
        X_2 => PowerSignal_VCC_P5V0                          -- ObjectKind=Pin|PrimaryId=R13-2
      );

    R12 : X_3186                                             -- ObjectKind=Part|PrimaryId=R12|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB7,                            -- ObjectKind=Pin|PrimaryId=R12-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R12-2
      );

    R11 : X_3186                                             -- ObjectKind=Part|PrimaryId=R11|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB6,                            -- ObjectKind=Pin|PrimaryId=R11-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R11-2
      );

    R10 : X_3186                                             -- ObjectKind=Part|PrimaryId=R10|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB5,                            -- ObjectKind=Pin|PrimaryId=R10-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R10-2
      );

    R9 : X_3186                                              -- ObjectKind=Part|PrimaryId=R9|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB4,                            -- ObjectKind=Pin|PrimaryId=R9-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R9-2
      );

    R8 : X_3186                                              -- ObjectKind=Part|PrimaryId=R8|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB3,                            -- ObjectKind=Pin|PrimaryId=R8-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R8-2
      );

    R7 : X_3186                                              -- ObjectKind=Part|PrimaryId=R7|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB2,                            -- ObjectKind=Pin|PrimaryId=R7-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R7-2
      );

    R6 : X_3186                                              -- ObjectKind=Part|PrimaryId=R6|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB1,                            -- ObjectKind=Pin|PrimaryId=R6-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R6-2
      );

    R5 : X_3186                                              -- ObjectKind=Part|PrimaryId=R5|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_PB0,                            -- ObjectKind=Pin|PrimaryId=R5-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=R5-2
      );

    R4 : X_2444                                              -- ObjectKind=Part|PrimaryId=R4|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_J3_5,                               -- ObjectKind=Pin|PrimaryId=R4-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=R4-2
      );

    R3 : X_3050                                              -- ObjectKind=Part|PrimaryId=R3|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_DEBUG_LED_3,                    -- ObjectKind=Pin|PrimaryId=R3-1
        X_2 => PinSignal_D3_1                                -- ObjectKind=Pin|PrimaryId=R3-2
      );

    R2 : X_3050                                              -- ObjectKind=Part|PrimaryId=R2|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_DEBUG_LED_2,                    -- ObjectKind=Pin|PrimaryId=R2-1
        X_2 => PinSignal_D2_1                                -- ObjectKind=Pin|PrimaryId=R2-2
      );

    R1 : X_3050                                              -- ObjectKind=Part|PrimaryId=R1|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_DEBUG_LED_1,                    -- ObjectKind=Pin|PrimaryId=R1-1
        X_2 => PinSignal_D1_1                                -- ObjectKind=Pin|PrimaryId=R1-2
      );

    J3 : PDIMINUSHEADER                                      -- ObjectKind=Part|PrimaryId=J3|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_X_1,                            -- ObjectKind=Pin|PrimaryId=J3-1
        X_2 => PowerSignal_VCC_P3V3,                         -- ObjectKind=Pin|PrimaryId=J3-2
        X_5 => PinSignal_J3_5,                               -- ObjectKind=Pin|PrimaryId=J3-5
        X_6 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=J3-6
      );

    J2 : Lysreklameparalellbusskontakt2015                   -- ObjectKind=Part|PrimaryId=J2|SecondaryId=1
      Port Map
      (
        X_1  => NamedIOSignal_KANAL_1MINUS1,                 -- ObjectKind=Pin|PrimaryId=J2-1
        X_2  => NamedIOSignal_KANAL_1MINUS2,                 -- ObjectKind=Pin|PrimaryId=J2-2
        X_3  => NamedIOSignal_KANAL_1MINUS3,                 -- ObjectKind=Pin|PrimaryId=J2-3
        X_4  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-4
        X_5  => NamedIOSignal_KANAL_2MINUS1,                 -- ObjectKind=Pin|PrimaryId=J2-5
        X_6  => NamedIOSignal_KANAL_2MINUS2,                 -- ObjectKind=Pin|PrimaryId=J2-6
        X_7  => NamedIOSignal_KANAL_2MINUS3,                 -- ObjectKind=Pin|PrimaryId=J2-7
        X_8  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-8
        X_9  => NamedIOSignal_KANAL_3MINUS1,                 -- ObjectKind=Pin|PrimaryId=J2-9
        X_10 => NamedIOSignal_KANAL_3MINUS2,                 -- ObjectKind=Pin|PrimaryId=J2-10
        X_11 => NamedIOSignal_KANAL_3MINUS3,                 -- ObjectKind=Pin|PrimaryId=J2-11
        X_12 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-12
        X_13 => NamedIOSignal_KANAL_4MINUS1,                 -- ObjectKind=Pin|PrimaryId=J2-13
        X_14 => NamedIOSignal_KANAL_4MINUS2,                 -- ObjectKind=Pin|PrimaryId=J2-14
        X_15 => NamedIOSignal_KANAL_4MINUS3,                 -- ObjectKind=Pin|PrimaryId=J2-15
        X_16 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-16
        X_17 => NamedIOSignal_KANAL_5MINUS1,                 -- ObjectKind=Pin|PrimaryId=J2-17
        X_18 => NamedIOSignal_KANAL_5MINUS2,                 -- ObjectKind=Pin|PrimaryId=J2-18
        X_19 => NamedIOSignal_KANAL_5MINUS3,                 -- ObjectKind=Pin|PrimaryId=J2-19
        X_20 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-20
        X_21 => NamedIOSignal_KANAL_6MINUS1,                 -- ObjectKind=Pin|PrimaryId=J2-21
        X_22 => NamedIOSignal_KANAL_6MINUS2,                 -- ObjectKind=Pin|PrimaryId=J2-22
        X_23 => NamedIOSignal_KANAL_6MINUS3,                 -- ObjectKind=Pin|PrimaryId=J2-23
        X_24 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-24
        X_25 => PowerSignal_VCC_P5V0,                        -- ObjectKind=Pin|PrimaryId=J2-25
        X_26 => NamedIOSignal_BOKS_ID,                       -- ObjectKind=Pin|PrimaryId=J2-26
        X_27 => PowerSignal_VCC_P5V0,                        -- ObjectKind=Pin|PrimaryId=J2-27
        X_28 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-28
        X_29 => NamedSignal_DMXMINUS,                        -- ObjectKind=Pin|PrimaryId=J2-29
        X_30 => NamedSignal_DMXPLUS,                         -- ObjectKind=Pin|PrimaryId=J2-30
        X_31 => NamedSignal_DMXMINUS,                        -- ObjectKind=Pin|PrimaryId=J2-31
        X_32 => NamedSignal_DMXPLUS                          -- ObjectKind=Pin|PrimaryId=J2-32
      );

    J1 : Header_1x3                                          -- ObjectKind=Part|PrimaryId=J1|SecondaryId=1
      Port Map
      (
        X_1 => NamedIOSignal_RXD1,                           -- ObjectKind=Pin|PrimaryId=J1-1
        X_2 => NamedIOSignal_TXD1,                           -- ObjectKind=Pin|PrimaryId=J1-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=J1-3
      );

    D3 : SMD_LED_Red                                         -- ObjectKind=Part|PrimaryId=D3|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_D3_1,                               -- ObjectKind=Pin|PrimaryId=D3-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=D3-2
      );

    D2 : SMD_LED_Red                                         -- ObjectKind=Part|PrimaryId=D2|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_D2_1,                               -- ObjectKind=Pin|PrimaryId=D2-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=D2-2
      );

    D1 : SMD_LED_Red                                         -- ObjectKind=Part|PrimaryId=D1|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_D1_1,                               -- ObjectKind=Pin|PrimaryId=D1-1
        X_2 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=D1-2
      );

    C11 : X_100nF_0603                                       -- ObjectKind=Part|PrimaryId=C11|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C11-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=C11-2
      );

    C10 : X_100nF_0603                                       -- ObjectKind=Part|PrimaryId=C10|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C10-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=C10-2
      );

    C9 : X_100nF_0603                                        -- ObjectKind=Part|PrimaryId=C9|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C9-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=C9-2
      );

    C8 : X_100nF_0603                                        -- ObjectKind=Part|PrimaryId=C8|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C8-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=C8-2
      );

    C7 : X_100nF_0603                                        -- ObjectKind=Part|PrimaryId=C7|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C7-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=C7-2
      );

    C6 : X_100nF_0603                                        -- ObjectKind=Part|PrimaryId=C6|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C6-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=C6-2
      );

    C5 : X_22uF_1206                                         -- ObjectKind=Part|PrimaryId=C5|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C5-1
        X_2 => PowerSignal_VCC_P3V3                          -- ObjectKind=Pin|PrimaryId=C5-2
      );

    C4 : X_22uF_1206                                         -- ObjectKind=Part|PrimaryId=C4|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C4-1
        X_2 => PowerSignal_VCC_P5V0                          -- ObjectKind=Pin|PrimaryId=C4-2
      );

    C3 : X_8_2pF_0603                                        -- ObjectKind=Part|PrimaryId=C3|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C3-1
        X_2 => NamedIOSignal_X_58                            -- ObjectKind=Pin|PrimaryId=C3-2
      );

    C2 : X_8_2pF_0603                                        -- ObjectKind=Part|PrimaryId=C2|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C2-1
        X_2 => NamedIOSignal_X_59                            -- ObjectKind=Pin|PrimaryId=C2-2
      );

    C1 : X_100nF_0603                                        -- ObjectKind=Part|PrimaryId=C1|SecondaryId=1
      Port Map
      (
        X_1 => PowerSignal_GND,                              -- ObjectKind=Pin|PrimaryId=C1-1
        X_2 => PinSignal_J3_5                                -- ObjectKind=Pin|PrimaryId=C1-2
      );

    -- Signal Assignments
    ---------------------
    PowerSignal_GND <= '0'; -- ObjectKind=Net|PrimaryId=GND

End Structure;
------------------------------------------------------------

