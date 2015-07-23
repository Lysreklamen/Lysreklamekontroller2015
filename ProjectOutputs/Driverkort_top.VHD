------------------------------------------------------------
-- VHDL Driverkort_top
-- 2015 7 23 20 4 41
-- Created By "DXP VHDL Generator"
-- "Copyright (c) 2002-2014 Altium Limited"
-- Product Version: 15.0.7.36915
------------------------------------------------------------

------------------------------------------------------------
-- VHDL Driverkort_top
------------------------------------------------------------

Library IEEE;
Use     IEEE.std_logic_1164.all;

Entity Driverkort_top Is
  attribute MacroCell : boolean;

End Driverkort_top;
------------------------------------------------------------

------------------------------------------------------------
Architecture Structure Of Driverkort_top Is
   Component X_2DD2679                                       -- ObjectKind=Part|PrimaryId=Q1-1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=Q1-1-1
        X_2 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=Q1-1-2
        X_3 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=Q1-1-3
      );
   End Component;

   Component X_3180                                          -- ObjectKind=Part|PrimaryId=R1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=R1-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=R1-2
      );
   End Component;

   Component Lysreklameparalellbusskontakt2015               -- ObjectKind=Part|PrimaryId=J1|SecondaryId=1
      port
      (
        X_1  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-1
        X_2  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-2
        X_3  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-3
        X_4  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-4
        X_5  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-5
        X_6  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-6
        X_7  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-7
        X_8  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-8
        X_9  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-9
        X_10 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-10
        X_11 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-11
        X_12 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-12
        X_13 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-13
        X_14 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-14
        X_15 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-15
        X_16 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-16
        X_17 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-17
        X_18 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-18
        X_19 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-19
        X_20 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-20
        X_21 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-21
        X_22 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-22
        X_23 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-23
        X_24 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-24
        X_25 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-25
        X_26 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-26
        X_27 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-27
        X_28 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-28
        X_29 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-29
        X_30 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-30
        X_31 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J1-31
        X_32 : inout STD_LOGIC                               -- ObjectKind=Pin|PrimaryId=J1-32
      );
   End Component;


    Signal NamedSignal_DMXMINUS          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX-
    Signal NamedSignal_DMXMINUS_RETUR    : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX-_RETUR
    Signal NamedSignal_DMXPLUS           : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX+
    Signal NamedSignal_DMXPLUS_RETUR     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX+_RETUR
    Signal NamedSignal_KANAL_1MINUS1     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_1-1
    Signal NamedSignal_KANAL_1MINUS1_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_1-1_OUT
    Signal NamedSignal_KANAL_1MINUS2     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_1-2
    Signal NamedSignal_KANAL_1MINUS2_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_1-2_OUT
    Signal NamedSignal_KANAL_1MINUS3     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_1-3
    Signal NamedSignal_KANAL_1MINUS3_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_1-3_OUT
    Signal NamedSignal_KANAL_2MINUS1     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_2-1
    Signal NamedSignal_KANAL_2MINUS1_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_2-1_OUT
    Signal NamedSignal_KANAL_2MINUS2     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_2-2
    Signal NamedSignal_KANAL_2MINUS2_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_2-2_OUT
    Signal NamedSignal_KANAL_2MINUS3     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_2-3
    Signal NamedSignal_KANAL_2MINUS3_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_2-3_OUT
    Signal NamedSignal_KANAL_3MINUS1     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_3-1
    Signal NamedSignal_KANAL_3MINUS1_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_3-1_OUT
    Signal NamedSignal_KANAL_3MINUS2     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_3-2
    Signal NamedSignal_KANAL_3MINUS2_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_3-2_OUT
    Signal NamedSignal_KANAL_3MINUS3     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_3-3
    Signal NamedSignal_KANAL_3MINUS3_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_3-3_OUT
    Signal NamedSignal_KANAL_4MINUS1     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_4-1
    Signal NamedSignal_KANAL_4MINUS1_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_4-1_OUT
    Signal NamedSignal_KANAL_4MINUS2     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_4-2
    Signal NamedSignal_KANAL_4MINUS2_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_4-2_OUT
    Signal NamedSignal_KANAL_4MINUS3     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_4-3
    Signal NamedSignal_KANAL_4MINUS3_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_4-3_OUT
    Signal NamedSignal_KANAL_5MINUS1     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_5-1
    Signal NamedSignal_KANAL_5MINUS1_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_5-1_OUT
    Signal NamedSignal_KANAL_5MINUS2     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_5-2
    Signal NamedSignal_KANAL_5MINUS2_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_5-2_OUT
    Signal NamedSignal_KANAL_5MINUS3     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_5-3
    Signal NamedSignal_KANAL_5MINUS3_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_5-3_OUT
    Signal NamedSignal_KANAL_6MINUS1     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_6-1
    Signal NamedSignal_KANAL_6MINUS1_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_6-1_OUT
    Signal NamedSignal_KANAL_6MINUS2     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_6-2
    Signal NamedSignal_KANAL_6MINUS2_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_6-2_OUT
    Signal NamedSignal_KANAL_6MINUS3     : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_6-3
    Signal NamedSignal_KANAL_6MINUS3_OUT : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_6-3_OUT
    Signal PinSignal_Q1MINUS1_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ1-1_1
    Signal PinSignal_Q1MINUS2_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ1-2_1
    Signal PinSignal_Q1MINUS3_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ1-3_1
    Signal PinSignal_Q2MINUS1_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ2-1_1
    Signal PinSignal_Q2MINUS2_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ2-2_1
    Signal PinSignal_Q2MINUS3_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ2-3_1
    Signal PinSignal_Q3MINUS1_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ3-1_1
    Signal PinSignal_Q3MINUS2_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ3-2_1
    Signal PinSignal_Q3MINUS3_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ3-3_1
    Signal PinSignal_Q4MINUS1_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ4-1_1
    Signal PinSignal_Q4MINUS2_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ4-2_1
    Signal PinSignal_Q4MINUS3_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ4-3_1
    Signal PinSignal_Q5MINUS1_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ5-1_1
    Signal PinSignal_Q5MINUS2_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ5-2_1
    Signal PinSignal_Q5MINUS3_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ5-3_1
    Signal PinSignal_Q6MINUS1_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ6-1_1
    Signal PinSignal_Q6MINUS2_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ6-2_1
    Signal PinSignal_Q6MINUS3_1          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetQ6-3_1
    Signal PowerSignal_GND               : STD_LOGIC; -- ObjectKind=Net|PrimaryId=GND
    Signal PowerSignal_VCC_P5V0          : STD_LOGIC; -- ObjectKind=Net|PrimaryId=VCC_P5V0

   attribute Database_Table_Name : string;
   attribute Database_Table_Name of R18  : Label is "altium_Motstander";
   attribute Database_Table_Name of R17  : Label is "altium_Motstander";
   attribute Database_Table_Name of R16  : Label is "altium_Motstander";
   attribute Database_Table_Name of R15  : Label is "altium_Motstander";
   attribute Database_Table_Name of R14  : Label is "altium_Motstander";
   attribute Database_Table_Name of R13  : Label is "altium_Motstander";
   attribute Database_Table_Name of R12  : Label is "altium_Motstander";
   attribute Database_Table_Name of R11  : Label is "altium_Motstander";
   attribute Database_Table_Name of R10  : Label is "altium_Motstander";
   attribute Database_Table_Name of R9   : Label is "altium_Motstander";
   attribute Database_Table_Name of R8   : Label is "altium_Motstander";
   attribute Database_Table_Name of R7   : Label is "altium_Motstander";
   attribute Database_Table_Name of R6   : Label is "altium_Motstander";
   attribute Database_Table_Name of R5   : Label is "altium_Motstander";
   attribute Database_Table_Name of R4   : Label is "altium_Motstander";
   attribute Database_Table_Name of R3   : Label is "altium_Motstander";
   attribute Database_Table_Name of R2   : Label is "altium_Motstander";
   attribute Database_Table_Name of R1   : Label is "altium_Motstander";
   attribute Database_Table_Name of Q6-3 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q6-2 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q6-1 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q5-3 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q5-2 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q5-1 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q4-3 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q4-2 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q4-1 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q3-3 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q3-2 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q3-1 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q2-3 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q2-2 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q2-1 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q1-3 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q1-2 : Label is "altium_Transistorer";
   attribute Database_Table_Name of Q1-1 : Label is "altium_Transistorer";

   attribute Design_comment : string;
   attribute Design_comment of Q6-3 : Label is "";
   attribute Design_comment of Q6-2 : Label is "";
   attribute Design_comment of Q6-1 : Label is "";
   attribute Design_comment of Q5-3 : Label is "";
   attribute Design_comment of Q5-2 : Label is "";
   attribute Design_comment of Q5-1 : Label is "";
   attribute Design_comment of Q4-3 : Label is "";
   attribute Design_comment of Q4-2 : Label is "";
   attribute Design_comment of Q4-1 : Label is "";
   attribute Design_comment of Q3-3 : Label is "";
   attribute Design_comment of Q3-2 : Label is "";
   attribute Design_comment of Q3-1 : Label is "";
   attribute Design_comment of Q2-3 : Label is "";
   attribute Design_comment of Q2-2 : Label is "";
   attribute Design_comment of Q2-1 : Label is "";
   attribute Design_comment of Q1-3 : Label is "";
   attribute Design_comment of Q1-2 : Label is "";
   attribute Design_comment of Q1-1 : Label is "";

   attribute id : string;
   attribute id of R18  : Label is "3180";
   attribute id of R17  : Label is "3180";
   attribute id of R16  : Label is "3180";
   attribute id of R15  : Label is "3180";
   attribute id of R14  : Label is "3180";
   attribute id of R13  : Label is "3180";
   attribute id of R12  : Label is "3180";
   attribute id of R11  : Label is "3180";
   attribute id of R10  : Label is "3180";
   attribute id of R9   : Label is "3180";
   attribute id of R8   : Label is "3180";
   attribute id of R7   : Label is "3180";
   attribute id of R6   : Label is "3180";
   attribute id of R5   : Label is "3180";
   attribute id of R4   : Label is "3180";
   attribute id of R3   : Label is "3180";
   attribute id of R2   : Label is "3180";
   attribute id of R1   : Label is "3180";
   attribute id of Q6-3 : Label is "3047";
   attribute id of Q6-2 : Label is "3047";
   attribute id of Q6-1 : Label is "3047";
   attribute id of Q5-3 : Label is "3047";
   attribute id of Q5-2 : Label is "3047";
   attribute id of Q5-1 : Label is "3047";
   attribute id of Q4-3 : Label is "3047";
   attribute id of Q4-2 : Label is "3047";
   attribute id of Q4-1 : Label is "3047";
   attribute id of Q3-3 : Label is "3047";
   attribute id of Q3-2 : Label is "3047";
   attribute id of Q3-1 : Label is "3047";
   attribute id of Q2-3 : Label is "3047";
   attribute id of Q2-2 : Label is "3047";
   attribute id of Q2-1 : Label is "3047";
   attribute id of Q1-3 : Label is "3047";
   attribute id of Q1-2 : Label is "3047";
   attribute id of Q1-1 : Label is "3047";

   attribute leverandor : string;
   attribute leverandor of Q6-3 : Label is "Farnell";
   attribute leverandor of Q6-2 : Label is "Farnell";
   attribute leverandor of Q6-1 : Label is "Farnell";
   attribute leverandor of Q5-3 : Label is "Farnell";
   attribute leverandor of Q5-2 : Label is "Farnell";
   attribute leverandor of Q5-1 : Label is "Farnell";
   attribute leverandor of Q4-3 : Label is "Farnell";
   attribute leverandor of Q4-2 : Label is "Farnell";
   attribute leverandor of Q4-1 : Label is "Farnell";
   attribute leverandor of Q3-3 : Label is "Farnell";
   attribute leverandor of Q3-2 : Label is "Farnell";
   attribute leverandor of Q3-1 : Label is "Farnell";
   attribute leverandor of Q2-3 : Label is "Farnell";
   attribute leverandor of Q2-2 : Label is "Farnell";
   attribute leverandor of Q2-1 : Label is "Farnell";
   attribute leverandor of Q1-3 : Label is "Farnell";
   attribute leverandor of Q1-2 : Label is "Farnell";
   attribute leverandor of Q1-1 : Label is "Farnell";

   attribute leverandor_varenr : string;
   attribute leverandor_varenr of Q6-3 : Label is "1710681";
   attribute leverandor_varenr of Q6-2 : Label is "1710681";
   attribute leverandor_varenr of Q6-1 : Label is "1710681";
   attribute leverandor_varenr of Q5-3 : Label is "1710681";
   attribute leverandor_varenr of Q5-2 : Label is "1710681";
   attribute leverandor_varenr of Q5-1 : Label is "1710681";
   attribute leverandor_varenr of Q4-3 : Label is "1710681";
   attribute leverandor_varenr of Q4-2 : Label is "1710681";
   attribute leverandor_varenr of Q4-1 : Label is "1710681";
   attribute leverandor_varenr of Q3-3 : Label is "1710681";
   attribute leverandor_varenr of Q3-2 : Label is "1710681";
   attribute leverandor_varenr of Q3-1 : Label is "1710681";
   attribute leverandor_varenr of Q2-3 : Label is "1710681";
   attribute leverandor_varenr of Q2-2 : Label is "1710681";
   attribute leverandor_varenr of Q2-1 : Label is "1710681";
   attribute leverandor_varenr of Q1-3 : Label is "1710681";
   attribute leverandor_varenr of Q1-2 : Label is "1710681";
   attribute leverandor_varenr of Q1-1 : Label is "1710681";

   attribute navn : string;
   attribute navn of R18  : Label is "470R(0603)";
   attribute navn of R17  : Label is "470R(0603)";
   attribute navn of R16  : Label is "470R(0603)";
   attribute navn of R15  : Label is "470R(0603)";
   attribute navn of R14  : Label is "470R(0603)";
   attribute navn of R13  : Label is "470R(0603)";
   attribute navn of R12  : Label is "470R(0603)";
   attribute navn of R11  : Label is "470R(0603)";
   attribute navn of R10  : Label is "470R(0603)";
   attribute navn of R9   : Label is "470R(0603)";
   attribute navn of R8   : Label is "470R(0603)";
   attribute navn of R7   : Label is "470R(0603)";
   attribute navn of R6   : Label is "470R(0603)";
   attribute navn of R5   : Label is "470R(0603)";
   attribute navn of R4   : Label is "470R(0603)";
   attribute navn of R3   : Label is "470R(0603)";
   attribute navn of R2   : Label is "470R(0603)";
   attribute navn of R1   : Label is "470R(0603)";
   attribute navn of Q6-3 : Label is "2DD2679";
   attribute navn of Q6-2 : Label is "2DD2679";
   attribute navn of Q6-1 : Label is "2DD2679";
   attribute navn of Q5-3 : Label is "2DD2679";
   attribute navn of Q5-2 : Label is "2DD2679";
   attribute navn of Q5-1 : Label is "2DD2679";
   attribute navn of Q4-3 : Label is "2DD2679";
   attribute navn of Q4-2 : Label is "2DD2679";
   attribute navn of Q4-1 : Label is "2DD2679";
   attribute navn of Q3-3 : Label is "2DD2679";
   attribute navn of Q3-2 : Label is "2DD2679";
   attribute navn of Q3-1 : Label is "2DD2679";
   attribute navn of Q2-3 : Label is "2DD2679";
   attribute navn of Q2-2 : Label is "2DD2679";
   attribute navn of Q2-1 : Label is "2DD2679";
   attribute navn of Q1-3 : Label is "2DD2679";
   attribute navn of Q1-2 : Label is "2DD2679";
   attribute navn of Q1-1 : Label is "2DD2679";

   attribute nokkelord : string;
   attribute nokkelord of R18  : Label is "Resistor, Motstand";
   attribute nokkelord of R17  : Label is "Resistor, Motstand";
   attribute nokkelord of R16  : Label is "Resistor, Motstand";
   attribute nokkelord of R15  : Label is "Resistor, Motstand";
   attribute nokkelord of R14  : Label is "Resistor, Motstand";
   attribute nokkelord of R13  : Label is "Resistor, Motstand";
   attribute nokkelord of R12  : Label is "Resistor, Motstand";
   attribute nokkelord of R11  : Label is "Resistor, Motstand";
   attribute nokkelord of R10  : Label is "Resistor, Motstand";
   attribute nokkelord of R9   : Label is "Resistor, Motstand";
   attribute nokkelord of R8   : Label is "Resistor, Motstand";
   attribute nokkelord of R7   : Label is "Resistor, Motstand";
   attribute nokkelord of R6   : Label is "Resistor, Motstand";
   attribute nokkelord of R5   : Label is "Resistor, Motstand";
   attribute nokkelord of R4   : Label is "Resistor, Motstand";
   attribute nokkelord of R3   : Label is "Resistor, Motstand";
   attribute nokkelord of R2   : Label is "Resistor, Motstand";
   attribute nokkelord of R1   : Label is "Resistor, Motstand";
   attribute nokkelord of Q6-3 : Label is "NPN";
   attribute nokkelord of Q6-2 : Label is "NPN";
   attribute nokkelord of Q6-1 : Label is "NPN";
   attribute nokkelord of Q5-3 : Label is "NPN";
   attribute nokkelord of Q5-2 : Label is "NPN";
   attribute nokkelord of Q5-1 : Label is "NPN";
   attribute nokkelord of Q4-3 : Label is "NPN";
   attribute nokkelord of Q4-2 : Label is "NPN";
   attribute nokkelord of Q4-1 : Label is "NPN";
   attribute nokkelord of Q3-3 : Label is "NPN";
   attribute nokkelord of Q3-2 : Label is "NPN";
   attribute nokkelord of Q3-1 : Label is "NPN";
   attribute nokkelord of Q2-3 : Label is "NPN";
   attribute nokkelord of Q2-2 : Label is "NPN";
   attribute nokkelord of Q2-1 : Label is "NPN";
   attribute nokkelord of Q1-3 : Label is "NPN";
   attribute nokkelord of Q1-2 : Label is "NPN";
   attribute nokkelord of Q1-1 : Label is "NPN";

   attribute pakke_opprettet : string;
   attribute pakke_opprettet of R18  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R17  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R16  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R15  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R14  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R13  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R12  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R11  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R10  : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R9   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R8   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R7   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R6   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R5   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R4   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R3   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R2   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of R1   : Label is "08.07.2014 21:15:30";
   attribute pakke_opprettet of Q6-3 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q6-2 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q6-1 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q5-3 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q5-2 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q5-1 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q4-3 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q4-2 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q4-1 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q3-3 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q3-2 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q3-1 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q2-3 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q2-2 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q2-1 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q1-3 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q1-2 : Label is "07.05.2015 20:46:56";
   attribute pakke_opprettet of Q1-1 : Label is "07.05.2015 20:46:56";

   attribute pakke_opprettet_av : string;
   attribute pakke_opprettet_av of R18  : Label is "815";
   attribute pakke_opprettet_av of R17  : Label is "815";
   attribute pakke_opprettet_av of R16  : Label is "815";
   attribute pakke_opprettet_av of R15  : Label is "815";
   attribute pakke_opprettet_av of R14  : Label is "815";
   attribute pakke_opprettet_av of R13  : Label is "815";
   attribute pakke_opprettet_av of R12  : Label is "815";
   attribute pakke_opprettet_av of R11  : Label is "815";
   attribute pakke_opprettet_av of R10  : Label is "815";
   attribute pakke_opprettet_av of R9   : Label is "815";
   attribute pakke_opprettet_av of R8   : Label is "815";
   attribute pakke_opprettet_av of R7   : Label is "815";
   attribute pakke_opprettet_av of R6   : Label is "815";
   attribute pakke_opprettet_av of R5   : Label is "815";
   attribute pakke_opprettet_av of R4   : Label is "815";
   attribute pakke_opprettet_av of R3   : Label is "815";
   attribute pakke_opprettet_av of R2   : Label is "815";
   attribute pakke_opprettet_av of R1   : Label is "815";
   attribute pakke_opprettet_av of Q6-3 : Label is "815";
   attribute pakke_opprettet_av of Q6-2 : Label is "815";
   attribute pakke_opprettet_av of Q6-1 : Label is "815";
   attribute pakke_opprettet_av of Q5-3 : Label is "815";
   attribute pakke_opprettet_av of Q5-2 : Label is "815";
   attribute pakke_opprettet_av of Q5-1 : Label is "815";
   attribute pakke_opprettet_av of Q4-3 : Label is "815";
   attribute pakke_opprettet_av of Q4-2 : Label is "815";
   attribute pakke_opprettet_av of Q4-1 : Label is "815";
   attribute pakke_opprettet_av of Q3-3 : Label is "815";
   attribute pakke_opprettet_av of Q3-2 : Label is "815";
   attribute pakke_opprettet_av of Q3-1 : Label is "815";
   attribute pakke_opprettet_av of Q2-3 : Label is "815";
   attribute pakke_opprettet_av of Q2-2 : Label is "815";
   attribute pakke_opprettet_av of Q2-1 : Label is "815";
   attribute pakke_opprettet_av of Q1-3 : Label is "815";
   attribute pakke_opprettet_av of Q1-2 : Label is "815";
   attribute pakke_opprettet_av of Q1-1 : Label is "815";

   attribute pakketype : string;
   attribute pakketype of R18  : Label is "93";
   attribute pakketype of R17  : Label is "93";
   attribute pakketype of R16  : Label is "93";
   attribute pakketype of R15  : Label is "93";
   attribute pakketype of R14  : Label is "93";
   attribute pakketype of R13  : Label is "93";
   attribute pakketype of R12  : Label is "93";
   attribute pakketype of R11  : Label is "93";
   attribute pakketype of R10  : Label is "93";
   attribute pakketype of R9   : Label is "93";
   attribute pakketype of R8   : Label is "93";
   attribute pakketype of R7   : Label is "93";
   attribute pakketype of R6   : Label is "93";
   attribute pakketype of R5   : Label is "93";
   attribute pakketype of R4   : Label is "93";
   attribute pakketype of R3   : Label is "93";
   attribute pakketype of R2   : Label is "93";
   attribute pakketype of R1   : Label is "93";
   attribute pakketype of Q6-3 : Label is "77";
   attribute pakketype of Q6-2 : Label is "77";
   attribute pakketype of Q6-1 : Label is "77";
   attribute pakketype of Q5-3 : Label is "77";
   attribute pakketype of Q5-2 : Label is "77";
   attribute pakketype of Q5-1 : Label is "77";
   attribute pakketype of Q4-3 : Label is "77";
   attribute pakketype of Q4-2 : Label is "77";
   attribute pakketype of Q4-1 : Label is "77";
   attribute pakketype of Q3-3 : Label is "77";
   attribute pakketype of Q3-2 : Label is "77";
   attribute pakketype of Q3-1 : Label is "77";
   attribute pakketype of Q2-3 : Label is "77";
   attribute pakketype of Q2-2 : Label is "77";
   attribute pakketype of Q2-1 : Label is "77";
   attribute pakketype of Q1-3 : Label is "77";
   attribute pakketype of Q1-2 : Label is "77";
   attribute pakketype of Q1-1 : Label is "77";

   attribute pris : string;
   attribute pris of R18  : Label is "0";
   attribute pris of R17  : Label is "0";
   attribute pris of R16  : Label is "0";
   attribute pris of R15  : Label is "0";
   attribute pris of R14  : Label is "0";
   attribute pris of R13  : Label is "0";
   attribute pris of R12  : Label is "0";
   attribute pris of R11  : Label is "0";
   attribute pris of R10  : Label is "0";
   attribute pris of R9   : Label is "0";
   attribute pris of R8   : Label is "0";
   attribute pris of R7   : Label is "0";
   attribute pris of R6   : Label is "0";
   attribute pris of R5   : Label is "0";
   attribute pris of R4   : Label is "0";
   attribute pris of R3   : Label is "0";
   attribute pris of R2   : Label is "0";
   attribute pris of R1   : Label is "0";
   attribute pris of Q6-3 : Label is "4";
   attribute pris of Q6-2 : Label is "4";
   attribute pris of Q6-1 : Label is "4";
   attribute pris of Q5-3 : Label is "4";
   attribute pris of Q5-2 : Label is "4";
   attribute pris of Q5-1 : Label is "4";
   attribute pris of Q4-3 : Label is "4";
   attribute pris of Q4-2 : Label is "4";
   attribute pris of Q4-1 : Label is "4";
   attribute pris of Q3-3 : Label is "4";
   attribute pris of Q3-2 : Label is "4";
   attribute pris of Q3-1 : Label is "4";
   attribute pris of Q2-3 : Label is "4";
   attribute pris of Q2-2 : Label is "4";
   attribute pris of Q2-1 : Label is "4";
   attribute pris of Q1-3 : Label is "4";
   attribute pris of Q1-2 : Label is "4";
   attribute pris of Q1-1 : Label is "4";

   attribute produsent : string;
   attribute produsent of Q6-3 : Label is "Diodes";
   attribute produsent of Q6-2 : Label is "Diodes";
   attribute produsent of Q6-1 : Label is "Diodes";
   attribute produsent of Q5-3 : Label is "Diodes";
   attribute produsent of Q5-2 : Label is "Diodes";
   attribute produsent of Q5-1 : Label is "Diodes";
   attribute produsent of Q4-3 : Label is "Diodes";
   attribute produsent of Q4-2 : Label is "Diodes";
   attribute produsent of Q4-1 : Label is "Diodes";
   attribute produsent of Q3-3 : Label is "Diodes";
   attribute produsent of Q3-2 : Label is "Diodes";
   attribute produsent of Q3-1 : Label is "Diodes";
   attribute produsent of Q2-3 : Label is "Diodes";
   attribute produsent of Q2-2 : Label is "Diodes";
   attribute produsent of Q2-1 : Label is "Diodes";
   attribute produsent of Q1-3 : Label is "Diodes";
   attribute produsent of Q1-2 : Label is "Diodes";
   attribute produsent of Q1-1 : Label is "Diodes";

   attribute Status : string;
   attribute Status of Q6-3 : Label is "New";
   attribute Status of Q6-2 : Label is "New";
   attribute Status of Q6-1 : Label is "New";
   attribute Status of Q5-3 : Label is "New";
   attribute Status of Q5-2 : Label is "New";
   attribute Status of Q5-1 : Label is "New";
   attribute Status of Q4-3 : Label is "New";
   attribute Status of Q4-2 : Label is "New";
   attribute Status of Q4-1 : Label is "New";
   attribute Status of Q3-3 : Label is "New";
   attribute Status of Q3-2 : Label is "New";
   attribute Status of Q3-1 : Label is "New";
   attribute Status of Q2-3 : Label is "New";
   attribute Status of Q2-2 : Label is "New";
   attribute Status of Q2-1 : Label is "New";
   attribute Status of Q1-3 : Label is "New";
   attribute Status of Q1-2 : Label is "New";
   attribute Status of Q1-1 : Label is "New";

   attribute symbol_opprettet : string;
   attribute symbol_opprettet of R18  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R17  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R16  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R15  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R14  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R13  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R12  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R11  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R10  : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R9   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R8   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R7   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R6   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R5   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R4   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R3   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R2   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of R1   : Label is "08.07.2014 21:16:33";
   attribute symbol_opprettet of Q6-3 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q6-2 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q6-1 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q5-3 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q5-2 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q5-1 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q4-3 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q4-2 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q4-1 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q3-3 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q3-2 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q3-1 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q2-3 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q2-2 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q2-1 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q1-3 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q1-2 : Label is "07.05.2015 21:04:46";
   attribute symbol_opprettet of Q1-1 : Label is "07.05.2015 21:04:46";

   attribute symbol_opprettet_av : string;
   attribute symbol_opprettet_av of R18  : Label is "oystesm";
   attribute symbol_opprettet_av of R17  : Label is "oystesm";
   attribute symbol_opprettet_av of R16  : Label is "oystesm";
   attribute symbol_opprettet_av of R15  : Label is "oystesm";
   attribute symbol_opprettet_av of R14  : Label is "oystesm";
   attribute symbol_opprettet_av of R13  : Label is "oystesm";
   attribute symbol_opprettet_av of R12  : Label is "oystesm";
   attribute symbol_opprettet_av of R11  : Label is "oystesm";
   attribute symbol_opprettet_av of R10  : Label is "oystesm";
   attribute symbol_opprettet_av of R9   : Label is "oystesm";
   attribute symbol_opprettet_av of R8   : Label is "oystesm";
   attribute symbol_opprettet_av of R7   : Label is "oystesm";
   attribute symbol_opprettet_av of R6   : Label is "oystesm";
   attribute symbol_opprettet_av of R5   : Label is "oystesm";
   attribute symbol_opprettet_av of R4   : Label is "oystesm";
   attribute symbol_opprettet_av of R3   : Label is "oystesm";
   attribute symbol_opprettet_av of R2   : Label is "oystesm";
   attribute symbol_opprettet_av of R1   : Label is "oystesm";
   attribute symbol_opprettet_av of Q6-3 : Label is "815";
   attribute symbol_opprettet_av of Q6-2 : Label is "815";
   attribute symbol_opprettet_av of Q6-1 : Label is "815";
   attribute symbol_opprettet_av of Q5-3 : Label is "815";
   attribute symbol_opprettet_av of Q5-2 : Label is "815";
   attribute symbol_opprettet_av of Q5-1 : Label is "815";
   attribute symbol_opprettet_av of Q4-3 : Label is "815";
   attribute symbol_opprettet_av of Q4-2 : Label is "815";
   attribute symbol_opprettet_av of Q4-1 : Label is "815";
   attribute symbol_opprettet_av of Q3-3 : Label is "815";
   attribute symbol_opprettet_av of Q3-2 : Label is "815";
   attribute symbol_opprettet_av of Q3-1 : Label is "815";
   attribute symbol_opprettet_av of Q2-3 : Label is "815";
   attribute symbol_opprettet_av of Q2-2 : Label is "815";
   attribute symbol_opprettet_av of Q2-1 : Label is "815";
   attribute symbol_opprettet_av of Q1-3 : Label is "815";
   attribute symbol_opprettet_av of Q1-2 : Label is "815";
   attribute symbol_opprettet_av of Q1-1 : Label is "815";

   attribute Verified_by : string;
   attribute Verified_by of Q6-3 : Label is "";
   attribute Verified_by of Q6-2 : Label is "";
   attribute Verified_by of Q6-1 : Label is "";
   attribute Verified_by of Q5-3 : Label is "";
   attribute Verified_by of Q5-2 : Label is "";
   attribute Verified_by of Q5-1 : Label is "";
   attribute Verified_by of Q4-3 : Label is "";
   attribute Verified_by of Q4-2 : Label is "";
   attribute Verified_by of Q4-1 : Label is "";
   attribute Verified_by of Q3-3 : Label is "";
   attribute Verified_by of Q3-2 : Label is "";
   attribute Verified_by of Q3-1 : Label is "";
   attribute Verified_by of Q2-3 : Label is "";
   attribute Verified_by of Q2-2 : Label is "";
   attribute Verified_by of Q2-1 : Label is "";
   attribute Verified_by of Q1-3 : Label is "";
   attribute Verified_by of Q1-2 : Label is "";
   attribute Verified_by of Q1-1 : Label is "";

   attribute Verified_date : string;
   attribute Verified_date of Q6-3 : Label is "";
   attribute Verified_date of Q6-2 : Label is "";
   attribute Verified_date of Q6-1 : Label is "";
   attribute Verified_date of Q5-3 : Label is "";
   attribute Verified_date of Q5-2 : Label is "";
   attribute Verified_date of Q5-1 : Label is "";
   attribute Verified_date of Q4-3 : Label is "";
   attribute Verified_date of Q4-2 : Label is "";
   attribute Verified_date of Q4-1 : Label is "";
   attribute Verified_date of Q3-3 : Label is "";
   attribute Verified_date of Q3-2 : Label is "";
   attribute Verified_date of Q3-1 : Label is "";
   attribute Verified_date of Q2-3 : Label is "";
   attribute Verified_date of Q2-2 : Label is "";
   attribute Verified_date of Q2-1 : Label is "";
   attribute Verified_date of Q1-3 : Label is "";
   attribute Verified_date of Q1-2 : Label is "";
   attribute Verified_date of Q1-1 : Label is "";


Begin
    R18 : X_3180                                             -- ObjectKind=Part|PrimaryId=R18|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_6MINUS3,                    -- ObjectKind=Pin|PrimaryId=R18-1
        X_2 => PinSignal_Q6MINUS3_1                          -- ObjectKind=Pin|PrimaryId=R18-2
      );

    R17 : X_3180                                             -- ObjectKind=Part|PrimaryId=R17|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_6MINUS2,                    -- ObjectKind=Pin|PrimaryId=R17-1
        X_2 => PinSignal_Q6MINUS2_1                          -- ObjectKind=Pin|PrimaryId=R17-2
      );

    R16 : X_3180                                             -- ObjectKind=Part|PrimaryId=R16|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_6MINUS1,                    -- ObjectKind=Pin|PrimaryId=R16-1
        X_2 => PinSignal_Q6MINUS1_1                          -- ObjectKind=Pin|PrimaryId=R16-2
      );

    R15 : X_3180                                             -- ObjectKind=Part|PrimaryId=R15|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_5MINUS3,                    -- ObjectKind=Pin|PrimaryId=R15-1
        X_2 => PinSignal_Q5MINUS3_1                          -- ObjectKind=Pin|PrimaryId=R15-2
      );

    R14 : X_3180                                             -- ObjectKind=Part|PrimaryId=R14|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_5MINUS2,                    -- ObjectKind=Pin|PrimaryId=R14-1
        X_2 => PinSignal_Q5MINUS2_1                          -- ObjectKind=Pin|PrimaryId=R14-2
      );

    R13 : X_3180                                             -- ObjectKind=Part|PrimaryId=R13|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_5MINUS1,                    -- ObjectKind=Pin|PrimaryId=R13-1
        X_2 => PinSignal_Q5MINUS1_1                          -- ObjectKind=Pin|PrimaryId=R13-2
      );

    R12 : X_3180                                             -- ObjectKind=Part|PrimaryId=R12|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_4MINUS3,                    -- ObjectKind=Pin|PrimaryId=R12-1
        X_2 => PinSignal_Q4MINUS3_1                          -- ObjectKind=Pin|PrimaryId=R12-2
      );

    R11 : X_3180                                             -- ObjectKind=Part|PrimaryId=R11|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_4MINUS2,                    -- ObjectKind=Pin|PrimaryId=R11-1
        X_2 => PinSignal_Q4MINUS2_1                          -- ObjectKind=Pin|PrimaryId=R11-2
      );

    R10 : X_3180                                             -- ObjectKind=Part|PrimaryId=R10|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_4MINUS1,                    -- ObjectKind=Pin|PrimaryId=R10-1
        X_2 => PinSignal_Q4MINUS1_1                          -- ObjectKind=Pin|PrimaryId=R10-2
      );

    R9 : X_3180                                              -- ObjectKind=Part|PrimaryId=R9|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_3MINUS3,                    -- ObjectKind=Pin|PrimaryId=R9-1
        X_2 => PinSignal_Q3MINUS3_1                          -- ObjectKind=Pin|PrimaryId=R9-2
      );

    R8 : X_3180                                              -- ObjectKind=Part|PrimaryId=R8|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_3MINUS2,                    -- ObjectKind=Pin|PrimaryId=R8-1
        X_2 => PinSignal_Q3MINUS2_1                          -- ObjectKind=Pin|PrimaryId=R8-2
      );

    R7 : X_3180                                              -- ObjectKind=Part|PrimaryId=R7|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_3MINUS1,                    -- ObjectKind=Pin|PrimaryId=R7-1
        X_2 => PinSignal_Q3MINUS1_1                          -- ObjectKind=Pin|PrimaryId=R7-2
      );

    R6 : X_3180                                              -- ObjectKind=Part|PrimaryId=R6|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_2MINUS3,                    -- ObjectKind=Pin|PrimaryId=R6-1
        X_2 => PinSignal_Q2MINUS3_1                          -- ObjectKind=Pin|PrimaryId=R6-2
      );

    R5 : X_3180                                              -- ObjectKind=Part|PrimaryId=R5|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_2MINUS2,                    -- ObjectKind=Pin|PrimaryId=R5-1
        X_2 => PinSignal_Q2MINUS2_1                          -- ObjectKind=Pin|PrimaryId=R5-2
      );

    R4 : X_3180                                              -- ObjectKind=Part|PrimaryId=R4|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_2MINUS1,                    -- ObjectKind=Pin|PrimaryId=R4-1
        X_2 => PinSignal_Q2MINUS1_1                          -- ObjectKind=Pin|PrimaryId=R4-2
      );

    R3 : X_3180                                              -- ObjectKind=Part|PrimaryId=R3|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_1MINUS3,                    -- ObjectKind=Pin|PrimaryId=R3-1
        X_2 => PinSignal_Q1MINUS3_1                          -- ObjectKind=Pin|PrimaryId=R3-2
      );

    R2 : X_3180                                              -- ObjectKind=Part|PrimaryId=R2|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_1MINUS2,                    -- ObjectKind=Pin|PrimaryId=R2-1
        X_2 => PinSignal_Q1MINUS2_1                          -- ObjectKind=Pin|PrimaryId=R2-2
      );

    R1 : X_3180                                              -- ObjectKind=Part|PrimaryId=R1|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_1MINUS1,                    -- ObjectKind=Pin|PrimaryId=R1-1
        X_2 => PinSignal_Q1MINUS1_1                          -- ObjectKind=Pin|PrimaryId=R1-2
      );

    Q6MINUS3 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q6-3|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q6MINUS3_1,                         -- ObjectKind=Pin|PrimaryId=Q6-3-1
        X_2 => NamedSignal_KANAL_6MINUS3_OUT,                -- ObjectKind=Pin|PrimaryId=Q6-3-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q6-3-3
      );

    Q6MINUS2 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q6-2|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q6MINUS2_1,                         -- ObjectKind=Pin|PrimaryId=Q6-2-1
        X_2 => NamedSignal_KANAL_6MINUS2_OUT,                -- ObjectKind=Pin|PrimaryId=Q6-2-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q6-2-3
      );

    Q6MINUS1 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q6-1|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q6MINUS1_1,                         -- ObjectKind=Pin|PrimaryId=Q6-1-1
        X_2 => NamedSignal_KANAL_6MINUS1_OUT,                -- ObjectKind=Pin|PrimaryId=Q6-1-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q6-1-3
      );

    Q5MINUS3 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q5-3|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q5MINUS3_1,                         -- ObjectKind=Pin|PrimaryId=Q5-3-1
        X_2 => NamedSignal_KANAL_5MINUS3_OUT,                -- ObjectKind=Pin|PrimaryId=Q5-3-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q5-3-3
      );

    Q5MINUS2 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q5-2|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q5MINUS2_1,                         -- ObjectKind=Pin|PrimaryId=Q5-2-1
        X_2 => NamedSignal_KANAL_5MINUS2_OUT,                -- ObjectKind=Pin|PrimaryId=Q5-2-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q5-2-3
      );

    Q5MINUS1 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q5-1|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q5MINUS1_1,                         -- ObjectKind=Pin|PrimaryId=Q5-1-1
        X_2 => NamedSignal_KANAL_5MINUS1_OUT,                -- ObjectKind=Pin|PrimaryId=Q5-1-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q5-1-3
      );

    Q4MINUS3 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q4-3|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q4MINUS3_1,                         -- ObjectKind=Pin|PrimaryId=Q4-3-1
        X_2 => NamedSignal_KANAL_4MINUS3_OUT,                -- ObjectKind=Pin|PrimaryId=Q4-3-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q4-3-3
      );

    Q4MINUS2 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q4-2|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q4MINUS2_1,                         -- ObjectKind=Pin|PrimaryId=Q4-2-1
        X_2 => NamedSignal_KANAL_4MINUS2_OUT,                -- ObjectKind=Pin|PrimaryId=Q4-2-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q4-2-3
      );

    Q4MINUS1 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q4-1|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q4MINUS1_1,                         -- ObjectKind=Pin|PrimaryId=Q4-1-1
        X_2 => NamedSignal_KANAL_4MINUS1_OUT,                -- ObjectKind=Pin|PrimaryId=Q4-1-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q4-1-3
      );

    Q3MINUS3 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q3-3|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q3MINUS3_1,                         -- ObjectKind=Pin|PrimaryId=Q3-3-1
        X_2 => NamedSignal_KANAL_3MINUS3_OUT,                -- ObjectKind=Pin|PrimaryId=Q3-3-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q3-3-3
      );

    Q3MINUS2 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q3-2|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q3MINUS2_1,                         -- ObjectKind=Pin|PrimaryId=Q3-2-1
        X_2 => NamedSignal_KANAL_3MINUS2_OUT,                -- ObjectKind=Pin|PrimaryId=Q3-2-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q3-2-3
      );

    Q3MINUS1 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q3-1|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q3MINUS1_1,                         -- ObjectKind=Pin|PrimaryId=Q3-1-1
        X_2 => NamedSignal_KANAL_3MINUS1_OUT,                -- ObjectKind=Pin|PrimaryId=Q3-1-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q3-1-3
      );

    Q2MINUS3 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q2-3|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q2MINUS3_1,                         -- ObjectKind=Pin|PrimaryId=Q2-3-1
        X_2 => NamedSignal_KANAL_2MINUS3_OUT,                -- ObjectKind=Pin|PrimaryId=Q2-3-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q2-3-3
      );

    Q2MINUS2 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q2-2|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q2MINUS2_1,                         -- ObjectKind=Pin|PrimaryId=Q2-2-1
        X_2 => NamedSignal_KANAL_2MINUS2_OUT,                -- ObjectKind=Pin|PrimaryId=Q2-2-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q2-2-3
      );

    Q2MINUS1 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q2-1|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q2MINUS1_1,                         -- ObjectKind=Pin|PrimaryId=Q2-1-1
        X_2 => NamedSignal_KANAL_2MINUS1_OUT,                -- ObjectKind=Pin|PrimaryId=Q2-1-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q2-1-3
      );

    Q1MINUS3 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q1-3|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q1MINUS3_1,                         -- ObjectKind=Pin|PrimaryId=Q1-3-1
        X_2 => NamedSignal_KANAL_1MINUS3_OUT,                -- ObjectKind=Pin|PrimaryId=Q1-3-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q1-3-3
      );

    Q1MINUS2 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q1-2|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q1MINUS2_1,                         -- ObjectKind=Pin|PrimaryId=Q1-2-1
        X_2 => NamedSignal_KANAL_1MINUS2_OUT,                -- ObjectKind=Pin|PrimaryId=Q1-2-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q1-2-3
      );

    Q1MINUS1 : X_2DD2679                                     -- ObjectKind=Part|PrimaryId=Q1-1|SecondaryId=1
      Port Map
      (
        X_1 => PinSignal_Q1MINUS1_1,                         -- ObjectKind=Pin|PrimaryId=Q1-1-1
        X_2 => NamedSignal_KANAL_1MINUS1_OUT,                -- ObjectKind=Pin|PrimaryId=Q1-1-2
        X_3 => PowerSignal_GND                               -- ObjectKind=Pin|PrimaryId=Q1-1-3
      );

    J2 : Lysreklameparalellbusskontakt2015                   -- ObjectKind=Part|PrimaryId=J2|SecondaryId=1
      Port Map
      (
        X_1  => NamedSignal_KANAL_1MINUS1_OUT,               -- ObjectKind=Pin|PrimaryId=J2-1
        X_2  => NamedSignal_KANAL_1MINUS2_OUT,               -- ObjectKind=Pin|PrimaryId=J2-2
        X_3  => NamedSignal_KANAL_1MINUS3_OUT,               -- ObjectKind=Pin|PrimaryId=J2-3
        X_4  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-4
        X_5  => NamedSignal_KANAL_2MINUS1_OUT,               -- ObjectKind=Pin|PrimaryId=J2-5
        X_6  => NamedSignal_KANAL_2MINUS2_OUT,               -- ObjectKind=Pin|PrimaryId=J2-6
        X_7  => NamedSignal_KANAL_2MINUS3_OUT,               -- ObjectKind=Pin|PrimaryId=J2-7
        X_8  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-8
        X_9  => NamedSignal_KANAL_3MINUS1_OUT,               -- ObjectKind=Pin|PrimaryId=J2-9
        X_10 => NamedSignal_KANAL_3MINUS2_OUT,               -- ObjectKind=Pin|PrimaryId=J2-10
        X_11 => NamedSignal_KANAL_3MINUS3_OUT,               -- ObjectKind=Pin|PrimaryId=J2-11
        X_12 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-12
        X_13 => NamedSignal_KANAL_4MINUS1_OUT,               -- ObjectKind=Pin|PrimaryId=J2-13
        X_14 => NamedSignal_KANAL_4MINUS2_OUT,               -- ObjectKind=Pin|PrimaryId=J2-14
        X_15 => NamedSignal_KANAL_4MINUS3_OUT,               -- ObjectKind=Pin|PrimaryId=J2-15
        X_16 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-16
        X_17 => NamedSignal_KANAL_5MINUS1_OUT,               -- ObjectKind=Pin|PrimaryId=J2-17
        X_18 => NamedSignal_KANAL_5MINUS2_OUT,               -- ObjectKind=Pin|PrimaryId=J2-18
        X_19 => NamedSignal_KANAL_5MINUS3_OUT,               -- ObjectKind=Pin|PrimaryId=J2-19
        X_20 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-20
        X_21 => NamedSignal_KANAL_6MINUS1_OUT,               -- ObjectKind=Pin|PrimaryId=J2-21
        X_22 => NamedSignal_KANAL_6MINUS2_OUT,               -- ObjectKind=Pin|PrimaryId=J2-22
        X_23 => NamedSignal_KANAL_6MINUS3_OUT,               -- ObjectKind=Pin|PrimaryId=J2-23
        X_24 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-24
        X_25 => PowerSignal_VCC_P5V0,                        -- ObjectKind=Pin|PrimaryId=J2-25
        X_27 => PowerSignal_VCC_P5V0,                        -- ObjectKind=Pin|PrimaryId=J2-27
        X_28 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J2-28
        X_29 => NamedSignal_DMXPLUS,                         -- ObjectKind=Pin|PrimaryId=J2-29
        X_30 => NamedSignal_DMXMINUS,                        -- ObjectKind=Pin|PrimaryId=J2-30
        X_31 => NamedSignal_DMXPLUS_RETUR,                   -- ObjectKind=Pin|PrimaryId=J2-31
        X_32 => NamedSignal_DMXMINUS_RETUR                   -- ObjectKind=Pin|PrimaryId=J2-32
      );

    J1 : Lysreklameparalellbusskontakt2015                   -- ObjectKind=Part|PrimaryId=J1|SecondaryId=1
      Port Map
      (
        X_1  => NamedSignal_KANAL_1MINUS1,                   -- ObjectKind=Pin|PrimaryId=J1-1
        X_2  => NamedSignal_KANAL_1MINUS2,                   -- ObjectKind=Pin|PrimaryId=J1-2
        X_3  => NamedSignal_KANAL_1MINUS3,                   -- ObjectKind=Pin|PrimaryId=J1-3
        X_4  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-4
        X_5  => NamedSignal_KANAL_2MINUS1,                   -- ObjectKind=Pin|PrimaryId=J1-5
        X_6  => NamedSignal_KANAL_2MINUS2,                   -- ObjectKind=Pin|PrimaryId=J1-6
        X_7  => NamedSignal_KANAL_2MINUS3,                   -- ObjectKind=Pin|PrimaryId=J1-7
        X_8  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-8
        X_9  => NamedSignal_KANAL_3MINUS1,                   -- ObjectKind=Pin|PrimaryId=J1-9
        X_10 => NamedSignal_KANAL_3MINUS2,                   -- ObjectKind=Pin|PrimaryId=J1-10
        X_11 => NamedSignal_KANAL_3MINUS3,                   -- ObjectKind=Pin|PrimaryId=J1-11
        X_12 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-12
        X_13 => NamedSignal_KANAL_4MINUS1,                   -- ObjectKind=Pin|PrimaryId=J1-13
        X_14 => NamedSignal_KANAL_4MINUS2,                   -- ObjectKind=Pin|PrimaryId=J1-14
        X_15 => NamedSignal_KANAL_4MINUS3,                   -- ObjectKind=Pin|PrimaryId=J1-15
        X_16 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-16
        X_17 => NamedSignal_KANAL_5MINUS1,                   -- ObjectKind=Pin|PrimaryId=J1-17
        X_18 => NamedSignal_KANAL_5MINUS2,                   -- ObjectKind=Pin|PrimaryId=J1-18
        X_19 => NamedSignal_KANAL_5MINUS3,                   -- ObjectKind=Pin|PrimaryId=J1-19
        X_20 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-20
        X_21 => NamedSignal_KANAL_6MINUS1,                   -- ObjectKind=Pin|PrimaryId=J1-21
        X_22 => NamedSignal_KANAL_6MINUS2,                   -- ObjectKind=Pin|PrimaryId=J1-22
        X_23 => NamedSignal_KANAL_6MINUS3,                   -- ObjectKind=Pin|PrimaryId=J1-23
        X_24 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-24
        X_25 => PowerSignal_VCC_P5V0,                        -- ObjectKind=Pin|PrimaryId=J1-25
        X_26 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-26
        X_27 => PowerSignal_VCC_P5V0,                        -- ObjectKind=Pin|PrimaryId=J1-27
        X_28 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J1-28
        X_29 => NamedSignal_DMXPLUS,                         -- ObjectKind=Pin|PrimaryId=J1-29
        X_30 => NamedSignal_DMXMINUS,                        -- ObjectKind=Pin|PrimaryId=J1-30
        X_31 => NamedSignal_DMXPLUS_RETUR,                   -- ObjectKind=Pin|PrimaryId=J1-31
        X_32 => NamedSignal_DMXMINUS_RETUR                   -- ObjectKind=Pin|PrimaryId=J1-32
      );

    -- Signal Assignments
    ---------------------
    PowerSignal_GND <= '0'; -- ObjectKind=Net|PrimaryId=GND

End Structure;
------------------------------------------------------------

