------------------------------------------------------------
-- VHDL Bokskort_top
-- 2015 7 23 20 4 41
-- Created By "DXP VHDL Generator"
-- "Copyright (c) 2002-2014 Altium Limited"
-- Product Version: 15.0.7.36915
------------------------------------------------------------

------------------------------------------------------------
-- VHDL Bokskort_top
------------------------------------------------------------

Library IEEE;
Use     IEEE.std_logic_1164.all;

Entity Bokskort_top Is
  attribute MacroCell : boolean;

End Bokskort_top;
------------------------------------------------------------

------------------------------------------------------------
Architecture Structure Of Bokskort_top Is
   Component X_4P_LED_Connector                              -- ObjectKind=Part|PrimaryId=J1|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J1-1
        X_2 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J1-2
        X_3 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J1-3
        X_4 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=J1-4
      );
   End Component;

   Component Header_1x2                                      -- ObjectKind=Part|PrimaryId=J8|SecondaryId=1
      port
      (
        X_1 : inout STD_LOGIC;                               -- ObjectKind=Pin|PrimaryId=J8-1
        X_2 : inout STD_LOGIC                                -- ObjectKind=Pin|PrimaryId=J8-2
      );
   End Component;

   Component Lysreklameparalellbusskontakt2015               -- ObjectKind=Part|PrimaryId=J7|SecondaryId=1
      port
      (
        X_1  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-1
        X_2  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-2
        X_3  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-3
        X_4  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-4
        X_5  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-5
        X_6  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-6
        X_7  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-7
        X_8  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-8
        X_9  : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-9
        X_10 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-10
        X_11 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-11
        X_12 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-12
        X_13 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-13
        X_14 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-14
        X_15 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-15
        X_16 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-16
        X_17 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-17
        X_18 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-18
        X_19 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-19
        X_20 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-20
        X_21 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-21
        X_22 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-22
        X_23 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-23
        X_24 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-24
        X_25 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-25
        X_26 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-26
        X_27 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-27
        X_28 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-28
        X_29 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-29
        X_30 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-30
        X_31 : inout STD_LOGIC;                              -- ObjectKind=Pin|PrimaryId=J7-31
        X_32 : inout STD_LOGIC                               -- ObjectKind=Pin|PrimaryId=J7-32
      );
   End Component;


    Signal NamedSignal_DMXMINUS       : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX-
    Signal NamedSignal_DMXMINUS_RETUR : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX-_RETUR
    Signal NamedSignal_DMXPLUS        : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX+
    Signal NamedSignal_DMXPLUS_RETUR  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=DMX+_RETUR
    Signal NamedSignal_KANAL_1MINUS1  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_1-1
    Signal NamedSignal_KANAL_2MINUS1  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_2-1
    Signal NamedSignal_KANAL_3MINUS1  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_3-1
    Signal NamedSignal_KANAL_4MINUS1  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_4-1
    Signal NamedSignal_KANAL_5MINUS1  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_5-1
    Signal NamedSignal_KANAL_6MINUS1  : STD_LOGIC; -- ObjectKind=Net|PrimaryId=KANAL_6-1
    Signal PinSignal_J1_2             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ1_2
    Signal PinSignal_J1_3             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ1_3
    Signal PinSignal_J2_2             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ2_2
    Signal PinSignal_J2_3             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ2_3
    Signal PinSignal_J3_2             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ3_2
    Signal PinSignal_J3_3             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ3_3
    Signal PinSignal_J4_2             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ4_2
    Signal PinSignal_J4_3             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ4_3
    Signal PinSignal_J5_2             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ5_2
    Signal PinSignal_J5_3             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ5_3
    Signal PinSignal_J6_2             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ6_2
    Signal PinSignal_J6_3             : STD_LOGIC; -- ObjectKind=Net|PrimaryId=NetJ6_3
    Signal PowerSignal_GND            : STD_LOGIC; -- ObjectKind=Net|PrimaryId=GND
    Signal PowerSignal_VCC            : STD_LOGIC; -- ObjectKind=Net|PrimaryId=VCC

   attribute Database_Table_Name : string;
   attribute Database_Table_Name of J9 : Label is "altium";
   attribute Database_Table_Name of J8 : Label is "altium";

   attribute id : string;
   attribute id of J9 : Label is "2429";
   attribute id of J8 : Label is "2429";

   attribute navn : string;
   attribute navn of J9 : Label is "Header 1x2";
   attribute navn of J8 : Label is "Header 1x2";

   attribute pakke_opprettet : string;
   attribute pakke_opprettet of J9 : Label is "13.11.2014 12:30:48";
   attribute pakke_opprettet of J8 : Label is "13.11.2014 12:30:48";

   attribute pakke_opprettet_av : string;
   attribute pakke_opprettet_av of J9 : Label is "815";
   attribute pakke_opprettet_av of J8 : Label is "815";

   attribute pakketype : string;
   attribute pakketype of J9 : Label is "92";
   attribute pakketype of J8 : Label is "92";

   attribute pris : string;
   attribute pris of J9 : Label is "1";
   attribute pris of J8 : Label is "1";

   attribute symbol_opprettet : string;
   attribute symbol_opprettet of J9 : Label is "13.11.2014 10:58:53";
   attribute symbol_opprettet of J8 : Label is "13.11.2014 10:58:53";

   attribute symbol_opprettet_av : string;
   attribute symbol_opprettet_av of J9 : Label is "815";
   attribute symbol_opprettet_av of J8 : Label is "815";


Begin
    J9 : Header_1x2                                          -- ObjectKind=Part|PrimaryId=J9|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_DMXPLUS,                          -- ObjectKind=Pin|PrimaryId=J9-1
        X_2 => NamedSignal_DMXMINUS                          -- ObjectKind=Pin|PrimaryId=J9-2
      );

    J8 : Header_1x2                                          -- ObjectKind=Part|PrimaryId=J8|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_DMXPLUS_RETUR,                    -- ObjectKind=Pin|PrimaryId=J8-1
        X_2 => NamedSignal_DMXMINUS_RETUR                    -- ObjectKind=Pin|PrimaryId=J8-2
      );

    J7 : Lysreklameparalellbusskontakt2015                   -- ObjectKind=Part|PrimaryId=J7|SecondaryId=1
      Port Map
      (
        X_1  => NamedSignal_KANAL_1MINUS1,                   -- ObjectKind=Pin|PrimaryId=J7-1
        X_2  => PinSignal_J6_2,                              -- ObjectKind=Pin|PrimaryId=J7-2
        X_3  => PinSignal_J6_3,                              -- ObjectKind=Pin|PrimaryId=J7-3
        X_4  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-4
        X_5  => NamedSignal_KANAL_2MINUS1,                   -- ObjectKind=Pin|PrimaryId=J7-5
        X_6  => PinSignal_J5_2,                              -- ObjectKind=Pin|PrimaryId=J7-6
        X_7  => PinSignal_J5_3,                              -- ObjectKind=Pin|PrimaryId=J7-7
        X_8  => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-8
        X_9  => NamedSignal_KANAL_3MINUS1,                   -- ObjectKind=Pin|PrimaryId=J7-9
        X_10 => PinSignal_J4_2,                              -- ObjectKind=Pin|PrimaryId=J7-10
        X_11 => PinSignal_J4_3,                              -- ObjectKind=Pin|PrimaryId=J7-11
        X_12 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-12
        X_13 => NamedSignal_KANAL_4MINUS1,                   -- ObjectKind=Pin|PrimaryId=J7-13
        X_14 => PinSignal_J3_2,                              -- ObjectKind=Pin|PrimaryId=J7-14
        X_15 => PinSignal_J3_3,                              -- ObjectKind=Pin|PrimaryId=J7-15
        X_16 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-16
        X_17 => NamedSignal_KANAL_5MINUS1,                   -- ObjectKind=Pin|PrimaryId=J7-17
        X_18 => PinSignal_J2_2,                              -- ObjectKind=Pin|PrimaryId=J7-18
        X_19 => PinSignal_J2_3,                              -- ObjectKind=Pin|PrimaryId=J7-19
        X_20 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-20
        X_21 => NamedSignal_KANAL_6MINUS1,                   -- ObjectKind=Pin|PrimaryId=J7-21
        X_22 => PinSignal_J1_2,                              -- ObjectKind=Pin|PrimaryId=J7-22
        X_23 => PinSignal_J1_3,                              -- ObjectKind=Pin|PrimaryId=J7-23
        X_24 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-24
        X_25 => PowerSignal_VCC,                             -- ObjectKind=Pin|PrimaryId=J7-25
        X_26 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-26
        X_27 => PowerSignal_VCC,                             -- ObjectKind=Pin|PrimaryId=J7-27
        X_28 => PowerSignal_GND,                             -- ObjectKind=Pin|PrimaryId=J7-28
        X_29 => NamedSignal_DMXPLUS,                         -- ObjectKind=Pin|PrimaryId=J7-29
        X_30 => NamedSignal_DMXMINUS,                        -- ObjectKind=Pin|PrimaryId=J7-30
        X_31 => NamedSignal_DMXPLUS_RETUR,                   -- ObjectKind=Pin|PrimaryId=J7-31
        X_32 => NamedSignal_DMXMINUS_RETUR                   -- ObjectKind=Pin|PrimaryId=J7-32
      );

    J6 : X_4P_LED_Connector                                  -- ObjectKind=Part|PrimaryId=J6|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_1MINUS1,                    -- ObjectKind=Pin|PrimaryId=J6-1
        X_2 => PinSignal_J6_2,                               -- ObjectKind=Pin|PrimaryId=J6-2
        X_3 => PinSignal_J6_3,                               -- ObjectKind=Pin|PrimaryId=J6-3
        X_4 => PowerSignal_VCC                               -- ObjectKind=Pin|PrimaryId=J6-4
      );

    J5 : X_4P_LED_Connector                                  -- ObjectKind=Part|PrimaryId=J5|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_2MINUS1,                    -- ObjectKind=Pin|PrimaryId=J5-1
        X_2 => PinSignal_J5_2,                               -- ObjectKind=Pin|PrimaryId=J5-2
        X_3 => PinSignal_J5_3,                               -- ObjectKind=Pin|PrimaryId=J5-3
        X_4 => PowerSignal_VCC                               -- ObjectKind=Pin|PrimaryId=J5-4
      );

    J4 : X_4P_LED_Connector                                  -- ObjectKind=Part|PrimaryId=J4|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_3MINUS1,                    -- ObjectKind=Pin|PrimaryId=J4-1
        X_2 => PinSignal_J4_2,                               -- ObjectKind=Pin|PrimaryId=J4-2
        X_3 => PinSignal_J4_3,                               -- ObjectKind=Pin|PrimaryId=J4-3
        X_4 => PowerSignal_VCC                               -- ObjectKind=Pin|PrimaryId=J4-4
      );

    J3 : X_4P_LED_Connector                                  -- ObjectKind=Part|PrimaryId=J3|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_4MINUS1,                    -- ObjectKind=Pin|PrimaryId=J3-1
        X_2 => PinSignal_J3_2,                               -- ObjectKind=Pin|PrimaryId=J3-2
        X_3 => PinSignal_J3_3,                               -- ObjectKind=Pin|PrimaryId=J3-3
        X_4 => PowerSignal_VCC                               -- ObjectKind=Pin|PrimaryId=J3-4
      );

    J2 : X_4P_LED_Connector                                  -- ObjectKind=Part|PrimaryId=J2|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_5MINUS1,                    -- ObjectKind=Pin|PrimaryId=J2-1
        X_2 => PinSignal_J2_2,                               -- ObjectKind=Pin|PrimaryId=J2-2
        X_3 => PinSignal_J2_3,                               -- ObjectKind=Pin|PrimaryId=J2-3
        X_4 => PowerSignal_VCC                               -- ObjectKind=Pin|PrimaryId=J2-4
      );

    J1 : X_4P_LED_Connector                                  -- ObjectKind=Part|PrimaryId=J1|SecondaryId=1
      Port Map
      (
        X_1 => NamedSignal_KANAL_6MINUS1,                    -- ObjectKind=Pin|PrimaryId=J1-1
        X_2 => PinSignal_J1_2,                               -- ObjectKind=Pin|PrimaryId=J1-2
        X_3 => PinSignal_J1_3,                               -- ObjectKind=Pin|PrimaryId=J1-3
        X_4 => PowerSignal_VCC                               -- ObjectKind=Pin|PrimaryId=J1-4
      );

    -- Signal Assignments
    ---------------------
    PowerSignal_GND <= '0'; -- ObjectKind=Net|PrimaryId=GND
    PowerSignal_VCC <= '1'; -- ObjectKind=Net|PrimaryId=VCC

End Structure;
------------------------------------------------------------

