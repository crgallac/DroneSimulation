  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 6;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (quadRotorSim_P)
    ;%
      section.nData     = 11;
      section.data(11)  = dumData; %prealloc
      
	  ;% quadRotorSim_P.Obstacles
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_P.dt
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 11;
	
	  ;% quadRotorSim_P.navSphere
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 12;
	
	  ;% quadRotorSim_P.originOffset
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 13;
	
	  ;% quadRotorSim_P.radiusAvatar
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 16;
	
	  ;% quadRotorSim_P.DiscrDer_ICPrevScaledInput
	  section.data(6).logicalSrcIdx = 6;
	  section.data(6).dtTransOffset = 17;
	
	  ;% quadRotorSim_P.DiscreteDerivative_ICPrevScaledInput
	  section.data(7).logicalSrcIdx = 7;
	  section.data(7).dtTransOffset = 20;
	
	  ;% quadRotorSim_P.PacketOutput5_MaxMissedTicks
	  section.data(8).logicalSrcIdx = 9;
	  section.data(8).dtTransOffset = 21;
	
	  ;% quadRotorSim_P.PacketInput4_MaxMissedTicks
	  section.data(9).logicalSrcIdx = 10;
	  section.data(9).dtTransOffset = 22;
	
	  ;% quadRotorSim_P.PacketOutput5_YieldWhenWaiting
	  section.data(10).logicalSrcIdx = 11;
	  section.data(10).dtTransOffset = 23;
	
	  ;% quadRotorSim_P.PacketInput4_YieldWhenWaiting
	  section.data(11).logicalSrcIdx = 12;
	  section.data(11).dtTransOffset = 24;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% quadRotorSim_P.PacketOutput5_PacketID
	  section.data(1).logicalSrcIdx = 13;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_P.PacketInput4_PacketID
	  section.data(2).logicalSrcIdx = 14;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% quadRotorSim_P.WrapToZero_Threshold
	  section.data(1).logicalSrcIdx = 15;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(3) = section;
      clear section
      
      section.nData     = 22;
      section.data(22)  = dumData; %prealloc
      
	  ;% quadRotorSim_P.TSamp_WtEt
	  section.data(1).logicalSrcIdx = 16;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_P.Constant1_Value
	  section.data(2).logicalSrcIdx = 17;
	  section.data(2).dtTransOffset = 1;
	
	  ;% quadRotorSim_P.Constant3_Value
	  section.data(3).logicalSrcIdx = 18;
	  section.data(3).dtTransOffset = 5;
	
	  ;% quadRotorSim_P.TSamp_WtEt_c
	  section.data(4).logicalSrcIdx = 19;
	  section.data(4).dtTransOffset = 8;
	
	  ;% quadRotorSim_P.k_trans_Value
	  section.data(5).logicalSrcIdx = 20;
	  section.data(5).dtTransOffset = 9;
	
	  ;% quadRotorSim_P.b_trans_Value
	  section.data(6).logicalSrcIdx = 21;
	  section.data(6).dtTransOffset = 10;
	
	  ;% quadRotorSim_P.workspace_Value
	  section.data(7).logicalSrcIdx = 22;
	  section.data(7).dtTransOffset = 11;
	
	  ;% quadRotorSim_P.moveTimeDelay_Value
	  section.data(8).logicalSrcIdx = 23;
	  section.data(8).dtTransOffset = 15;
	
	  ;% quadRotorSim_P.moveConst_Value
	  section.data(9).logicalSrcIdx = 24;
	  section.data(9).dtTransOffset = 16;
	
	  ;% quadRotorSim_P.UnitDelay_InitialCondition
	  section.data(10).logicalSrcIdx = 25;
	  section.data(10).dtTransOffset = 17;
	
	  ;% quadRotorSim_P.Constant_Value
	  section.data(11).logicalSrcIdx = 26;
	  section.data(11).dtTransOffset = 21;
	
	  ;% quadRotorSim_P.Constant_Value_c
	  section.data(12).logicalSrcIdx = 27;
	  section.data(12).dtTransOffset = 22;
	
	  ;% quadRotorSim_P.Constant1_Value_d
	  section.data(13).logicalSrcIdx = 28;
	  section.data(13).dtTransOffset = 23;
	
	  ;% quadRotorSim_P.Gain_Gain
	  section.data(14).logicalSrcIdx = 29;
	  section.data(14).dtTransOffset = 24;
	
	  ;% quadRotorSim_P.Constant4_Value
	  section.data(15).logicalSrcIdx = 30;
	  section.data(15).dtTransOffset = 25;
	
	  ;% quadRotorSim_P.Constant3_Value_d
	  section.data(16).logicalSrcIdx = 31;
	  section.data(16).dtTransOffset = 28;
	
	  ;% quadRotorSim_P.Constant8_Value
	  section.data(17).logicalSrcIdx = 32;
	  section.data(17).dtTransOffset = 31;
	
	  ;% quadRotorSim_P.Constant1_Value_o
	  section.data(18).logicalSrcIdx = 33;
	  section.data(18).dtTransOffset = 34;
	
	  ;% quadRotorSim_P.Constant5_Value
	  section.data(19).logicalSrcIdx = 34;
	  section.data(19).dtTransOffset = 35;
	
	  ;% quadRotorSim_P.Gain1_Gain
	  section.data(20).logicalSrcIdx = 35;
	  section.data(20).dtTransOffset = 36;
	
	  ;% quadRotorSim_P.Constant6_Value
	  section.data(21).logicalSrcIdx = 36;
	  section.data(21).dtTransOffset = 37;
	
	  ;% quadRotorSim_P.Constant2_Value
	  section.data(22).logicalSrcIdx = 37;
	  section.data(22).dtTransOffset = 38;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(4) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% quadRotorSim_P.Constant_Value_g
	  section.data(1).logicalSrcIdx = 38;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_P.Output_InitialCondition
	  section.data(2).logicalSrcIdx = 39;
	  section.data(2).dtTransOffset = 1;
	
	  ;% quadRotorSim_P.FixPtConstant_Value
	  section.data(3).logicalSrcIdx = 40;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(5) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% quadRotorSim_P.ManualSwitch1_CurrentSetting
	  section.data(1).logicalSrcIdx = 41;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_P.ManualSwitch_CurrentSetting
	  section.data(2).logicalSrcIdx = 42;
	  section.data(2).dtTransOffset = 1;
	
	  ;% quadRotorSim_P.ManualSwitch_CurrentSetting_d
	  section.data(3).logicalSrcIdx = 43;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(6) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 3;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (quadRotorSim_B)
    ;%
      section.nData     = 23;
      section.data(23)  = dumData; %prealloc
      
	  ;% quadRotorSim_B.Conversion
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_B.TSamp
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 3;
	
	  ;% quadRotorSim_B.TSamp_a
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 6;
	
	  ;% quadRotorSim_B.ManualSwitch
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 9;
	
	  ;% quadRotorSim_B.Sum3
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 12;
	
	  ;% quadRotorSim_B.Constant9
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 15;
	
	  ;% quadRotorSim_B.RateTransition
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 26;
	
	  ;% quadRotorSim_B.Constant4
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 29;
	
	  ;% quadRotorSim_B.Constant1
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 30;
	
	  ;% quadRotorSim_B.Constant5
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 31;
	
	  ;% quadRotorSim_B.Gain1
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 32;
	
	  ;% quadRotorSim_B.Constant6
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 33;
	
	  ;% quadRotorSim_B.TmpSignalConversionAtVRSinkInport5
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 34;
	
	  ;% quadRotorSim_B.Constant2
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 37;
	
	  ;% quadRotorSim_B.RateTransition1
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 38;
	
	  ;% quadRotorSim_B.Constant7
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 41;
	
	  ;% quadRotorSim_B.F
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 42;
	
	  ;% quadRotorSim_B.deviceForces
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 45;
	
	  ;% quadRotorSim_B.tableMoveOut
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 48;
	
	  ;% quadRotorSim_B.time1
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 51;
	
	  ;% quadRotorSim_B.xyzP
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 52;
	
	  ;% quadRotorSim_B.y
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 55;
	
	  ;% quadRotorSim_B.xyzPout
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 58;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% quadRotorSim_B.PacketInput4_o1
	  section.data(1).logicalSrcIdx = 23;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_B.Conversion1
	  section.data(2).logicalSrcIdx = 24;
	  section.data(2).dtTransOffset = 3;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(2) = section;
      clear section
      
      section.nData     = 4;
      section.data(4)  = dumData; %prealloc
      
	  ;% quadRotorSim_B.PacketInput4_o2
	  section.data(1).logicalSrcIdx = 25;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_B.Output
	  section.data(2).logicalSrcIdx = 26;
	  section.data(2).dtTransOffset = 1;
	
	  ;% quadRotorSim_B.Sum1
	  section.data(3).logicalSrcIdx = 27;
	  section.data(3).dtTransOffset = 2;
	
	  ;% quadRotorSim_B.FixPtSwitch
	  section.data(4).logicalSrcIdx = 28;
	  section.data(4).dtTransOffset = 3;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(3) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 3;
    sectIdxOffset = 3;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (quadRotorSim_DW)
    ;%
      section.nData     = 4;
      section.data(4)  = dumData; %prealloc
      
	  ;% quadRotorSim_DW.UD_DSTATE
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_DW.UD_DSTATE_j
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 3;
	
	  ;% quadRotorSim_DW.UnitDelay_DSTATE
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 6;
	
	  ;% quadRotorSim_DW.clock
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 10;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 4;
      section.data(4)  = dumData; %prealloc
      
	  ;% quadRotorSim_DW.PacketOutput5_PWORK
	  section.data(1).logicalSrcIdx = 4;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadRotorSim_DW.PacketInput4_PWORK
	  section.data(2).logicalSrcIdx = 5;
	  section.data(2).dtTransOffset = 2;
	
	  ;% quadRotorSim_DW.Scope_PWORK.LoggedData
	  section.data(3).logicalSrcIdx = 6;
	  section.data(3).dtTransOffset = 3;
	
	  ;% quadRotorSim_DW.VRSink_PWORK
	  section.data(4).logicalSrcIdx = 7;
	  section.data(4).dtTransOffset = 4;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% quadRotorSim_DW.Output_DSTATE
	  section.data(1).logicalSrcIdx = 8;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 3811013152;
  targMap.checksum1 = 1689044043;
  targMap.checksum2 = 1072623783;
  targMap.checksum3 = 2143496835;

