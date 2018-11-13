#include <css_loam_velodyne/gaussians.h>

void initKernels(std::vector<float> gaussian1[KERNEL_NUM], std::vector<float> gaussian2[KERNEL_NUM])
{
	gaussian1[0] = std::vector<float>(g1_0, g1_0 + sizeof(g1_0) / sizeof(float) );
	gaussian2[0] = std::vector<float>(g2_0, g2_0 + sizeof(g2_0) / sizeof(float) );

	gaussian1[1] = std::vector<float>(g1_1, g1_1 + sizeof(g1_1) / sizeof(float) );
	gaussian2[1] = std::vector<float>(g2_1, g2_1 + sizeof(g2_1) / sizeof(float) );

	gaussian1[2] = std::vector<float>(g1_2, g1_2 + sizeof(g1_2) / sizeof(float) );
	gaussian2[2] = std::vector<float>(g2_2, g2_2 + sizeof(g2_2) / sizeof(float) );

	gaussian1[3] = std::vector<float>(g1_3, g1_3 + sizeof(g1_3) / sizeof(float) );
	gaussian2[3] = std::vector<float>(g2_3, g2_3 + sizeof(g2_3) / sizeof(float) );

	gaussian1[4] = std::vector<float>(g1_4, g1_4 + sizeof(g1_4) / sizeof(float) );
	gaussian2[4] = std::vector<float>(g2_4, g2_4 + sizeof(g2_4) / sizeof(float) );

	gaussian1[5] = std::vector<float>(g1_5, g1_5 + sizeof(g1_5) / sizeof(float) );
	gaussian2[5] = std::vector<float>(g2_5, g2_5 + sizeof(g2_5) / sizeof(float) );

	gaussian1[6] = std::vector<float>(g1_6, g1_6 + sizeof(g1_6) / sizeof(float) );
	gaussian2[6] = std::vector<float>(g2_6, g2_6 + sizeof(g2_6) / sizeof(float) );

	gaussian1[7] = std::vector<float>(g1_7, g1_7 + sizeof(g1_7) / sizeof(float) );
	gaussian2[7] = std::vector<float>(g2_7, g2_7 + sizeof(g2_7) / sizeof(float) );

	gaussian1[8] = std::vector<float>(g1_8, g1_8 + sizeof(g1_8) / sizeof(float) );
	gaussian2[8] = std::vector<float>(g2_8, g2_8 + sizeof(g2_8) / sizeof(float) );

	gaussian1[9] = std::vector<float>(g1_9, g1_9 + sizeof(g1_9) / sizeof(float) );
	gaussian2[9] = std::vector<float>(g2_9, g2_9 + sizeof(g2_9) / sizeof(float) );

	gaussian1[10] = std::vector<float>(g1_10, g1_10 + sizeof(g1_10) / sizeof(float) );
	gaussian2[10] = std::vector<float>(g2_10, g2_10 + sizeof(g2_10) / sizeof(float) );

	gaussian1[11] = std::vector<float>(g1_11, g1_11 + sizeof(g1_11) / sizeof(float) );
	gaussian2[11] = std::vector<float>(g2_11, g2_11 + sizeof(g2_11) / sizeof(float) );

	gaussian1[12] = std::vector<float>(g1_12, g1_12 + sizeof(g1_12) / sizeof(float) );
	gaussian2[12] = std::vector<float>(g2_12, g2_12 + sizeof(g2_12) / sizeof(float) );

	gaussian1[13] = std::vector<float>(g1_13, g1_13 + sizeof(g1_13) / sizeof(float) );
	gaussian2[13] = std::vector<float>(g2_13, g2_13 + sizeof(g2_13) / sizeof(float) );

	gaussian1[14] = std::vector<float>(g1_14, g1_14 + sizeof(g1_14) / sizeof(float) );
	gaussian2[14] = std::vector<float>(g2_14, g2_14 + sizeof(g2_14) / sizeof(float) );

	gaussian1[15] = std::vector<float>(g1_15, g1_15 + sizeof(g1_15) / sizeof(float) );
	gaussian2[15] = std::vector<float>(g2_15, g2_15 + sizeof(g2_15) / sizeof(float) );

	gaussian1[16] = std::vector<float>(g1_16, g1_16 + sizeof(g1_16) / sizeof(float) );
	gaussian2[16] = std::vector<float>(g2_16, g2_16 + sizeof(g2_16) / sizeof(float) );

	gaussian1[17] = std::vector<float>(g1_17, g1_17 + sizeof(g1_17) / sizeof(float) );
	gaussian2[17] = std::vector<float>(g2_17, g2_17 + sizeof(g2_17) / sizeof(float) );

	gaussian1[18] = std::vector<float>(g1_18, g1_18 + sizeof(g1_18) / sizeof(float) );
	gaussian2[18] = std::vector<float>(g2_18, g2_18 + sizeof(g2_18) / sizeof(float) );

	gaussian1[19] = std::vector<float>(g1_19, g1_19 + sizeof(g1_19) / sizeof(float) );
	gaussian2[19] = std::vector<float>(g2_19, g2_19 + sizeof(g2_19) / sizeof(float) );

	gaussian1[20] = std::vector<float>(g1_20, g1_20 + sizeof(g1_20) / sizeof(float) );
	gaussian2[20] = std::vector<float>(g2_20, g2_20 + sizeof(g2_20) / sizeof(float) );

	gaussian1[21] = std::vector<float>(g1_21, g1_21 + sizeof(g1_21) / sizeof(float) );
	gaussian2[21] = std::vector<float>(g2_21, g2_21 + sizeof(g2_21) / sizeof(float) );

	gaussian1[22] = std::vector<float>(g1_22, g1_22 + sizeof(g1_22) / sizeof(float) );
	gaussian2[22] = std::vector<float>(g2_22, g2_22 + sizeof(g2_22) / sizeof(float) );

	gaussian1[23] = std::vector<float>(g1_23, g1_23 + sizeof(g1_23) / sizeof(float) );
	gaussian2[23] = std::vector<float>(g2_23, g2_23 + sizeof(g2_23) / sizeof(float) );

	gaussian1[24] = std::vector<float>(g1_24, g1_24 + sizeof(g1_24) / sizeof(float) );
	gaussian2[24] = std::vector<float>(g2_24, g2_24 + sizeof(g2_24) / sizeof(float) );

	gaussian1[25] = std::vector<float>(g1_25, g1_25 + sizeof(g1_25) / sizeof(float) );
	gaussian2[25] = std::vector<float>(g2_25, g2_25 + sizeof(g2_25) / sizeof(float) );

	gaussian1[26] = std::vector<float>(g1_26, g1_26 + sizeof(g1_26) / sizeof(float) );
	gaussian2[26] = std::vector<float>(g2_26, g2_26 + sizeof(g2_26) / sizeof(float) );

	gaussian1[27] = std::vector<float>(g1_27, g1_27 + sizeof(g1_27) / sizeof(float) );
	gaussian2[27] = std::vector<float>(g2_27, g2_27 + sizeof(g2_27) / sizeof(float) );

	gaussian1[28] = std::vector<float>(g1_28, g1_28 + sizeof(g1_28) / sizeof(float) );
	gaussian2[28] = std::vector<float>(g2_28, g2_28 + sizeof(g2_28) / sizeof(float) );

	gaussian1[29] = std::vector<float>(g1_29, g1_29 + sizeof(g1_29) / sizeof(float) );
	gaussian2[29] = std::vector<float>(g2_29, g2_29 + sizeof(g2_29) / sizeof(float) );

	gaussian1[30] = std::vector<float>(g1_30, g1_30 + sizeof(g1_30) / sizeof(float) );
	gaussian2[30] = std::vector<float>(g2_30, g2_30 + sizeof(g2_30) / sizeof(float) );

	gaussian1[31] = std::vector<float>(g1_31, g1_31 + sizeof(g1_31) / sizeof(float) );
	gaussian2[31] = std::vector<float>(g2_31, g2_31 + sizeof(g2_31) / sizeof(float) );

	gaussian1[32] = std::vector<float>(g1_32, g1_32 + sizeof(g1_32) / sizeof(float) );
	gaussian2[32] = std::vector<float>(g2_32, g2_32 + sizeof(g2_32) / sizeof(float) );

	gaussian1[33] = std::vector<float>(g1_33, g1_33 + sizeof(g1_33) / sizeof(float) );
	gaussian2[33] = std::vector<float>(g2_33, g2_33 + sizeof(g2_33) / sizeof(float) );

	gaussian1[34] = std::vector<float>(g1_34, g1_34 + sizeof(g1_34) / sizeof(float) );
	gaussian2[34] = std::vector<float>(g2_34, g2_34 + sizeof(g2_34) / sizeof(float) );

	gaussian1[35] = std::vector<float>(g1_35, g1_35 + sizeof(g1_35) / sizeof(float) );
	gaussian2[35] = std::vector<float>(g2_35, g2_35 + sizeof(g2_35) / sizeof(float) );

	gaussian1[36] = std::vector<float>(g1_36, g1_36 + sizeof(g1_36) / sizeof(float) );
	gaussian2[36] = std::vector<float>(g2_36, g2_36 + sizeof(g2_36) / sizeof(float) );

	gaussian1[37] = std::vector<float>(g1_37, g1_37 + sizeof(g1_37) / sizeof(float) );
	gaussian2[37] = std::vector<float>(g2_37, g2_37 + sizeof(g2_37) / sizeof(float) );

	gaussian1[38] = std::vector<float>(g1_38, g1_38 + sizeof(g1_38) / sizeof(float) );
	gaussian2[38] = std::vector<float>(g2_38, g2_38 + sizeof(g2_38) / sizeof(float) );

	gaussian1[39] = std::vector<float>(g1_39, g1_39 + sizeof(g1_39) / sizeof(float) );
	gaussian2[39] = std::vector<float>(g2_39, g2_39 + sizeof(g2_39) / sizeof(float) );

	gaussian1[40] = std::vector<float>(g1_40, g1_40 + sizeof(g1_40) / sizeof(float) );
	gaussian2[40] = std::vector<float>(g2_40, g2_40 + sizeof(g2_40) / sizeof(float) );

	gaussian1[41] = std::vector<float>(g1_41, g1_41 + sizeof(g1_41) / sizeof(float) );
	gaussian2[41] = std::vector<float>(g2_41, g2_41 + sizeof(g2_41) / sizeof(float) );

	gaussian1[42] = std::vector<float>(g1_42, g1_42 + sizeof(g1_42) / sizeof(float) );
	gaussian2[42] = std::vector<float>(g2_42, g2_42 + sizeof(g2_42) / sizeof(float) );

	gaussian1[43] = std::vector<float>(g1_43, g1_43 + sizeof(g1_43) / sizeof(float) );
	gaussian2[43] = std::vector<float>(g2_43, g2_43 + sizeof(g2_43) / sizeof(float) );

	gaussian1[44] = std::vector<float>(g1_44, g1_44 + sizeof(g1_44) / sizeof(float) );
	gaussian2[44] = std::vector<float>(g2_44, g2_44 + sizeof(g2_44) / sizeof(float) );

	gaussian1[45] = std::vector<float>(g1_45, g1_45 + sizeof(g1_45) / sizeof(float) );
	gaussian2[45] = std::vector<float>(g2_45, g2_45 + sizeof(g2_45) / sizeof(float) );

	gaussian1[46] = std::vector<float>(g1_46, g1_46 + sizeof(g1_46) / sizeof(float) );
	gaussian2[46] = std::vector<float>(g2_46, g2_46 + sizeof(g2_46) / sizeof(float) );

	gaussian1[47] = std::vector<float>(g1_47, g1_47 + sizeof(g1_47) / sizeof(float) );
	gaussian2[47] = std::vector<float>(g2_47, g2_47 + sizeof(g2_47) / sizeof(float) );

	gaussian1[48] = std::vector<float>(g1_48, g1_48 + sizeof(g1_48) / sizeof(float) );
	gaussian2[48] = std::vector<float>(g2_48, g2_48 + sizeof(g2_48) / sizeof(float) );

	gaussian1[49] = std::vector<float>(g1_49, g1_49 + sizeof(g1_49) / sizeof(float) );
	gaussian2[49] = std::vector<float>(g2_49, g2_49 + sizeof(g2_49) / sizeof(float) );

	gaussian1[50] = std::vector<float>(g1_50, g1_50 + sizeof(g1_50) / sizeof(float) );
	gaussian2[50] = std::vector<float>(g2_50, g2_50 + sizeof(g2_50) / sizeof(float) );

	gaussian1[51] = std::vector<float>(g1_51, g1_51 + sizeof(g1_51) / sizeof(float) );
	gaussian2[51] = std::vector<float>(g2_51, g2_51 + sizeof(g2_51) / sizeof(float) );

	gaussian1[52] = std::vector<float>(g1_52, g1_52 + sizeof(g1_52) / sizeof(float) );
	gaussian2[52] = std::vector<float>(g2_52, g2_52 + sizeof(g2_52) / sizeof(float) );

	gaussian1[53] = std::vector<float>(g1_53, g1_53 + sizeof(g1_53) / sizeof(float) );
	gaussian2[53] = std::vector<float>(g2_53, g2_53 + sizeof(g2_53) / sizeof(float) );

	gaussian1[54] = std::vector<float>(g1_54, g1_54 + sizeof(g1_54) / sizeof(float) );
	gaussian2[54] = std::vector<float>(g2_54, g2_54 + sizeof(g2_54) / sizeof(float) );

	gaussian1[55] = std::vector<float>(g1_55, g1_55 + sizeof(g1_55) / sizeof(float) );
	gaussian2[55] = std::vector<float>(g2_55, g2_55 + sizeof(g2_55) / sizeof(float) );

	gaussian1[56] = std::vector<float>(g1_56, g1_56 + sizeof(g1_56) / sizeof(float) );
	gaussian2[56] = std::vector<float>(g2_56, g2_56 + sizeof(g2_56) / sizeof(float) );

	gaussian1[57] = std::vector<float>(g1_57, g1_57 + sizeof(g1_57) / sizeof(float) );
	gaussian2[57] = std::vector<float>(g2_57, g2_57 + sizeof(g2_57) / sizeof(float) );

	gaussian1[58] = std::vector<float>(g1_58, g1_58 + sizeof(g1_58) / sizeof(float) );
	gaussian2[58] = std::vector<float>(g2_58, g2_58 + sizeof(g2_58) / sizeof(float) );

	gaussian1[59] = std::vector<float>(g1_59, g1_59 + sizeof(g1_59) / sizeof(float) );
	gaussian2[59] = std::vector<float>(g2_59, g2_59 + sizeof(g2_59) / sizeof(float) );

	gaussian1[60] = std::vector<float>(g1_60, g1_60 + sizeof(g1_60) / sizeof(float) );
	gaussian2[60] = std::vector<float>(g2_60, g2_60 + sizeof(g2_60) / sizeof(float) );

	gaussian1[61] = std::vector<float>(g1_61, g1_61 + sizeof(g1_61) / sizeof(float) );
	gaussian2[61] = std::vector<float>(g2_61, g2_61 + sizeof(g2_61) / sizeof(float) );

	gaussian1[62] = std::vector<float>(g1_62, g1_62 + sizeof(g1_62) / sizeof(float) );
	gaussian2[62] = std::vector<float>(g2_62, g2_62 + sizeof(g2_62) / sizeof(float) );

	gaussian1[63] = std::vector<float>(g1_63, g1_63 + sizeof(g1_63) / sizeof(float) );
	gaussian2[63] = std::vector<float>(g2_63, g2_63 + sizeof(g2_63) / sizeof(float) );

	gaussian1[64] = std::vector<float>(g1_64, g1_64 + sizeof(g1_64) / sizeof(float) );
	gaussian2[64] = std::vector<float>(g2_64, g2_64 + sizeof(g2_64) / sizeof(float) );

	gaussian1[65] = std::vector<float>(g1_65, g1_65 + sizeof(g1_65) / sizeof(float) );
	gaussian2[65] = std::vector<float>(g2_65, g2_65 + sizeof(g2_65) / sizeof(float) );

	gaussian1[66] = std::vector<float>(g1_66, g1_66 + sizeof(g1_66) / sizeof(float) );
	gaussian2[66] = std::vector<float>(g2_66, g2_66 + sizeof(g2_66) / sizeof(float) );

	gaussian1[67] = std::vector<float>(g1_67, g1_67 + sizeof(g1_67) / sizeof(float) );
	gaussian2[67] = std::vector<float>(g2_67, g2_67 + sizeof(g2_67) / sizeof(float) );

	gaussian1[68] = std::vector<float>(g1_68, g1_68 + sizeof(g1_68) / sizeof(float) );
	gaussian2[68] = std::vector<float>(g2_68, g2_68 + sizeof(g2_68) / sizeof(float) );

	gaussian1[69] = std::vector<float>(g1_69, g1_69 + sizeof(g1_69) / sizeof(float) );
	gaussian2[69] = std::vector<float>(g2_69, g2_69 + sizeof(g2_69) / sizeof(float) );

	gaussian1[70] = std::vector<float>(g1_70, g1_70 + sizeof(g1_70) / sizeof(float) );
	gaussian2[70] = std::vector<float>(g2_70, g2_70 + sizeof(g2_70) / sizeof(float) );

	gaussian1[71] = std::vector<float>(g1_71, g1_71 + sizeof(g1_71) / sizeof(float) );
	gaussian2[71] = std::vector<float>(g2_71, g2_71 + sizeof(g2_71) / sizeof(float) );

	gaussian1[72] = std::vector<float>(g1_72, g1_72 + sizeof(g1_72) / sizeof(float) );
	gaussian2[72] = std::vector<float>(g2_72, g2_72 + sizeof(g2_72) / sizeof(float) );

	gaussian1[73] = std::vector<float>(g1_73, g1_73 + sizeof(g1_73) / sizeof(float) );
	gaussian2[73] = std::vector<float>(g2_73, g2_73 + sizeof(g2_73) / sizeof(float) );

	gaussian1[74] = std::vector<float>(g1_74, g1_74 + sizeof(g1_74) / sizeof(float) );
	gaussian2[74] = std::vector<float>(g2_74, g2_74 + sizeof(g2_74) / sizeof(float) );

	gaussian1[75] = std::vector<float>(g1_75, g1_75 + sizeof(g1_75) / sizeof(float) );
	gaussian2[75] = std::vector<float>(g2_75, g2_75 + sizeof(g2_75) / sizeof(float) );

	gaussian1[76] = std::vector<float>(g1_76, g1_76 + sizeof(g1_76) / sizeof(float) );
	gaussian2[76] = std::vector<float>(g2_76, g2_76 + sizeof(g2_76) / sizeof(float) );

	gaussian1[77] = std::vector<float>(g1_77, g1_77 + sizeof(g1_77) / sizeof(float) );
	gaussian2[77] = std::vector<float>(g2_77, g2_77 + sizeof(g2_77) / sizeof(float) );

	gaussian1[78] = std::vector<float>(g1_78, g1_78 + sizeof(g1_78) / sizeof(float) );
	gaussian2[78] = std::vector<float>(g2_78, g2_78 + sizeof(g2_78) / sizeof(float) );

	gaussian1[79] = std::vector<float>(g1_79, g1_79 + sizeof(g1_79) / sizeof(float) );
	gaussian2[79] = std::vector<float>(g2_79, g2_79 + sizeof(g2_79) / sizeof(float) );

	gaussian1[80] = std::vector<float>(g1_80, g1_80 + sizeof(g1_80) / sizeof(float) );
	gaussian2[80] = std::vector<float>(g2_80, g2_80 + sizeof(g2_80) / sizeof(float) );

	gaussian1[81] = std::vector<float>(g1_81, g1_81 + sizeof(g1_81) / sizeof(float) );
	gaussian2[81] = std::vector<float>(g2_81, g2_81 + sizeof(g2_81) / sizeof(float) );

	gaussian1[82] = std::vector<float>(g1_82, g1_82 + sizeof(g1_82) / sizeof(float) );
	gaussian2[82] = std::vector<float>(g2_82, g2_82 + sizeof(g2_82) / sizeof(float) );

	gaussian1[83] = std::vector<float>(g1_83, g1_83 + sizeof(g1_83) / sizeof(float) );
	gaussian2[83] = std::vector<float>(g2_83, g2_83 + sizeof(g2_83) / sizeof(float) );

	gaussian1[84] = std::vector<float>(g1_84, g1_84 + sizeof(g1_84) / sizeof(float) );
	gaussian2[84] = std::vector<float>(g2_84, g2_84 + sizeof(g2_84) / sizeof(float) );

	gaussian1[85] = std::vector<float>(g1_85, g1_85 + sizeof(g1_85) / sizeof(float) );
	gaussian2[85] = std::vector<float>(g2_85, g2_85 + sizeof(g2_85) / sizeof(float) );

	gaussian1[86] = std::vector<float>(g1_86, g1_86 + sizeof(g1_86) / sizeof(float) );
	gaussian2[86] = std::vector<float>(g2_86, g2_86 + sizeof(g2_86) / sizeof(float) );

	gaussian1[87] = std::vector<float>(g1_87, g1_87 + sizeof(g1_87) / sizeof(float) );
	gaussian2[87] = std::vector<float>(g2_87, g2_87 + sizeof(g2_87) / sizeof(float) );

	gaussian1[88] = std::vector<float>(g1_88, g1_88 + sizeof(g1_88) / sizeof(float) );
	gaussian2[88] = std::vector<float>(g2_88, g2_88 + sizeof(g2_88) / sizeof(float) );

	gaussian1[89] = std::vector<float>(g1_89, g1_89 + sizeof(g1_89) / sizeof(float) );
	gaussian2[89] = std::vector<float>(g2_89, g2_89 + sizeof(g2_89) / sizeof(float) );

	gaussian1[90] = std::vector<float>(g1_90, g1_90 + sizeof(g1_90) / sizeof(float) );
	gaussian2[90] = std::vector<float>(g2_90, g2_90 + sizeof(g2_90) / sizeof(float) );

	gaussian1[91] = std::vector<float>(g1_91, g1_91 + sizeof(g1_91) / sizeof(float) );
	gaussian2[91] = std::vector<float>(g2_91, g2_91 + sizeof(g2_91) / sizeof(float) );

	gaussian1[92] = std::vector<float>(g1_92, g1_92 + sizeof(g1_92) / sizeof(float) );
	gaussian2[92] = std::vector<float>(g2_92, g2_92 + sizeof(g2_92) / sizeof(float) );

	gaussian1[93] = std::vector<float>(g1_93, g1_93 + sizeof(g1_93) / sizeof(float) );
	gaussian2[93] = std::vector<float>(g2_93, g2_93 + sizeof(g2_93) / sizeof(float) );

	gaussian1[94] = std::vector<float>(g1_94, g1_94 + sizeof(g1_94) / sizeof(float) );
	gaussian2[94] = std::vector<float>(g2_94, g2_94 + sizeof(g2_94) / sizeof(float) );

	gaussian1[95] = std::vector<float>(g1_95, g1_95 + sizeof(g1_95) / sizeof(float) );
	gaussian2[95] = std::vector<float>(g2_95, g2_95 + sizeof(g2_95) / sizeof(float) );

	gaussian1[96] = std::vector<float>(g1_96, g1_96 + sizeof(g1_96) / sizeof(float) );
	gaussian2[96] = std::vector<float>(g2_96, g2_96 + sizeof(g2_96) / sizeof(float) );

	gaussian1[97] = std::vector<float>(g1_97, g1_97 + sizeof(g1_97) / sizeof(float) );
	gaussian2[97] = std::vector<float>(g2_97, g2_97 + sizeof(g2_97) / sizeof(float) );

	gaussian1[98] = std::vector<float>(g1_98, g1_98 + sizeof(g1_98) / sizeof(float) );
	gaussian2[98] = std::vector<float>(g2_98, g2_98 + sizeof(g2_98) / sizeof(float) );

	gaussian1[99] = std::vector<float>(g1_99, g1_99 + sizeof(g1_99) / sizeof(float) );
	gaussian2[99] = std::vector<float>(g2_99, g2_99 + sizeof(g2_99) / sizeof(float) );

	gaussian1[100] = std::vector<float>(g1_100, g1_100 + sizeof(g1_100) / sizeof(float) );
	gaussian2[100] = std::vector<float>(g2_100, g2_100 + sizeof(g2_100) / sizeof(float) );

	gaussian1[101] = std::vector<float>(g1_101, g1_101 + sizeof(g1_101) / sizeof(float) );
	gaussian2[101] = std::vector<float>(g2_101, g2_101 + sizeof(g2_101) / sizeof(float) );

	gaussian1[102] = std::vector<float>(g1_102, g1_102 + sizeof(g1_102) / sizeof(float) );
	gaussian2[102] = std::vector<float>(g2_102, g2_102 + sizeof(g2_102) / sizeof(float) );

	gaussian1[103] = std::vector<float>(g1_103, g1_103 + sizeof(g1_103) / sizeof(float) );
	gaussian2[103] = std::vector<float>(g2_103, g2_103 + sizeof(g2_103) / sizeof(float) );

	gaussian1[104] = std::vector<float>(g1_104, g1_104 + sizeof(g1_104) / sizeof(float) );
	gaussian2[104] = std::vector<float>(g2_104, g2_104 + sizeof(g2_104) / sizeof(float) );

	gaussian1[105] = std::vector<float>(g1_105, g1_105 + sizeof(g1_105) / sizeof(float) );
	gaussian2[105] = std::vector<float>(g2_105, g2_105 + sizeof(g2_105) / sizeof(float) );

	gaussian1[106] = std::vector<float>(g1_106, g1_106 + sizeof(g1_106) / sizeof(float) );
	gaussian2[106] = std::vector<float>(g2_106, g2_106 + sizeof(g2_106) / sizeof(float) );

	gaussian1[107] = std::vector<float>(g1_107, g1_107 + sizeof(g1_107) / sizeof(float) );
	gaussian2[107] = std::vector<float>(g2_107, g2_107 + sizeof(g2_107) / sizeof(float) );

	gaussian1[108] = std::vector<float>(g1_108, g1_108 + sizeof(g1_108) / sizeof(float) );
	gaussian2[108] = std::vector<float>(g2_108, g2_108 + sizeof(g2_108) / sizeof(float) );

	gaussian1[109] = std::vector<float>(g1_109, g1_109 + sizeof(g1_109) / sizeof(float) );
	gaussian2[109] = std::vector<float>(g2_109, g2_109 + sizeof(g2_109) / sizeof(float) );

	gaussian1[110] = std::vector<float>(g1_110, g1_110 + sizeof(g1_110) / sizeof(float) );
	gaussian2[110] = std::vector<float>(g2_110, g2_110 + sizeof(g2_110) / sizeof(float) );

	gaussian1[111] = std::vector<float>(g1_111, g1_111 + sizeof(g1_111) / sizeof(float) );
	gaussian2[111] = std::vector<float>(g2_111, g2_111 + sizeof(g2_111) / sizeof(float) );

	gaussian1[112] = std::vector<float>(g1_112, g1_112 + sizeof(g1_112) / sizeof(float) );
	gaussian2[112] = std::vector<float>(g2_112, g2_112 + sizeof(g2_112) / sizeof(float) );

	gaussian1[113] = std::vector<float>(g1_113, g1_113 + sizeof(g1_113) / sizeof(float) );
	gaussian2[113] = std::vector<float>(g2_113, g2_113 + sizeof(g2_113) / sizeof(float) );

	gaussian1[114] = std::vector<float>(g1_114, g1_114 + sizeof(g1_114) / sizeof(float) );
	gaussian2[114] = std::vector<float>(g2_114, g2_114 + sizeof(g2_114) / sizeof(float) );

	gaussian1[115] = std::vector<float>(g1_115, g1_115 + sizeof(g1_115) / sizeof(float) );
	gaussian2[115] = std::vector<float>(g2_115, g2_115 + sizeof(g2_115) / sizeof(float) );

	gaussian1[116] = std::vector<float>(g1_116, g1_116 + sizeof(g1_116) / sizeof(float) );
	gaussian2[116] = std::vector<float>(g2_116, g2_116 + sizeof(g2_116) / sizeof(float) );

	gaussian1[117] = std::vector<float>(g1_117, g1_117 + sizeof(g1_117) / sizeof(float) );
	gaussian2[117] = std::vector<float>(g2_117, g2_117 + sizeof(g2_117) / sizeof(float) );

	gaussian1[118] = std::vector<float>(g1_118, g1_118 + sizeof(g1_118) / sizeof(float) );
	gaussian2[118] = std::vector<float>(g2_118, g2_118 + sizeof(g2_118) / sizeof(float) );

	gaussian1[119] = std::vector<float>(g1_119, g1_119 + sizeof(g1_119) / sizeof(float) );
	gaussian2[119] = std::vector<float>(g2_119, g2_119 + sizeof(g2_119) / sizeof(float) );

	gaussian1[120] = std::vector<float>(g1_120, g1_120 + sizeof(g1_120) / sizeof(float) );
	gaussian2[120] = std::vector<float>(g2_120, g2_120 + sizeof(g2_120) / sizeof(float) );

	gaussian1[121] = std::vector<float>(g1_121, g1_121 + sizeof(g1_121) / sizeof(float) );
	gaussian2[121] = std::vector<float>(g2_121, g2_121 + sizeof(g2_121) / sizeof(float) );

	gaussian1[122] = std::vector<float>(g1_122, g1_122 + sizeof(g1_122) / sizeof(float) );
	gaussian2[122] = std::vector<float>(g2_122, g2_122 + sizeof(g2_122) / sizeof(float) );

	gaussian1[123] = std::vector<float>(g1_123, g1_123 + sizeof(g1_123) / sizeof(float) );
	gaussian2[123] = std::vector<float>(g2_123, g2_123 + sizeof(g2_123) / sizeof(float) );

	gaussian1[124] = std::vector<float>(g1_124, g1_124 + sizeof(g1_124) / sizeof(float) );
	gaussian2[124] = std::vector<float>(g2_124, g2_124 + sizeof(g2_124) / sizeof(float) );

	gaussian1[125] = std::vector<float>(g1_125, g1_125 + sizeof(g1_125) / sizeof(float) );
	gaussian2[125] = std::vector<float>(g2_125, g2_125 + sizeof(g2_125) / sizeof(float) );

	gaussian1[126] = std::vector<float>(g1_126, g1_126 + sizeof(g1_126) / sizeof(float) );
	gaussian2[126] = std::vector<float>(g2_126, g2_126 + sizeof(g2_126) / sizeof(float) );

	gaussian1[127] = std::vector<float>(g1_127, g1_127 + sizeof(g1_127) / sizeof(float) );
	gaussian2[127] = std::vector<float>(g2_127, g2_127 + sizeof(g2_127) / sizeof(float) );

	gaussian1[128] = std::vector<float>(g1_128, g1_128 + sizeof(g1_128) / sizeof(float) );
	gaussian2[128] = std::vector<float>(g2_128, g2_128 + sizeof(g2_128) / sizeof(float) );

	gaussian1[129] = std::vector<float>(g1_129, g1_129 + sizeof(g1_129) / sizeof(float) );
	gaussian2[129] = std::vector<float>(g2_129, g2_129 + sizeof(g2_129) / sizeof(float) );

	gaussian1[130] = std::vector<float>(g1_130, g1_130 + sizeof(g1_130) / sizeof(float) );
	gaussian2[130] = std::vector<float>(g2_130, g2_130 + sizeof(g2_130) / sizeof(float) );

	gaussian1[131] = std::vector<float>(g1_131, g1_131 + sizeof(g1_131) / sizeof(float) );
	gaussian2[131] = std::vector<float>(g2_131, g2_131 + sizeof(g2_131) / sizeof(float) );

	gaussian1[132] = std::vector<float>(g1_132, g1_132 + sizeof(g1_132) / sizeof(float) );
	gaussian2[132] = std::vector<float>(g2_132, g2_132 + sizeof(g2_132) / sizeof(float) );

	gaussian1[133] = std::vector<float>(g1_133, g1_133 + sizeof(g1_133) / sizeof(float) );
	gaussian2[133] = std::vector<float>(g2_133, g2_133 + sizeof(g2_133) / sizeof(float) );

	gaussian1[134] = std::vector<float>(g1_134, g1_134 + sizeof(g1_134) / sizeof(float) );
	gaussian2[134] = std::vector<float>(g2_134, g2_134 + sizeof(g2_134) / sizeof(float) );

	gaussian1[135] = std::vector<float>(g1_135, g1_135 + sizeof(g1_135) / sizeof(float) );
	gaussian2[135] = std::vector<float>(g2_135, g2_135 + sizeof(g2_135) / sizeof(float) );

	gaussian1[136] = std::vector<float>(g1_136, g1_136 + sizeof(g1_136) / sizeof(float) );
	gaussian2[136] = std::vector<float>(g2_136, g2_136 + sizeof(g2_136) / sizeof(float) );

	gaussian1[137] = std::vector<float>(g1_137, g1_137 + sizeof(g1_137) / sizeof(float) );
	gaussian2[137] = std::vector<float>(g2_137, g2_137 + sizeof(g2_137) / sizeof(float) );

	gaussian1[138] = std::vector<float>(g1_138, g1_138 + sizeof(g1_138) / sizeof(float) );
	gaussian2[138] = std::vector<float>(g2_138, g2_138 + sizeof(g2_138) / sizeof(float) );

	gaussian1[139] = std::vector<float>(g1_139, g1_139 + sizeof(g1_139) / sizeof(float) );
	gaussian2[139] = std::vector<float>(g2_139, g2_139 + sizeof(g2_139) / sizeof(float) );

	gaussian1[140] = std::vector<float>(g1_140, g1_140 + sizeof(g1_140) / sizeof(float) );
	gaussian2[140] = std::vector<float>(g2_140, g2_140 + sizeof(g2_140) / sizeof(float) );

	gaussian1[141] = std::vector<float>(g1_141, g1_141 + sizeof(g1_141) / sizeof(float) );
	gaussian2[141] = std::vector<float>(g2_141, g2_141 + sizeof(g2_141) / sizeof(float) );

	gaussian1[142] = std::vector<float>(g1_142, g1_142 + sizeof(g1_142) / sizeof(float) );
	gaussian2[142] = std::vector<float>(g2_142, g2_142 + sizeof(g2_142) / sizeof(float) );

	gaussian1[143] = std::vector<float>(g1_143, g1_143 + sizeof(g1_143) / sizeof(float) );
	gaussian2[143] = std::vector<float>(g2_143, g2_143 + sizeof(g2_143) / sizeof(float) );

	gaussian1[144] = std::vector<float>(g1_144, g1_144 + sizeof(g1_144) / sizeof(float) );
	gaussian2[144] = std::vector<float>(g2_144, g2_144 + sizeof(g2_144) / sizeof(float) );

	gaussian1[145] = std::vector<float>(g1_145, g1_145 + sizeof(g1_145) / sizeof(float) );
	gaussian2[145] = std::vector<float>(g2_145, g2_145 + sizeof(g2_145) / sizeof(float) );

	gaussian1[146] = std::vector<float>(g1_146, g1_146 + sizeof(g1_146) / sizeof(float) );
	gaussian2[146] = std::vector<float>(g2_146, g2_146 + sizeof(g2_146) / sizeof(float) );

	gaussian1[147] = std::vector<float>(g1_147, g1_147 + sizeof(g1_147) / sizeof(float) );
	gaussian2[147] = std::vector<float>(g2_147, g2_147 + sizeof(g2_147) / sizeof(float) );

	gaussian1[148] = std::vector<float>(g1_148, g1_148 + sizeof(g1_148) / sizeof(float) );
	gaussian2[148] = std::vector<float>(g2_148, g2_148 + sizeof(g2_148) / sizeof(float) );

	gaussian1[149] = std::vector<float>(g1_149, g1_149 + sizeof(g1_149) / sizeof(float) );
	gaussian2[149] = std::vector<float>(g2_149, g2_149 + sizeof(g2_149) / sizeof(float) );

	gaussian1[150] = std::vector<float>(g1_150, g1_150 + sizeof(g1_150) / sizeof(float) );
	gaussian2[150] = std::vector<float>(g2_150, g2_150 + sizeof(g2_150) / sizeof(float) );

	gaussian1[151] = std::vector<float>(g1_151, g1_151 + sizeof(g1_151) / sizeof(float) );
	gaussian2[151] = std::vector<float>(g2_151, g2_151 + sizeof(g2_151) / sizeof(float) );

	gaussian1[152] = std::vector<float>(g1_152, g1_152 + sizeof(g1_152) / sizeof(float) );
	gaussian2[152] = std::vector<float>(g2_152, g2_152 + sizeof(g2_152) / sizeof(float) );

	gaussian1[153] = std::vector<float>(g1_153, g1_153 + sizeof(g1_153) / sizeof(float) );
	gaussian2[153] = std::vector<float>(g2_153, g2_153 + sizeof(g2_153) / sizeof(float) );

	gaussian1[154] = std::vector<float>(g1_154, g1_154 + sizeof(g1_154) / sizeof(float) );
	gaussian2[154] = std::vector<float>(g2_154, g2_154 + sizeof(g2_154) / sizeof(float) );

	gaussian1[155] = std::vector<float>(g1_155, g1_155 + sizeof(g1_155) / sizeof(float) );
	gaussian2[155] = std::vector<float>(g2_155, g2_155 + sizeof(g2_155) / sizeof(float) );

	gaussian1[156] = std::vector<float>(g1_156, g1_156 + sizeof(g1_156) / sizeof(float) );
	gaussian2[156] = std::vector<float>(g2_156, g2_156 + sizeof(g2_156) / sizeof(float) );

	gaussian1[157] = std::vector<float>(g1_157, g1_157 + sizeof(g1_157) / sizeof(float) );
	gaussian2[157] = std::vector<float>(g2_157, g2_157 + sizeof(g2_157) / sizeof(float) );

	gaussian1[158] = std::vector<float>(g1_158, g1_158 + sizeof(g1_158) / sizeof(float) );
	gaussian2[158] = std::vector<float>(g2_158, g2_158 + sizeof(g2_158) / sizeof(float) );

	gaussian1[159] = std::vector<float>(g1_159, g1_159 + sizeof(g1_159) / sizeof(float) );
	gaussian2[159] = std::vector<float>(g2_159, g2_159 + sizeof(g2_159) / sizeof(float) );
}