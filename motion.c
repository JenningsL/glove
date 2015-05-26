#include "motion.h"
#include <math.h>
#include <stdlib.h>
#define BLOCKSIZE 7204*2 //һ�����������в����Ĵ�С in Byte 7204
//��ȡ��Ӧid��flash���ݿ�����ݵ�theta,uj,sj��

//float theta[601]={1.453210,0.279501,0.288659,0.090089,0.202582,-0.099591,0.219694,0.305309,0.251228,0.115554,0.226482,-0.098607,-0.028152,0.334372,0.213126,0.140293,0.240471,-0.097347,-0.258557,0.360442,0.169142,0.165111,0.272285,-0.249783,-0.155328,0.369893,0.122988,0.189005,0.300128,-0.040416,-0.202412,0.378911,0.077175,0.211944,0.283267,-0.032341,-0.291491,0.389399,0.031352,0.233917,0.300014,-0.195229,-0.239525,0.386713,-0.013909,0.255424,0.295278,-0.282386,-0.310315,0.375542,-0.057470,0.276669,0.312376,-0.105959,-0.233020,0.363274,-0.096108,0.296682,0.333365,0.107581,-0.253246,0.355610,-0.130111,0.312219,0.276415,-0.037707,-0.306381,0.350337,-0.159775,0.326372,0.243300,-0.341691,-0.184190,0.337253,-0.185662,0.333705,0.156572,-0.285601,-0.238268,0.318300,-0.208572,0.332753,0.101895,-0.132397,-0.066683,0.294265,-0.225280,0.327209,0.021125,-0.113409,-0.158876,0.268690,-0.237104,0.318096,0.099441,-0.027935,-0.160758,0.241502,-0.244354,0.306676,0.091223,0.006025,-0.054759,0.211814,-0.249572,0.293655,0.061577,-0.111647,-0.086856,0.181808,-0.251181,0.275310,0.054069,-0.089101,-0.129308,0.153051,-0.248453,0.257171,-0.063829,-0.119205,-0.150589,0.127112,-0.244031,0.236381,0.120923,0.075602,-0.182858,0.103608,-0.235444,0.214331,-0.030429,0.081674,-0.163443,0.083549,-0.224598,0.193087,-0.017759,-0.073233,-0.209554,0.066126,-0.210935,0.165913,-0.266100,0.007099,-0.178600,0.061384,-0.193986,0.138605,-0.346896,0.080732,-0.117386,0.060500,-0.175472,0.117885,-0.181605,0.219240,-0.149330,0.060104,-0.155361,0.105327,-0.101344,-0.045667,-0.103993,0.067131,-0.135089,0.090250,-0.080721,0.009087,-0.105596,0.078412,-0.114111,0.072496,-0.128869,0.025088,-0.095993,0.084763,-0.092964,0.061797,-0.120172,-0.047009,-0.146535,0.088835,-0.072760,0.060448,-0.083252,-0.081920,-0.075908,0.097679,-0.052827,0.061388,-0.044151,0.014789,-0.239877,0.111725,-0.032882,0.068262,-0.112617,-0.001831,-0.212492,0.124609,-0.013149,0.083294,-0.160633,0.012918,-0.113724,0.138505,0.005492,0.101315,-0.191495,-0.073418,-0.138869,0.151917,0.024022,0.123246,-0.110224,0.063445,-0.191720,0.164435,0.043215,0.143192,-0.087134,-0.006924,-0.140311,0.175903,0.061308,0.159213,-0.140445,0.033153,-0.110805,0.182832,0.078235,0.173356,-0.095095,-0.018039,-0.240891,0.191542,0.096041,0.177595,-0.073689,0.035826,-0.205046,0.198198,0.113947,0.176861,-0.200649,-0.026901,-0.231124,0.204554,0.130778,0.165896,-0.169581,-0.044795,-0.204965,0.209725,0.146911,0.147219,-0.216913,-0.042740,-0.136461,0.211939,0.162771,0.127286,-0.211591,0.032417,-0.227880,0.210182,0.177989,0.109494,-0.159141,-0.006842,-0.214977,0.205252,0.192823,0.089103,-0.149572,0.078866,-0.316140,0.200536,0.208235,0.067074,-0.129902,0.053831,-0.318757,0.191525,0.221885,0.047321,-0.149841,0.063630,-0.418492,0.178772,0.235132,0.034608,-0.115603,-0.004648,-0.272525,0.170390,0.247669,0.024076,-0.070837,0.112189,-0.306249,0.164440,0.258641,0.010320,-0.086814,0.087735,-0.257092,0.155810,0.267802,-0.004221,-0.081690,-0.031241,-0.237138,0.139583,0.274553,-0.015754,-0.066647,-0.045581,-0.192873,0.120399,0.279780,-0.025842,-0.058850,0.020037,-0.212577,0.104631,0.283521,-0.035632,-0.088166,0.085359,-0.303973,0.091818,0.285975,-0.046488,-0.110648,0.087850,-0.168424,0.078014,0.287861,-0.057218,-0.110824,-0.062388,-0.120166,0.066440,0.290406,-0.065943,-0.106746,-0.092219,-0.213609,0.059932,0.292683,-0.073533,-0.095700,-0.058427,-0.085706,0.052866,0.292973,-0.077231,-0.079172,0.056899,-0.048927,0.046366,0.291190,-0.080599,-0.138929,0.077715,-0.033361,0.036497,0.288118,-0.084397,-0.060162,-0.094248,-0.148431,0.025142,0.285967,-0.086508,-0.071588,-0.088920,0.000828,0.018234,0.283135,-0.084431,-0.173394,0.009349,0.003735,0.011166,0.279398,-0.082568,-0.074335,0.024445,-0.096552,-0.000083,0.274318,-0.082080,-0.123679,0.031004,0.063605,-0.014949,0.268719,-0.081134,-0.054561,0.011294,0.073194,-0.029367,0.263163,-0.078481,-0.082508,-0.110981,0.111757,-0.040855,0.257208,-0.078110,-0.145416,-0.038499,0.083482,-0.051887,0.250572,-0.079509,-0.070443,0.054246,0.153620,-0.066860,0.245302,-0.080082,-0.034547,0.139249,0.061905,-0.084434,0.239280,-0.080724,-0.073653,0.184975,0.150242,-0.094495,0.234148,-0.081667,-0.106780,0.120747,0.122400,-0.097825,0.230621,-0.083465,-0.152562,0.024693,0.046571,-0.102043,0.227878,-0.084645,-0.111458,-0.098637,-0.015287,-0.111633,0.224749,-0.085024,-0.104720,-0.046345,0.089716,-0.121242,0.222231,-0.083701,-0.097582,0.147617,-0.045523,-0.128663,0.219860,-0.083127,-0.074485,0.020836,0.008692,-0.134595,0.218104,-0.082348,-0.028539,-0.043092,0.132024,-0.139816,0.215925,-0.083288,-0.061300,0.005064,-0.005387,-0.149631,0.213640,-0.084022,-0.054656,0.021688,0.029363,-0.156436,0.212181,-0.085888,-0.001969,0.109013,-0.030488,-0.162034,0.210644,-0.088022,-0.019798,0.186177,0.135548,-0.168762,0.210488,-0.090565,-0.007547,0.362299,-0.012671,-0.172482,0.209638,-0.092595,-0.005191,0.150281,-0.057791,-0.172633,0.208670,-0.092393,-0.010774,0.070658,0.023773,-0.172901,0.207690,-0.092194,-0.036378,-0.015356,0.086572,-0.172997,0.206913,-0.091437,-0.044395,0.002916,-0.108700,-0.174870,0.205979,-0.090735,-0.078367,0.091716,-0.037268,-0.176465,0.205085,-0.090365,-0.070914,0.113696,0.121505,-0.179019,0.204197,-0.090162,-0.037799,0.126428,0.089995,-0.181183,0.204059,-0.089033,0.019604,-0.037503,0.122351,-0.184810,0.202726,-0.088327,0.078385,-0.220974,0.158870,-0.190276,0.199727,-0.088084,0.023425,0.042248,0.029070,-0.195841,0.196652,-0.088533,0.000713,0.022229,0.077046,-0.200573,0.194861,-0.088781,-0.060531,-0.124449,0.041129,-0.204840,0.191617,-0.089925,-0.021167,-0.039163,0.011285,-0.211205,0.188070,-0.090975,-0.005234,-0.042725,-0.008345,-0.219013,0.184484,-0.092443,-0.043709,-0.048024,0.062510,-0.226549,0.181163,-0.093468,-0.177048,-0.092883,0.114519};
//float uj[600]={3636.658333,14338.458333,3080.091667,-328.100000,-135.500000,3.891667,3733.550000,13950.358333,3129.991667,-291.666667,-131.158333,-11.183333,3831.591667,13543.791667,3164.775000,-270.216667,-148.341667,-36.900000,3921.150000,13122.150000,3185.608333,-259.558333,-171.933333,-45.558333,3997.100000,12690.441667,3184.283333,-257.408333,-152.658333,-45.625000,4070.258333,12264.116667,3159.191667,-238.125000,-146.316667,-60.600000,4141.650000,11844.541667,3111.916667,-191.050000,-169.900000,-54.150000,4196.600000,11435.608333,3041.950000,-167.566667,-193.425000,-47.725000,4239.150000,11034.666667,2948.016667,-180.366667,-163.341667,-43.425000,4281.825000,10655.475000,2833.466667,-191.158333,-137.675000,-26.275000,4311.258333,10298.891667,2713.233333,-163.316667,-114.125000,12.283333,4337.075000,9961.958333,2578.083333,-133.350000,-99.066667,31.583333,4358.566667,9649.791667,2420.900000,-150.550000,-96.975000,50.858333,4375.108333,9360.383333,2251.333333,-180.550000,-79.758333,68.033333,4379.366667,9099.900000,2081.733333,-172.008333,-15.466667,78.783333,4394.491667,8870.991667,1894.108333,-195.608333,-13.366667,78.775000,4420.200000,8672.675000,1691.341667,-199.941667,-67.008333,76.633333,4435.958333,8485.775000,1485.850000,-165.725000,-81.900000,93.808333,4448.650000,8322.441667,1276.616667,-150.725000,-54.016667,80.958333,4469.325000,8186.208333,1068.691667,-122.858333,-26.275000,93.866667,4490.250000,8062.083333,867.808333,-107.891667,-24.075000,104.566667,4504.425000,7977.575000,684.650000,-107.816667,-15.500000,115.266667,4523.275000,7923.058333,519.550000,-92.883333,-15.516667,121.733333,4554.950000,7898.708333,367.200000,-116.458333,-30.533333,104.616667,4600.450000,7900.000000,235.183333,-110.108333,-47.658333,113.091667,4648.683333,7917.308333,123.875000,-97.183333,-36.950000,128.116667,4694.041667,7953.150000,30.300000,-97.116667,-9.125000,119.550000,4759.125000,7997.266667,-42.741667,-77.816667,-4.783333,123.833333,4828.275000,8047.316667,-92.766667,-56.425000,3.750000,117.458333,4883.641667,8109.975000,-122.958333,-41.358333,-4.783333,121.708333,4942.000000,8174.183333,-145.875000,-28.525000,12.366667,113.050000,5003.600000,8249.275000,-148.600000,3.591667,53.125000,89.558333,5072.491667,8336.933333,-126.200000,-0.616667,55.300000,106.616667,5146.675000,8435.750000,-94.758333,12.158333,87.383333,113.016667,5214.850000,8539.983333,-68.100000,10.041667,83.158333,100.216667,5261.625000,8654.433333,-20.583333,59.383333,98.125000,89.425000,5296.233333,8783.708333,49.991667,102.291667,89.500000,57.291667,5331.708333,8917.891667,122.941667,102.208333,93.775000,35.916667,5346.408333,9058.283333,189.508333,82.908333,91.725000,35.891667,5352.241667,9218.341667,253.566667,52.900000,128.216667,1.558333,5361.808333,9386.091667,310.891667,72.116667,164.508333,-22.091667,5361.025000,9561.983333,359.550000,72.166667,160.266667,-26.325000,5354.591667,9755.100000,401.950000,55.091667,175.266667,-39.158333,5341.133333,9947.791667,436.191667,70.041667,141.016667,-45.600000,5316.158333,10136.016667,466.175000,76.450000,128.116667,-84.183333,5274.158333,10317.066667,489.325000,80.758333,141.016667,-92.758333,5234.608333,10513.308333,510.075000,100.083333,175.258333,-112.066667,5191.875000,10713.775000,525.166667,102.283333,132.458333,-148.450000,5143.650000,10906.958333,539.725000,140.891667,68.175000,-154.883333,5086.950000,11075.508333,557.958333,166.583333,74.608333,-163.441667,5027.191667,11235.783333,572.891667,160.208333,102.316667,-193.408333,4965.925000,11388.658333,571.550000,143.025000,78.791667,-165.541667,4893.283333,11521.125000,566.258333,138.700000,31.691667,-135.641667,4800.200000,11631.458333,567.858333,155.858333,65.958333,-122.750000,4709.208333,11721.633333,577.716667,158.075000,95.966667,-90.583333,4629.316667,11789.158333,583.400000,153.758333,108.808333,-51.975000,4545.841667,11828.266667,577.608333,134.516667,68.116667,-45.558333,4459.575000,11859.008333,569.258333,155.875000,31.583333,-26.291667,4378.400000,11870.141667,574.450000,151.600000,8.175000,-9.125000,4287.633333,11857.291667,586.283333,151.583333,44.541667,-9.041667,4192.525000,11836.150000,599.241667,138.750000,70.200000,-17.525000,4109.750000,11804.925000,610.791667,151.600000,35.916667,-24.000000,4017.291667,11759.875000,628.041667,153.750000,10.266667,-26.158333,3918.333333,11710.866667,658.116667,147.316667,35.941667,-11.150000,3834.583333,11667.341667,692.600000,140.916667,42.416667,-2.616667,3767.191667,11627.866667,727.641667,119.458333,21.016667,-6.858333,3691.150000,11569.525000,759.591667,104.466667,14.616667,-30.416667,3605.991667,11509.283333,795.816667,121.608333,35.966667,-45.391667,3529.141667,11456.850000,832.258333,119.425000,53.158333,-41.158333,3465.341667,11413.983333,864.266667,132.233333,65.925000,-41.108333,3406.991667,11372.091667,896.716667,112.983333,59.541667,-45.466667,3348.033333,11325.408333,937.633333,127.941667,42.408333,-56.225000,3289.966667,11287.208333,978.758333,125.808333,21.033333,-62.625000,3246.741667,11260.958333,1014.608333,112.991667,46.716667,-62.633333,3212.825000,11245.183333,1043.766667,104.375000,55.275000,-79.741667,3176.025000,11237.466667,1077.391667,108.666667,38.250000,-96.900000,3142.883333,11240.875000,1114.925000,127.908333,33.958333,-94.758333,3113.125000,11250.766667,1155.833333,136.416667,42.475000,-105.483333,3096.233333,11271.966667,1191.958333,121.466667,53.150000,-84.133333,3087.200000,11298.358333,1227.108333,119.375000,44.683333,-82.083333,3078.175000,11328.816667,1263.616667,127.883333,53.150000,-88.441667,3069.175000,11365.033333,1300.191667,130.066667,42.500000,-94.841667,3064.475000,11403.800000,1332.691667,130.008333,48.966667,-103.325000,3067.100000,11453.316667,1357.750000,125.716667,59.625000,-96.941667,3064.200000,11508.475000,1385.008333,110.766667,48.941667,-92.658333,3059.308333,11564.941667,1415.708333,110.733333,55.400000,-94.741667,3059.925000,11623.966667,1448.150000,95.708333,72.525000,-105.341667,3061.641667,11683.975000,1478.441667,106.425000,55.400000,-99.033333,3064.000000,11749.100000,1506.616667,115.008333,51.016667,-92.608333,3061.383333,11815.700000,1533.675000,108.558333,38.208333,-88.333333,3049.433333,11879.200000,1559.608333,93.533333,48.966667,-96.783333,3034.300000,11943.758333,1582.583333,82.825000,61.766667,-90.433333,3028.433333,12010.358333,1604.250000,85.000000,44.516667,-101.091667,3022.600000,12076.625000,1623.241667,93.516667,12.433333,-92.591667,3009.516667,12140.733333,1644.341667,91.358333,27.391667,-84.016667,2992.175000,12201.150000,1661.633333,69.966667,31.750000,-88.241667,2974.300000,12259.591667,1674.316667,69.975000,18.966667,-96.791667,2958.350000,12321.466667,1686.091667,74.241667,21.066667,-103.241667,2938.725000,12382.841667,1699.408333,76.425000,18.858333,-81.941667,2916.175000,12440.825000,1708.325000,55.075000,23.175000,-84.041667};
//int sj[600]={10692,29861,23737,4884,2314,1286,10793,28754,23059,4884,2314,1286,11014,27555,22278,4884,1800,1287,11198,27309,21400,4370,1800,1543,11305,26957,20510,3856,2057,1287,11406,26568,19516,3855,2057,2057,11483,26172,18599,3599,1800,2314,11518,25714,17957,3856,2057,1800,11490,25636,17302,3085,1800,1800,11480,25830,16599,2828,2314,1800,11428,26046,16108,2828,2057,1800,11322,26243,15573,2571,1286,1800,11334,26413,15105,2314,1800,1800,11431,26620,14665,2057,2057,2058,11459,26865,14263,2057,1543,2057,11633,27127,13794,1543,1543,2314,11813,27382,13277,1286,1029,2571,11988,27625,12762,1286,1799,2571,12216,27837,12295,1286,1800,2571,12449,28040,11821,1286,1543,2828,12600,28236,11371,1029,1286,2571,12693,28440,11037,1286,1029,2314,12770,28703,10828,1286,1286,2314,12881,28942,10750,1286,1543,2571,12953,29142,10712,1543,1543,2571,12938,29325,10515,1800,1286,2571,12912,29486,10148,2314,1543,2828,13028,29661,9946,3085,3085,2571,13097,29833,9935,3342,2571,2571,13181,29958,9702,3342,2314,2571,13269,30032,9301,3342,2571,2314,13297,30102,8910,3342,2828,2314,13271,30147,8890,3856,3342,2571,13272,30239,8946,3856,3598,2571,13277,30633,8957,3599,2314,2314,13217,31093,8929,3856,2571,2571,13089,31542,8861,4113,3342,2571,12990,32088,8839,4113,3599,2315,12884,32564,8829,3856,3085,2314,12807,32854,8765,4113,2572,2314,12966,33154,8834,3599,3085,2571,13122,33411,9662,4114,3598,2571,13143,33586,10865,3342,3856,2828,13155,33732,11839,3342,3086,2828,13207,33882,12893,3856,3342,2828,13340,33765,14265,3599,3342,2828,13507,33733,15743,3085,2828,2828,13600,34162,17002,3086,3856,2828,13560,34319,18006,3342,2828,2828,13496,34425,18754,3856,3085,3085,13553,34567,19558,3085,3342,3085,13719,34532,20386,3085,3599,3085,13721,34560,20956,3599,3856,3085,13873,34855,21364,3599,2571,2828,14092,35081,21763,3085,3598,2571,14568,35075,22150,2828,3342,2314,15003,34802,22514,3342,2828,2571,15306,34691,22620,3342,2571,2314,15467,34543,22656,3085,1800,2057,15440,34381,22779,2571,2057,2057,15351,34310,22988,3085,2314,2057,15331,34317,22944,3342,1544,2571,15272,34295,22720,2828,1543,2057,15251,33969,22529,2314,2057,1800,15196,33776,22290,2314,2571,1544,15208,33818,22058,2828,2571,1800,15229,33627,21988,3342,1800,2057,15199,33566,22092,3598,1286,2056,15171,33565,22134,3085,2057,2057,15348,33455,22230,2571,2314,2314,15488,33025,22007,2314,2571,2057,15557,32807,21868,2828,1029,1800,15617,33035,21925,2572,1543,2057,15682,33069,22051,2828,2571,2057,15816,33025,21918,2828,2057,2057,15883,32944,21617,2828,1543,1800,15877,32888,21430,2571,1800,1798,15843,32656,21351,2571,2571,2057,15874,32288,21275,2314,3342,2314,15948,32013,21088,2826,2058,2057,16006,31849,20912,2569,2828,2314,16192,31706,20722,2569,2314,2057,16252,31478,20542,2571,1286,2056,16188,31298,20490,2571,1029,2055,16397,31198,20473,2826,2314,2571,16622,31262,20529,2828,2314,2057,16677,31213,20875,2312,2571,1800,16825,31287,21640,2571,2827,2057,16879,31304,22045,3085,1286,2055,16798,31324,22169,3342,2057,2056,16582,31374,22302,2828,2057,2057,16299,31449,22476,2314,2314,1800,16120,31437,22484,2314,1029,1798,16042,31402,22287,2314,1800,1798,16083,31371,22209,2314,1800,1800,15927,31378,22375,1800,1286,2057,15911,31361,22372,1543,1543,2054,16062,31290,22124,1800,2057,2055,16009,31219,21834,1798,1543,1798,15907,31144,21667,1541,772,1798};

float theta[601];
float uj[600];
int sj[600];
void getParamById(int id){
  u32 flash_begin_address=id*BLOCKSIZE;//���ݶ���idȷ����ʼ��ַ
  spi_read_floatArr(theta,flash_begin_address,601);
  flash_begin_address+=sizeof(theta);
  spi_read_floatArr(uj,flash_begin_address,600);
  flash_begin_address+=sizeof(uj);
  spi_read_intArr(sj,flash_begin_address,600);//��flash
}
//��theta,uj,sj�е����ݴ洢����Ӧid��flash���ݿ�
void setParamById(int id){
  u32 flash_begin_address=id*BLOCKSIZE;//���ݶ���idȷ����ʼ��ַ
  spi_write_floatArr(theta,flash_begin_address,601);
  flash_begin_address+=sizeof(theta);
  spi_write_floatArr(uj,flash_begin_address,600);
  flash_begin_address+=sizeof(uj);
  spi_write_intArr(sj,flash_begin_address,600); //дflash
}
//�Դ��������������й�һ��
void nomarlize(float *rawData){
  for(int x=0;x<600;x++){
    rawData[x]=(rawData[x]-uj[x])/sj[x];
  }
}
//�����Ƿ�ĳ������
float test(float *dataArr){
   
   nomarlize(dataArr);
   float z=theta[0];
   for(int x=0;x<600;x++){
     z+=dataArr[x]*theta[x+1];
   }
   float result;
   result=1/(1+exp(-z));
   return result;
}
//������Ľ϶������������Բ�ֵ��չ�ɱ�׼����
void expand(float *oldData,float *newData,int oldLen){
  int lack=100-oldLen,
      gaps=oldLen-1,
      average=floor(lack/gaps),//ƽ��ÿ����϶�����ٸ���,����ȡ��
      stilLack=lack-average*gaps;//ÿ��������average�������Ȼʣ�¶��ٸ���Ҫ����
  //int fillPlan[100];
  int *fillPlan;
  fillPlan=malloc(gaps*sizeof(int));//��̬�����ڴ�
  for(int x=0;x<gaps;x+=1){
    fillPlan[x]=average;
  }
  int i=gaps-1;//���һ��������±�
  while(stilLack>0){
    fillPlan[i]+=1;
    stilLack-=1;
    i-=1;
  }
  
  int index=0;//����Ŀǰ�Ѿ������˼������ݵ�
  for(int x=0;x<oldLen;x+=1){
    if(x==(oldLen-1)){
      newData[index*6]=oldData[x*6];
      newData[index*6+1]=oldData[x*6+1];
      newData[index*6+2]=oldData[x*6+2];
      newData[index*6+3]=oldData[x*6+3];
      newData[index*6+4]=oldData[x*6+4];
      newData[index*6+5]=oldData[x*6+5];//���һ�����ݵ�ֱ�Ӳ���
      continue;
    }
    newData[index*6]=oldData[x*6];
    newData[index*6+1]=oldData[x*6+1];
    newData[index*6+2]=oldData[x*6+2];
    newData[index*6+3]=oldData[x*6+3];
    newData[index*6+4]=oldData[x*6+4];
    newData[index*6+5]=oldData[x*6+5];//����һ��ԭ�е����ݵ�
    index+=1;
    float interval=1/(fillPlan[x]+1);
    for(int y=1;y<=fillPlan[x];y+=1){
      newData[index*6+y]=oldData[x*6]+y*interval*(oldData[x*6+6]-oldData[x*6]);
      newData[index*6+y+1]=oldData[x*6+1]+y*interval*(oldData[x*6+7]-oldData[x*6+1]);
      newData[index*6+y+2]=oldData[x*6+2]+y*interval*(oldData[x*6+8]-oldData[x*6+2]);
      newData[index*6+y+3]=oldData[x*6+3]+y*interval*(oldData[x*6+9]-oldData[x*6+3]);
      newData[index*6+y+4]=oldData[x*6+4]+y*interval*(oldData[x*6+10]-oldData[x*6+4]);
      newData[index*6+y+5]=oldData[x*6+5]+y*interval*(oldData[x*6+11]-oldData[x*6+5]);
      index+=1;
    }
  }
  free(fillPlan);//�ͷ��ڴ�
}
void compress(float *excessData,float *newData,int excessLen){
  int sampleGap=ceil(excessLen/100);//���Թ���ÿ�������ݲ���һ�Σ�����ȡ��
  
  uint8_t *samplePlan;//ÿλ�����Ƿ�����λ����
  samplePlan=malloc(excessLen*sizeof(char));
  uint8_t keptCount=0;//�Ѿ������˼���������
  for(int x=0;x<excessLen;x++){
    if(x%sampleGap==0){
      samplePlan[x]=1;//����
      keptCount+=1;
      
    }else{
      samplePlan[x]=0;//������
    }
    
  }
  uint8_t lack=100-keptCount;//��ȱ���ٸ�
  for(int x=0;x<excessLen;x++){
    if(lack==0){
      break;
    }
    else if(!samplePlan[x]){
      samplePlan[x]=1;
      lack--;
    }
  }//ǰ����������ȱ���
  uint8_t index=0;
  for(int x=0;x<excessLen;x++){
    if(samplePlan[x]){
      newData[index*6]=excessData[x*6];
      newData[index*6+1]=excessData[x*6+1];
      newData[index*6+2]=excessData[x*6+2];
      newData[index*6+3]=excessData[x*6+3];
      newData[index*6+4]=excessData[x*6+4];
      newData[index*6+5]=excessData[x*6+5];
    }
  }
}