/*
 * ========================================
 * 鏂囦欢鍚? set_back.c
 * 鎻忚堪: 鑸垫満缁勫悎鎺у埗妯″潡锛堝洖绋嬪姩浣滐級
 * 鍔熻兘: 鍗忚皟澶氫釜鑸垫満鎵ц澶嶆潅鐨勫姩浣滃簭鍒楋紙濡傛斁缃墿浣撴垨鍥炵▼锛?
 * ========================================
 */

#ifndef _SET_BACK_
#define _SET_BACK_
#include <stm32h7xx_hal.h>
#include "fun.c"

#include "HardwareInfo.c"

/*
 * 鍑芥暟鍚? photo_scan
 * 鍙傛暟:
 *   s0_data - 鑸垫満0鐨勭洰鏍囪搴︽暟鎹?(鑼冨洿: 500~2500)
 *   s1_data - 鑸垫満1鐨勭洰鏍囪搴︽暟鎹?(鑼冨洿: 500~2500)
 *   s2_data - 鑸垫満2鐨勭洰鏍囪搴︽暟鎹?(鑼冨洿: 500~2500)
 *   s3_data - 鑸垫満3鐨勭洰鏍囪搴︽暟鎹?(鑼冨洿: 500~2500)
 *   s4_data - 鑸垫満4鐨勭洰鏍囪搴︽暟鎹?(鑼冨洿: 500~2500)
 *   s5_data - 鑸垫満5鐨勭洰鏍囪搴︽暟鎹?(鑼冨洿: 500~2500)
 *   time - 鎬绘墽琛屾椂闂?(鍗曚綅: 姣)
 * 鎻忚堪:
 *   鎺у埗澶氫釜鑸垫満(0銆?銆?銆?銆?銆?)鑷畾涔夎搴︼紝
 *   閫氬父鐢ㄤ簬璁捐鑷畾涔夊姩浣溿€?
 */
void photo_scan(int s0_data, int s1_data, int s2_data, int s3_data, int s4_data, int s5_data, int time)
{
    // 鑸垫満0鎵ц鍓嶅崐娈垫椂闂?
    // 鍙傛暟: 鑸垫満ID(0), 鐩爣瑙掑害(s1_data), 鎵ц鏃堕棿(time/2)
    servo_control(0, s0_data, time);

    // 鑸垫満1鎵ц瀹屾暣鏃堕棿
    // 鍙傛暟: 鑸垫満ID(1), 鐩爣瑙掑害(servo1), 鎵ц鏃堕棿(time)
    servo_control(1, s1_data, time);

    // 鑸垫満2鎵ц鍥哄畾浣嶇疆
    // 鍙傛暟: 鑸垫満ID(2), 鐩爣瑙掑害(1150), 鎵ц鏃堕棿(time)
    servo_control(2, s2_data, time);

    // 鑸垫満3鎵ц鍥哄畾浣嶇疆
    // 鍙傛暟: 鑸垫満ID(3), 鐩爣瑙掑害(2200), 鎵ц鏃堕棿(time)
    servo_control(3, s3_data, time);

    // 鑸垫満2鎵ц鍥哄畾浣嶇疆
    // 鍙傛暟: 鑸垫満ID(4), 鐩爣瑙掑害(1150), 鎵ц鏃堕棿(time)
    servo_control(4, s4_data, time);

    // 鑸垫満3鎵ц鍥哄畾浣嶇疆
    // 鍙傛暟: 鑸垫満ID(5), 鐩爣瑙掑害(2200), 鎵ц鏃堕棿(time)
    servo_control(5, s5_data, time);

    // 绛夊緟鎵€鏈夎埖鏈烘墽琛屽畬鎴愶紙杞崲姣涓虹锛?
    SetWaitForTime((float)time / 1000);
}

#endif