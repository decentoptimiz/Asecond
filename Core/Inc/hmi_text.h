/*
 * hmi_text.h - Chinese text for the serial screen (GBK code page)
 *
 * The screen's text control expects GBK (Chinese) encoding, while this
 * source tree is UTF-8. All Chinese strings are therefore written as
 * explicit GBK byte escapes so the compiler's source charset and the
 * screen encoding can never mismatch.
 *
 * Font ASC_CHS_30 => ASCII char 15px, Chinese char 30px.
 */
#ifndef __HMI_TEXT_H
#define __HMI_TEXT_H

#define HT_FREQ "\xC6\xB5\xC2\xCA\x3A"   /* pin lv */
#define HT_VPP "\xB7\xE5\xB7\xE5\xD6\xB5\x3A"   /* feng feng zhi */
#define HT_AVG "\xC6\xBD\xBE\xF9\xD6\xB5\x3A"   /* ping jun zhi */
#define HT_PHASE "\xCF\xE0\xCE\xBB\xB2\xEE\x3A"   /* xiang wei cha */
#define HT_WAVE "\xB2\xA8\xD0\xCE\xC0\xE0\xD0\xCD\x3A"   /* bo xing lei xing */
#define HT_SINE "\xD5\xFD\xCF\xD2\xB2\xA8"   /* zheng xian bo */
#define HT_TRI "\xC8\xFD\xBD\xC7\xB2\xA8"   /* san jiao bo */
#define HT_SQUARE "\xB7\xBD\xB2\xA8"   /* fang bo */
#define HT_TRIG "\xB4\xA5\xB7\xA2"   /* chu fa (trigger) */
#define HT_VERT "\xB4\xB9\xD6\xB1"   /* chui zhi (vertical) */
#define HT_HORI "\xCB\xAE\xC6\xBD"   /* shui ping (horizontal) */
#define HT_CHSEL "\xCD\xA8\xB5\xC0"   /* tong dao (channel) */
#define HT_RISE "\xC9\xCF\xC9\xFD"   /* shang sheng (rising edge) */
#define HT_FALL "\xCF\xC2\xBD\xB5"   /* xia jiang (falling edge) */
#define HT_CH1 "\xCD\xA8\xB5\xC0\x31"   /* tong dao 1 */
#define HT_CH2 "\xCD\xA8\xB5\xC0\x32"   /* tong dao 2 */
#define HT_NONE "\x2D\x2D\x2D"   /* no value */

#endif /* __HMI_TEXT_H */
