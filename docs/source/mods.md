# Hardware mods

## BOOT and RESET buttons

Some of the ESP32-SI4732 receivers do not have the BOOT and RESET buttons soldered in. You will need these buttons if you want to recover a receiver that was bricked because of a failed flashing process.

You can use a pair of precision tweezers like this to short the button contacts:

![](_static/precision-tweezers.jpg)

Or solder the missing buttons:

![](_static/button.jpg) ![](_static/boot-reset-buttons-missing.jpg) ![](_static/boot-reset-buttons-soldered.jpg)

## Speaker

Some of the units have a smaller speaker. You can install a bigger one (1511) that is used in the original project ([aliexpress.com](https://www.aliexpress.com/item/1005006309723573.html), [aliexpress.ru](https://aliexpress.ru/item/1005005725740853.html)):

![](_static/speaker.jpg)

## Hi-Z match circuit

[ Micro Pocket SSB Receiver - NEW FIRMWARE + Hi-Z CIRCUIT Mod](https://youtu.be/BzrOE9BFpyU?feature=shared&t=916)

## Headphone amplifier

* <https://t.me/talkradio/174172/184530>

Note: a new hardware revision with the LM4809 headphone amplifier has been spotted, see <https://forum.vcfm.ru/viewtopic.php?f=9&t=2395&start=60#p187797>

## 3D-printed cases

* <https://www.printables.com/model/1295639-ats-mini-receiver-housing>

## On-device FT8 (GPIO11 ADC)

V4a/V4b boards already route radio audio to **ESP32 GPIO11** for digimode decoding — no soldering and no external ADC. Flash firmware with Waterfall enabled, open **UI Layout → Waterfall**, set **Mode → FT8** and **Date/Time** to UTC, then watch Serial for `AudioCapture rms=… rate=…` and `Ft8Local: decode …`.

Wire **SI4732 pin 1 (LOUT) → 4.7–10 kΩ → GPIO11**, plus **10–47 nF GPIO11→GND**. ADC capture runs only in **Mode → FT8** (not in USB/LSB/AM on the Waterfall layout), mutes the speaker amp while sampling, and is stopped before SSB patch loads so the UI does not freeze. Status `b=` ≈ 1800–2200 means a good tap. See [discussion #162](https://github.com/esp32-si4732/ats-mini/discussions/162).

## Other interesting mods

* [Si4732 MiniRX Modifications](https://peterneufeld.wordpress.com/2025/06/13/si4732a-minirx-modifications/) (also see [#73](https://github.com/esp32-si4732/ats-mini/discussions/73))
* [Si4732 Mini & ATS-20+ Hybrid Radio](https://www.youtube.com/watch?v=OyrTHNhbh58)
