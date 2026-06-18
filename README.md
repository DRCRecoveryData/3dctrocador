# 3DC Changer Installation (Color Changer for Klipper)

This project provides a practical way to use color-changing systems (such as the 3D Chameleon) with Klipper. It is designed for the Creality K1 series (K1, K1C, K1SE, K1MAX) and features automatic updates via Moonraker.

---

## 📦 Requirements

- Klipper already installed and running
- SSH or terminal access to your printer
- Active internet connection on the printer

---

## 🚀 Installation

Run the commands below directly in your printer's terminal:
```bash
git clone --depth 1 [https://github.com/Igordarin33/3dctrocador.git](https://github.com/Igordarin33/3dctrocador.git) /usr/data/printer_data/config/3DC

```
## 🔧 Activating camaleaoTURBO.cfg
Edit your main configuration file (printer.cfg) and add this line at the very beginning:
```ini
[include 3DC/camaleaoTURBO.cfg]

```
### Next, comment out the complete code block for the filament sensors:
Comment out both [filament_switch_sensor filament_sensor] and [filament_switch_sensor filament_sensor_2].
## 🛠️ Configuring Moonraker for Automatic Updates
Edit your /usr/data/printer_data/config/moonraker.conf file and add the following block:
```ini
[update_manager 3dc_macros]
type: git_repo
channel: stable
path: /usr/data/printer_data/config/3DC
origin: [https://github.com/Igordarin33/3dctrocador.git](https://github.com/Igordarin33/3dctrocador.git)
install_script: install.sh
managed_services:
  klipper

```
## 🛠️ Configuring gcode_macro.cfg
Comment out your original filament loading code and replace it with this one.
> 💡 **Note:** You should adjust the temperature in this macro. This will be the default temperature used whenever filament is loaded. It is recommended to set it to 240°C for ABS, 200°C for PLA, or edit it to match your preferences!
> 
```ini
[gcode_macro LOAD_MATERIAL]
gcode:
  {% if printer.extruder.temperature < 220 %}
      M118 Preheating to 220°C
      SET_HEATER_TEMPERATURE HEATER=extruder TARGET=220
  {% endif %}
  LOAD_MATERIAL_CLOSE_FAN2
  G91
  G1 E45 F300
  LOAD_MATERIAL_RESTORE_FAN2

```
## ♻️ Future Updates
Once configured, you will be able to update your macros directly through the Moonraker interface (under the "Updates" section in Fluidd/Mainsail).
# ✅ All Done!
After following the steps above, simply restart Klipper and your color-changing system will be installed and ready to use! Now, just finish adjusting your slicer's Start/End G-code.
# ❓ How Does It Work?
## 🖲️ Extruder Selection
Each extruder is selected based on the number of button press pulses:
| Pulses | Action |
|---|---|
| 1st | Selects T0 (Filament 1) |
| 2nd | Selects T1 (Filament 2) |
| 3rd | Selects T2 (Filament 3) |
| 4th | Selects T3 (Filament 4) |
| 5th | Selects T4 (Filament 5) |
| 6th | Selects T5 (Filament 6) |
| 7th | Selects T6 (Filament 7) |
| 8th | Selects T7 (Filament 8) |
| 9th | Selects Load / Start T0 |
| 10th | Selects Unload / Start |
| 11th | Selects Start |
| 12th | Selects Next |
| 13th | Selects Random |
| 14th | Selects Extra Pulse |
## 🤖 Internal Mechanics
The system utilizes two filament sensors:
 1. One to detect filament entry (extruder sensor).
 2. One to detect filament exit (the original rear sensor).
The printer toggles pin PA0 to select and feed the desired filament. When the filament reaches the extruder sensor, it is automatically grabbed and pulled. The unloading sequence works similarly, shaping the filament tip to ensure smooth and reliable swaps.
## 💬 Community & Parts
 * **Join our WhatsApp Group:** WhatsApp
 * **Printable Files & Parts List:** Printables
# 💖 Support This Project
If this project helped you or you believe in what I am building, please consider supporting it with a donation. Your support motivates me to keep improving, updating, and creating new free tools for the 3D printing community!
🔗 **Pix Key (Email):** igordarin34@gmail.com
## Credits
 * 3DChameleon
```
