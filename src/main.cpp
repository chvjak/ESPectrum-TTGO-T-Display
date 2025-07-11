/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or
visit https://zxespectrum.speccy.org/contacto

*/

#include "ESPectrum.h"
#include "FileUtils.h"
#include "menu1.h"
#include "menu_bt.h"
#include "Config.h"
TaskHandle_t mainTaskHandle;

extern "C" void app_main(void) {
  FileUtils::initFileSystem();
  std::string selected = showZ80FileMenu();
  if (!selected.empty()) {
    Config::ram_file = selected;
    Config::save();
  }

  showBTMenu();

  ESPectrum::setup();
  xTaskCreatePinnedToCore(&ESPectrum::loopTask, "mainTask", 5024, NULL, 5, &mainTaskHandle, 1);
}