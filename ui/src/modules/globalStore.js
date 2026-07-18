/*
 * Chamber Controller UI
 * Copyright (c) 2021-2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

import { defineStore } from 'pinia'
import { logInfo, logDebug, logError } from '@mp-se/espframework-ui-components'
import { sharedHttpClient as http } from '@mp-se/espframework-ui-components'

export const useGlobalStore = defineStore('global', {
  state: () => {
    return {
      id: '',
      platform: '',
      board: '',
      app_ver: '',
      app_build: '',
      firmware_file: '',

      initialized: false,
      disabled: false,
      configChanged: false,

      ui: {
        enableVoltageFragment: false,
        enableManualWifiEntry: false,
        enableScanForStrongestAp: false
      },

      feature: {
        ble: true,
        ble_sensor: true
      },

      messageError: '',
      messageWarning: '',
      messageSuccess: '',
      messageInfo: ''
    }
  },
  getters: {
    isError() {
      return this.messageError != '' ? true : false
    },
    isWarning() {
      return this.messageWarning != '' ? true : false
    },
    isSuccess() {
      return this.messageSuccess != '' ? true : false
    },
    isInfo() {
      return this.messageInfo != '' ? true : false
    },
    uiVersion() {
      logDebug('globalStore.uiVersion()', import.meta.env.VITE_APP_VERSION)
      return import.meta.env.VITE_APP_VERSION
    },
    uiBuild() {
      logDebug('globalStore.uiBuild()', import.meta.env.VITE_APP_BUILD)
      return import.meta.env.VITE_APP_BUILD
    }
  },
  actions: {
    clearMessages() {
      this.messageError = ''
      this.messageWarning = ''
      this.messageSuccess = ''
      this.messageInfo = ''
    },
    async load() {
      try {
        logInfo('globalStore.load()', 'Fetching /api/feature')
        const json = await http.getJson('api/feature')
        logDebug('globalStore.load()', json)

        this.board = json.board.toUpperCase()
        this.app_ver = json.app_ver
        this.app_build = json.app_build
        this.platform = json.platform.toUpperCase()
        this.firmware_file = json.firmware_file.toLowerCase()

        // Only update feature flags if they are present in the response
        if (json.ble !== undefined) {
          this.feature.ble = json.ble
        }
        if (json.ble_sensor !== undefined) {
          this.feature.ble_sensor = json.ble_sensor
        }

        logInfo('globalStore.load()', 'Fetching /api/feature completed')
        return true
      } catch (err) {
        logError('globalStore.load()', err)
        return false
      }
    }
  }
})
